#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import tf.transformations as transformations  # 新增四元数转换依赖
from tf2_ros import TransformBroadcaster  # 新增TF广播依赖
from geometry_msgs.msg import PoseStamped,TransformStamped


def rodrigues_rotation(r, theta):
    # n旋转轴[3x1]
    # theta为旋转角度
    # 旋转是过原点的，n是旋转轴
    r = np.array(r).reshape(3, 1)
    rx, ry, rz = r[:, 0]
    M = np.array([
        [0, -rz, ry],
        [rz, 0, -rx],
        [-ry, rx, 0]
    ])
    R = np.zeros([3,3])
    R[:3, :3] = np.cos(theta) * np.eye(3) +        \
                (1 - np.cos(theta)) * r @ r.T +    \
                np.sin(theta) * M
    return R

def rodrigues_rotation_vec_to_R(v):
    # r旋转向量[3x1]
    theta = np.linalg.norm(v)
    # print(theta)
    r = np.array(v).reshape(3, 1) / theta
    # print(r)
    return rodrigues_rotation(r, theta)

class ArucoDetector:
    def __init__(self):
        # self.k4a.start()
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera")
            raise IOError("Failed to open camera")
        self.pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
        
        # self.camera_matrix = np.array([609.0780029296875, 0, 608.9701538085938,
        #                 0, 640.1692504882812, 366.8263244628906,
        #                 0, 0, 1]).reshape(3,3)

        self.camera_matrix = np.array([[605.99646, 0., 334.4359],
                                       [0., 605.43365, 243.74963],
                                       [0., 0., 1.]])
        
        dist = np.array([0.00515398, -0.00872068, 0.000730499, 0.000393782, 0.0000648475])
        self.dist_coeffs = dist[0:5].reshape(1,5)
        
        # ArUco参数配置（修复参数命名错误）
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.parameters = aruco.DetectorParameters()
        self.arucodetector = aruco.ArucoDetector(dictionary=self.dictionary, detectorParams=self.parameters)  # 修复参数名arucoDict -> dictionary
        self.parameters.adaptiveThreshConstant = 7  # 增强动态光照适应性
        self.tf_broadcaster = TransformBroadcaster()

    def run(self):
        rospy.loginfo("Starting ArUco detector loop")
        rate = rospy.Rate(100) 
        while not rospy.is_shutdown():
            try:
                ret,capture = self.cap.read()
                if not ret:
                    rospy.logwarn("Failed to read frame from camera")
                    continue
                if capture is not None:
                    # 转换为灰度图并检测ArUco
                    cv2.imwrite("/tmp/aruco_capture.png", capture)
                    # gray = cv2.cvtColor(capture, cv2.COLOR_BGRA2GRAY)
                    corners, ids, _ = self.arucodetector.detectMarkers(capture)
                    # print(ids)
                    if ids is not None:
                        pose = self._estimate_pose(corners[0])#4个角点 (x,y)
                        pose.header.stamp = rospy.Time.now()  # 显式同步时间戳
                        pose.header.frame_id = "aruco_marker"
                        
                        self.pub.publish(pose)
                        self._publish_tf_transform(pose)

                    else:
                        rospy.loginfo("No ArUco markers detected.")
                rate.sleep()
            except Exception as e:
                rospy.logerr(f"ArUco detection error: {e}")

    def _estimate_pose(self, corners):
        # 定义ArUco标记尺寸（单位：米）
        rvec, tvec,_ = cv2.aruco.estimatePoseSingleMarkers(corners, 100, self.camera_matrix, self.dist_coeffs)
        # 计算PnP解
        # 转换为ROS PoseStamped消息
        pose = PoseStamped()
        pose.pose.position.x = tvec[0,0,0]/1000
        pose.pose.position.y = tvec[0,0,1]/1000
        pose.pose.position.z = tvec[0,0,2]/1000
        print("rec",rvec)
        # 将旋转向量转换为四元数
        R = rodrigues_rotation_vec_to_R(rvec[0][0])
        # print(R)
        R_4x4 = np.eye(4)
        R_4x4[:3, :3] = R
        quat = transformations.quaternion_from_matrix(R_4x4)
        # print("quat",quat)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        # print(pose)
        # print(pose)
        return pose

    def _publish_tf_transform(self, pose):
            """发布TF变换"""
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "camera_link"  # 相机坐标系
            transform.child_frame_id = "aruco_marker"  # 标记坐标系
            
            # 设置平移
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
        
            
            transform.transform.rotation.x = pose.pose.orientation.x
            transform.transform.rotation.y = pose.pose.orientation.y
            transform.transform.rotation.z = pose.pose.orientation.z
            transform.transform.rotation.w = pose.pose.orientation.w
            
            # 发布变换
            self.tf_broadcaster.sendTransform(transform)
        
if __name__ == "__main__":
    rospy.init_node("aruco_detector")
    detector = ArucoDetector()
    detector.run()