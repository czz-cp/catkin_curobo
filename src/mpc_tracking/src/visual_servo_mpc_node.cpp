#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <vector>
#include "mpc_tracking/mpc.h"

class VisualServoMPC {
private:
    ros::NodeHandle nh;
    ros::Publisher velocity_pub;
    ros::Subscriber joint_sub;
    
    std::unique_ptr<Mpc> mpc_ptr;
    
    // 视觉伺服相关
    Eigen::Vector6d current_pose_;      // 当前末端位姿 [x,y,z,rx,ry,rz]
    Eigen::Vector6d desired_pose_;      // 期望末端位姿
    bool pose_initialized_;
    
    // 图像特征相关
    std::vector<Eigen::Vector2d> current_features_;  // 当前特征点
    std::vector<Eigen::Vector2d> desired_features_;  // 期望特征点
    
public:
    VisualServoMPC() : pose_initialized_(false) {
        velocity_pub = nh.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
        joint_sub = nh.subscribe("/joint_states", 10, &VisualServoMPC::jointStateCallback, this);
        
        // 初始化MPC
        mpc_ptr = std::make_unique<Mpc>();
        
        // 设置视觉伺服专用权重
        std::vector<double> weights = {15, 15, 20,   // x,y,z位置权重（z方向更重要）
                                       8, 8, 10,     // 姿态权重
                                       2, 2, 3,      // 线速度权重
                                       1, 1, 1};     // 角速度权重
        mpc_ptr->setWeights(weights);
        
        // 初始化期望特征点（与你的视觉伺服代码一致）
        setupDesiredFeatures();
        
        ROS_INFO("Visual Servo MPC 初始化完成");
    }
    
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 从关节状态计算末端位姿（使用你的正向运动学）
        current_pose_ = computeForwardKinematics(msg);
        pose_initialized_ = true;
    }
    
    Eigen::Vector6d computeForwardKinematics(const sensor_msgs::JointState::ConstPtr& msg) {
        Eigen::Vector6d pose = Eigen::Vector6d::Zero();
        
        // 这里调用你的正向运动学函数
        // 可以使用UR官方运动学库或你的现有实现
        std::vector<double> joint_angles = parseJointStateData(*msg);
        
        // 调用正向运动学计算末端位姿
        // pose = your_forward_kinematics_function(joint_angles);
        
        return pose;
    }
    
    std::vector<double> parseJointStateData(const sensor_msgs::JointState& msg) {
        // 你的关节状态解析代码
        std::vector<std::string> joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        
        std::vector<double> joint_angles(6, 0.0);
        for (int i = 0; i < 6; i++) {
            auto it = std::find(msg.name.begin(), msg.name.end(), joint_names[i]);
            if (it != msg.name.end()) {
                int index = std::distance(msg.name.begin(), it);
                joint_angles[i] = msg.position[index];
            }
        }
        return joint_angles;
    }
    
    void setupDesiredFeatures() {
        // 设置期望特征点（与你的视觉伺服代码一致）
        desired_features_.resize(4);
        desired_features_[0] = Eigen::Vector2d(326.0, 300.0);  // 左上
        desired_features_[1] = Eigen::Vector2d(408.0, 300.0);  // 右上
        desired_features_[2] = Eigen::Vector2d(408.0, 382.0);  // 右下
        desired_features_[3] = Eigen::Vector2d(326.0, 382.0);  // 左下
    }
    
    void updateCurrentFeatures(const std::vector<vpImagePoint>& points) {
        // 从ViSP特征点更新当前特征
        current_features_.resize(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            current_features_[i] = Eigen::Vector2d(points[i].get_u(), points[i].get_v());
        }
    }
    
    Eigen::Vector6d computeDesiredPoseFromFeatures() {
        // 基于特征点误差计算期望位姿变化
        // 这里可以使用你的图像雅可比矩阵方法
        Eigen::Vector6d delta_pose = Eigen::Vector6d::Zero();
        
        if (current_features_.size() == 4 && desired_features_.size() == 4) {
            // 计算特征点误差
            Eigen::VectorXd error(8);
            for (int i = 0; i < 4; i++) {
                error(2*i) = current_features_[i](0) - desired_features_[i](0);
                error(2*i+1) = current_features_[i](1) - desired_features_[i](1);
            }
            
            // 简化的位姿调整（实际中应该使用图像雅可比矩阵）
            double error_norm = error.norm();
            if (error_norm > 10.0) {  // 误差较大时才调整
                delta_pose(0) = -error.segment<4>(0).mean() * 0.001;  // x调整
                delta_pose(1) = -error.segment<4>(2).mean() * 0.001;  // y调整
                delta_pose(2) = -error_norm * 0.0005;                 // z调整
            }
        }
        
        return current_pose_ + delta_pose;
    }
    
    bool solveVisualServoMPC() {
        if (!pose_initialized_) return false;
        
        // 基于特征点计算期望位姿
        desired_pose_ = computeDesiredPoseFromFeatures();
        
        // 构建期望轨迹（简单的恒定期望值）
        Eigen::MatrixXd desired_trajectory(6, mpc_ptr->getPredictionSteps() + 1);
        for (int i = 0; i <= mpc_ptr->getPredictionSteps(); i++) {
            desired_trajectory.col(i) = desired_pose_;
        }
        
        return mpc_ptr->solve(current_pose_, desired_trajectory);
    }
    
    geometry_msgs::Twist getControlCommand() {
        geometry_msgs::Twist twist;
        
        if (mpc_ptr->isSolved()) {
            std::vector<double> control = mpc_ptr->getFirstU();
            
            twist.linear.x = control[0];
            twist.linear.y = control[1];
            twist.linear.z = control[2];
            twist.angular.x = control[3];
            twist.angular.y = control[4];
            twist.angular.z = control[5];
        } else {
            // MPC求解失败时发布零速度
            twist.linear.x = twist.linear.y = twist.linear.z = 0;
            twist.angular.x = twist.angular.y = twist.angular.z = 0;
        }
        
        return twist;
    }
    
    void limitVelocity(geometry_msgs::Twist& twist) {
        // 速度限制（与你的现有代码一致）
        double linear_norm = sqrt(twist.linear.x*twist.linear.x + 
                                 twist.linear.y*twist.linear.y + 
                                 twist.linear.z*twist.linear.z);
        double max_linear = 0.1;
        
        if (linear_norm > max_linear) {
            double scale = max_linear / linear_norm;
            twist.linear.x *= scale;
            twist.linear.y *= scale;
            twist.linear.z *= scale;
        }
        
        double angular_norm = sqrt(twist.angular.x*twist.angular.x + 
                                  twist.angular.y*twist.angular.y + 
                                  twist.angular.z*twist.angular.z);
        double max_angular = 0.5;
        
        if (angular_norm > max_angular) {
            double scale = max_angular / angular_norm;
            twist.angular.x *= scale;
            twist.angular.y *= scale;
            twist.angular.z *= scale;
        }
    }
    
    void run() {
        ros::Rate rate(30);  // 30Hz，匹配相机帧率
        
        ROS_INFO("视觉伺服MPC节点启动");
        
        while (ros::ok()) {
            if (pose_initialized_) {
                if (solveVisualServoMPC()) {
                    geometry_msgs::Twist cmd = getControlCommand();
                    limitVelocity(cmd);
                    velocity_pub.publish(cmd);
                    
                    // 发布调试信息
                    Eigen::Vector6d error = desired_pose_ - current_pose_;
                    ROS_DEBUG_THROTTLE(1.0, "位姿误差范数: %.4f", error.norm());
                }
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_servo_mpc");
    
    try {
        VisualServoMPC visual_servo_mpc;
        visual_servo_mpc.run();
    } catch (const std::exception& e) {
        ROS_ERROR("视觉伺服MPC失败: %s", e.what());
        return -1;
    }
    
    return 0;
}