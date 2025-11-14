/**
 * UR10e with D435i Visual Servo Controller
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <librealsense2/rs.hpp>

// ViSP 相关头文件
#include <visp3/core/vpImage.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpImageConvert.h>

// 添加UR机器人状态解析相关的头文件
#include <vector>
#include <algorithm>

// UR官方运动学库
#include <ur_kinematics/ur_kin.h>

// OpenCV ArUco相关头文件
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>

class UR10eD435iVisualServo {
private:
    ros::NodeHandle nh;
    ros::Publisher velocity_pub;
    ros::Subscriber joint_sub;
    
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    
    // 图像处理相关
    vpRealSense2 g;
    vpImage<unsigned char> I;//存储当前帧的灰度图像
    vpCameraParameters cam;
    vpDisplayX *display;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
    std::vector<int> aruco_ids;
    std::vector<std::vector<cv::Point2f>> aruco_corners;
    
    // 特征点跟踪
    std::vector<vpDot2> blobs;
    std::vector<vpImagePoint> current_points;
    std::vector<vpImagePoint> desired_points;
    std::vector<vpPoint> object_points;
    
    // 控制参数
    double lambda;
    double max_linear_vel;
    double max_angular_vel;

    // 自适应增益参数
    double lambda_0;      // 初始增益
    double lambda_inf;    // 稳态增益  
    double lambda_0l;     // 衰减系数
    double mu;           // 时间衰减系数
    ros::Time start_time;// 控制开始时间
    
    // 手眼标定参数（与graspnet代码保持一致）
    Eigen::Matrix3d handeye_rot;  // 相机到末端的旋转矩阵
    Eigen::Vector3d handeye_trans; // 相机到末端的平移向量
    
    // 关节状态
    Eigen::Matrix<double, 6, 1> q_state;
    Eigen::Matrix<double, 6, 1> dq_state;
    Eigen::Matrix<double, 6, 1> current_cartesian_pose; // 当前末端位姿 [x, y, z, rx, ry, rz]
    
    // 图像处理参数
    const int Np = 4; // 4个特征点
    double opt_square_width;
    double L;
    double aruco_marker_size; // ArUco码尺寸（米）
    
    // 状态标志
    bool send_velocities;
    bool tracking_initialized;
    bool display_enabled;

public:
    UR10eD435iVisualServo() : 
        tf_listener(tf_buffer),
        display(NULL),
        send_velocities(false),
        tracking_initialized(false),
        display_enabled(true) {
        
        // 参数加载
        nh.param("lambda", lambda, 0.8);
        nh.param("max_linear_vel", max_linear_vel, 0.1);
        nh.param("max_angular_vel", max_angular_vel, 0.5);
        nh.param("square_width", opt_square_width, 0.15);//目标宽度
        nh.param("display_enabled", display_enabled, true);
        nh.param("aruco_marker_size", aruco_marker_size, 0.15);//目标宽度
        
        L = opt_square_width / 2.0;
        
        // 初始化发布器和订阅器
        velocity_pub = nh.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
        joint_sub = nh.subscribe("/joint_states", 10, &UR10eD435iVisualServo::jointStateCallback, this);
        
        // 设置手眼标定参数（与graspnet代码保持一致）
        setupHandEyeCalibration();
        
        // 初始化相机和视觉
        initializeCamera();
        initializeBlobs();
        setupObjectPoints();
        setupDesiredPoints();
        initializeAdaptiveGain();
        
        ROS_INFO("UR10e with D435i Visual Servo initialized");
        ROS_INFO("Hand-eye calibration loaded");
        ROS_INFO("Rotation matrix:\n%f %f %f\n%f %f %f\n%f %f %f", 
                handeye_rot(0,0), handeye_rot(0,1), handeye_rot(0,2),
                handeye_rot(1,0), handeye_rot(1,1), handeye_rot(1,2),
                handeye_rot(2,0), handeye_rot(2,1), handeye_rot(2,2));
        ROS_INFO("Translation vector: [%f, %f, %f]", 
                handeye_trans(0), handeye_trans(1), handeye_trans(2));
    }
    
    ~UR10eD435iVisualServo() {
        if (display) {
            delete display;
        }
    }
    
    void setupHandEyeCalibration() {
        // 使用与graspnet代码相同的手眼标定参数
        // rotation_matrix 和 translation_vector 从grasp.py中获取
        handeye_rot << 0.7701994, 0.63741788, -0.02216594,
                      -0.63772072, 0.77019341, -0.01069532,
                       0.01025467, 0.02237321, 0.99969709;
        
        handeye_trans << -0.06330511, -0.03645401, 0.05061932;
    }
    
    void initializeCamera() {
        try {
            rs2::config config;
            config.disable_stream(RS2_STREAM_DEPTH);
            config.disable_stream(RS2_STREAM_INFRARED);
            config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
            g.open(config);
            g.acquire(I);
            
            // 获取相机内参
            cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
            
            // 初始化显示
            if (display_enabled) {
                display = new vpDisplayX(I, 100, 100, "UR10e Visual Servo - D435i");
                vpDisplay::setTitle(I, "UR10e Visual Servo - Press Click to Start/Stop");
            }
            
            ROS_INFO("D435i Camera initialized");
            ROS_INFO("Camera parameters: fx=%f, fy=%f, u0=%f, v0=%f", 
                    cam.get_px(), cam.get_py(), cam.get_u0(), cam.get_v0());
                    
        } catch (const vpException &e) {
            ROS_ERROR("Camera initialization failed: %s", e.getMessage());
            throw;
        }
    }
    
    void initializeBlobs() {
        blobs.resize(Np);
        current_points.resize(Np);
        desired_points.resize(Np);
        
        for (int i = 0; i < Np; i++) {
            blobs[i].setGraphics(true);
            blobs[i].setGraphicsThickness(1);
            
            // 使用正确的 ViSP 函数名
            //blobs[i].setComputeMoments(false);  // 关闭矩计算
            //blobs[i].setEllipsoidShapePrecision(0.7);  // 降低形状精度要求
            // 边缘阈值设置（如果函数存在）
            // blobs[i].setFirstEdgeThreshold(0.5);  // 这个函数可能不存在，先注释掉
            // blobs[i].setEdgeThreshold(0.5);       // 这个函数可能不存在，先注释掉
        }
    }
    
    void setupObjectPoints() {
        // 设置目标物体的3D点（世界坐标系）
        object_points.clear();
        object_points.push_back(vpPoint(-L, -L, 0));  // 左下
        object_points.push_back(vpPoint( L, -L, 0));  // 右下
        object_points.push_back(vpPoint( L,  L, 0));  // 右上
        object_points.push_back(vpPoint(-L,  L, 0));  // 左上
    }
    
    
    void setupDesiredPoints() {
        // 设置更合理的期望点位置（图像中心区域）
        int center_u = I.getWidth() / 2;
        int center_v = I.getHeight() / 2;
        int offset = 65;  // 偏移量
        
        desired_points[0] = vpImagePoint(center_v - offset, center_u - offset); // 左上
        desired_points[1] = vpImagePoint(center_v - offset, center_u + offset); // 右上  
        desired_points[2] = vpImagePoint(center_v + offset, center_u + offset); // 右下
        desired_points[3] = vpImagePoint(center_v + offset, center_u - offset); // 左下
        
        ROS_INFO("Desired points set around image center");
    }
    
    // 解析关节状态数据（模仿UR_Robot.py中的parse_tcp_state_data）
    std::vector<double> parseJointStateData(const sensor_msgs::JointState& msg) {
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
    
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 更新关节状态
        std::vector<std::string> joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        
        for (int i = 0; i < 6; i++) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
            if (it != msg->name.end()) {
                int index = std::distance(msg->name.begin(), it);
                q_state(i) = msg->position[index];
                dq_state(i) = msg->velocity[index];
            }
        }
        
        
        // 这里可以添加正向运动学计算来获取当前末端位姿
        // 由于视觉伺服主要依赖图像特征，我们暂时使用关节角度
    }
    
    // 从RPY计算旋转矩阵（模仿UR_Robot.py中的rpy2R）
    Eigen::Matrix3d rpyToRotationMatrix(double rx, double ry, double rz) {
        Eigen::Matrix3d rot_x, rot_y, rot_z;
        
        // X轴旋转
        rot_x << 1, 0, 0,
                0, cos(rx), -sin(rx),
                0, sin(rx), cos(rx);
        
        // Y轴旋转
        rot_y << cos(ry), 0, sin(ry),
                0, 1, 0,
                -sin(ry), 0, cos(ry);
        
        // Z轴旋转
        rot_z << cos(rz), -sin(rz), 0,
                sin(rz), cos(rz), 0,
                0, 0, 1;
        
        return rot_z * rot_y * rot_x;
    }
    
    // 从旋转矩阵计算RPY（模仿UR_Robot.py中的R2rpy）
    Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
        double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        
        double x, y, z;
        if (!singular) {
            x = atan2(R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2(R(1,0), R(0,0));
        } else {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }
        
        return Eigen::Vector3d(x, y, z);
    }
    
    // 位姿转齐次矩阵（模仿UR_Robot.py中的pose2matrix）
    Eigen::Matrix4d poseToMatrix(const Eigen::Matrix<double, 6, 1>& pose) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        // 平移部分
        T(0,3) = pose(0);
        T(1,3) = pose(1);
        T(2,3) = pose(2);
        
        // 旋转部分（从RPY计算旋转矩阵）
        T.block<3,3>(0,0) = rpyToRotationMatrix(pose(3), pose(4), pose(5));
        
        return T;
    }
    
    Eigen::Matrix4d getEndEffectorToBaseTransform() {
        // 使用UR官方运动学库计算正向运动学
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        // 将Eigen向量转换为double数组（UR运动学库需要的格式）
        double q[6];
        for (int i = 0; i < 6; i++) {
            q[i] = q_state(i);
        }
        
        // 使用UR10e运动学参数计算正向运动学
        double T_array[16]; // 4x4矩阵，按行优先存储
        
        // 调用UR官方运动学库的正向运动学函数
        ur_kinematics::forward(q, T_array);
        
        // 将结果转换为Eigen矩阵（注意：UR库返回的是行优先，Eigen默认列优先）
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                T(i, j) = T_array[i * 4 + j];
            }
        }
        
        return T;
    }

    Eigen::Matrix4d getCameraToEndEffectorTransform() {
        // 构建相机到末端执行器的变换矩阵
        Eigen::Matrix4d T_cam2ee = Eigen::Matrix4d::Identity();
        T_cam2ee.block<3,3>(0,0) = handeye_rot;
        T_cam2ee.block<3,1>(0,3) = handeye_trans;
        return T_cam2ee;
    }
    
    void computeImageJacobian(const std::vector<vpImagePoint>& points, 
                             const std::vector<double>& depths,
                             Eigen::Matrix<double, 8, 6>& L) {
        L = Eigen::Matrix<double, 8, 6>::Zero();
        
        double fx = cam.get_px();
        double fy = cam.get_py();
        double u0 = cam.get_u0();
        double v0 = cam.get_v0();
        
        for (int i = 0; i < Np; i++) {
            double u = points[i].get_u();
            double v = points[i].get_v();
            double Z = depths[i];
            
            if (Z <= 0) Z = 0.5; // 深度安全检查
            
            // 归一化坐标
            double x = (u - u0) / fx;
            double y = (v - v0) / fy;
            
            // 图像雅可比矩阵（2×6）
            Eigen::Matrix<double, 2, 6> L_i;
            L_i << -1/Z,     0,  x/Z,  x*y,   -(1+x*x),   y,
                    0,    -1/Z,  y/Z,  1+y*y,   -x*y,    -x;
            
            // 填充到总雅可比矩阵
            L.block<2,6>(2*i, 0) = L_i;
        }
    }
    
    bool trackBlobs() {
        static int initialized_count = 0;  // 静态变量记录已初始化的blob数量
        
        if (!tracking_initialized) {
            if (initialized_count >= Np) {
                tracking_initialized = true;
                ROS_INFO("All %d blobs initialized, tracking started", Np);
                return true;
            }
            
            try {
                // 显示当前要初始化的blob编号
                if (display_enabled) {
                    vpDisplay::displayText(I, 100 + initialized_count * 20, 20, 
                        "Click to initialize blob " + std::to_string(initialized_count), 
                        vpColor::red);
                    vpDisplay::flush(I);
                }
                
                vpImagePoint germ;
                if (vpDisplay::getClick(I, germ, false)) {
                    blobs[initialized_count].initTracking(I, germ);
                    current_points[initialized_count] = blobs[initialized_count].getCog();
                    ROS_INFO("Blob %d initialized at (%.1f, %.1f)", 
                            initialized_count, germ.get_u(), germ.get_v());
                    
                    initialized_count++;  // 增加已初始化计数
                }
            } catch (...) {
                ROS_WARN("Blob tracking failed for point %d", initialized_count);
            }
            
            return false;  // 还没有全部初始化完成
        }
        
        // 正常跟踪模式
        bool all_tracked = true;
        for (int i = 0; i < Np; i++) {
            try {
                blobs[i].track(I);
                current_points[i] = blobs[i].getCog();
            } catch (...) {
                all_tracked = false;
                ROS_WARN("Blob tracking failed for point %d", i);
            }
        }
        return all_tracked;
    }

    void estimateDepths(std::vector<double>& depths) {
        depths.clear();
        depths.resize(Np, 0.5);  // 使用固定深度值
        
        // 完全简化深度估计，避免RealSense错误
        for (int i = 0; i < Np; i++) {
            depths[i] = 0.6;  // 50cm固定深度
        }
        
        // 可选：添加简单的深度估计逻辑（如果需要）
        // 但暂时使用固定值避免RealSense错误
    }
    
    double getAverageDepthAround(const vpImage<uint16_t>& depth_map, int u, int v, int window_size) {
        int half_window = window_size / 2;
        double sum = 0.0;
        int count = 0;
        
        for (int i = -half_window; i <= half_window; i++) {
            for (int j = -half_window; j <= half_window; j++) {
                int x = u + i;
                int y = v + j;
                
                if (x >= 0 && x < depth_map.getWidth() && 
                    y >= 0 && y < depth_map.getHeight()) {
                    
                    double depth_val = depth_map[y][x] / 1000.0;
                    if (depth_val > 0.1 && depth_val < 3.0) {
                        sum += depth_val;
                        count++;
                    }
                }
            }
        }
        
        return count > 0 ? sum / count : 0.5;
    }
    
    /*
     * Pseudo inverse image jacobian.
     */
    void pinv(Eigen::Matrix<double, 8, 6>& L, Eigen::Matrix<double, 6, 8>& pinvL, double alpha0 = 0.001, double w0 = 0.0001) {
        double w = 0, alpha = 0;

        double detL = (L * L.transpose()).determinant();
        if (detL < 1.0e-10) {
            w = 1.0e-5;
        } else {
            w = sqrt(detL);
        }

        if (w >= w0) {
            alpha = 0;
        } else {
            alpha = alpha0 * (1.0 - w / w0) * (1 - w / w0);
        }

        // 6x8 = 6x8 * (8x6 * 6x8)
        pinvL = L.transpose() * (L * L.transpose() - alpha * Eigen::MatrixXd::Identity(8, 8)).inverse();
    }


    void computeVelocityCommand() {
        try {
            // 获取新图像帧
            g.acquire(I);
            
            if (display_enabled) {
                vpDisplay::display(I);
                
                // 显示控制状态
                std::string status = send_velocities ? "CONTROL ACTIVE" : "CONTROL PAUSED";
                vpDisplay::displayText(I, 20, 20, status, vpColor::red);
                vpDisplay::displayText(I, 40, 20, "Left click: Start/Stop", vpColor::green);
                vpDisplay::displayText(I, 60, 20, "Right click: Reset tracking", vpColor::green);
            }

                 // 获取当前末端执行器位姿
                 Eigen::Matrix4d T_ee2base = getEndEffectorToBaseTransform();
                 Eigen::Matrix4d T_cam2ee = getCameraToEndEffectorTransform();
                 Eigen::Matrix4d T_cam2base = T_ee2base * T_cam2ee;
            
            // 跟踪特征点
            if (!trackBlobs()) {
                publishZeroVelocity();
                if (display_enabled) vpDisplay::flush(I);
                return;
            }
            
            // 估计深度
            std::vector<double> depths(Np);
            estimateDepths(depths);
            
            // 计算图像雅可比矩阵
            Eigen::Matrix<double, 8, 6> L_image;
            computeImageJacobian(current_points, depths, L_image);
            
            // 计算特征误差
            Eigen::Matrix<double, 8, 1> error;
            for (int i = 0; i < Np; i++) {
                error(2*i) = current_points[i].get_u() - desired_points[i].get_u();
                error(2*i+1) = current_points[i].get_v() - desired_points[i].get_v();
                
                // 在图像上显示误差
                if (display_enabled) {
                    vpDisplay::displayLine(I, current_points[i], desired_points[i], 
                                         vpColor(255, 128, 0), 2);
                    vpDisplay::displayText(I, current_points[i], std::to_string(i), vpColor::white);
                }
            }
            
            double error_norm = error.norm();
            
            // 计算相机速度（伪逆方法）
            //Eigen::Matrix<double, 6, 8> L_pinv = L_image.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::Matrix<double, 6, 8> L_pinv;
            pinv(L_image, L_pinv);
            //Eigen::JacobiSVD<Eigen::Matrix<double, 8, 6>> svd(L_image, Eigen::ComputeThinU | Eigen::ComputeThinV);
            //Eigen::Matrix<double, 6, 8> L_pinv = svd.matrixV() * 
            //svd.singularValues().array().inverse().matrix().asDiagonal() * 
            //svd.matrixU().adjoint();
            //Eigen::Matrix<double, 6, 1> v_camera = -lambda * L_pinv * error;】
            Eigen::Matrix<double, 6, 1> v_camera = computeAdaptiveVelocity(L_pinv, error);
            
            // 限制Z轴速度（深度方向不稳定）
            v_camera(2) = 0.0;
            
            // 转换到末端执行器坐标系（使用手眼标定）
            Eigen::Matrix<double, 6, 1> v_end_effector;
            v_end_effector.head<3>() = handeye_rot * v_camera.head<3>();
            v_end_effector.tail<3>() = handeye_rot * v_camera.tail<3>();
            
            // 发布速度命令或零速度
            if (send_velocities && error_norm > 5.0) {
                publishVelocityCommand(v_end_effector);
                ROS_DEBUG_THROTTLE(1.0, "Error norm: %.3f, Sending velocities", error_norm);
            } else {
                publishZeroVelocity();
                if (error_norm <= 5.0) {
                    ROS_INFO_THROTTLE(2.0, "Target reached! Error norm: %.3f", error_norm);
                }
            }
            
            // 处理用户输入
            handleUserInput();
            
            if (display_enabled) {
                vpDisplay::flush(I);
            }
            
        } catch (const vpException &e) {
            ROS_ERROR("Visual servo error: %s", e.getMessage());
            publishZeroVelocity();
        }
    }
    
    void initializeAdaptiveGain() {
        lambda_0 = 1.2;    // 较大的初始增益，快速响应
        lambda_inf = 0.3;  // 较小的稳态增益，避免振荡
        lambda_0l = 2.0;   // 速度相关的衰减系数
        mu = 0.5;         // 时间衰减系数
        start_time = ros::Time::now();
    }

    Eigen::Matrix<double, 6, 1> computeAdaptiveVelocity(
        const Eigen::Matrix<double, 6, 8>& L_pinv, 
        const Eigen::Matrix<double, 8, 1>& error) {
        
        // 基础伪逆控制
        Eigen::Matrix<double, 6, 1> v_c = -L_pinv * error;
        
        // 计算自适应增益（基于速度范数）
        double v_norm = v_c.lpNorm<Eigen::Infinity>(); // 最大速度分量
        double t = (ros::Time::now() - start_time).toSec();
        
        // 自适应增益公式
        lambda = (lambda_0 - lambda_inf) * exp(-lambda_0l * v_norm / (lambda_0 - lambda_inf)) + lambda_inf;
        
        // 可选：基于误差的自适应增益
        // double error_norm = error.lpNorm<Eigen::Infinity>();
        // lambda = (lambda_0 - lambda_inf) * exp(-lambda_0l * error_norm) + lambda_inf;
        
        // 应用增益
        v_c = lambda * v_c;
        
        // 可选：添加时间衰减项（用于初始阶段的平滑启动）
        // double v_c0 = 0.1; // 初始速度基准
        // v_c = v_c - lambda * v_c0 * exp(-mu * t);
        
        ROS_DEBUG_THROTTLE(0.5, "Adaptive gain: lambda=%.3f, v_norm=%.3f", lambda, v_norm);
        
        return v_c;
    }

    void handleUserInput() {
        if (!display_enabled) return;
        
        vpMouseButton::vpMouseButtonType button;
        if (vpDisplay::getClick(I, button, false)) {
            switch (button) {
                case vpMouseButton::button1:
                    send_velocities = !send_velocities;
                    ROS_INFO("Control %s", send_velocities ? "STARTED" : "STOPPED");
                    break;
                    
                case vpMouseButton::button3:
                    tracking_initialized = false;
                    ROS_INFO("Tracking reset");
                    break;
                    
                default:
                    break;
            }
        }
    }
    
    void publishVelocityCommand(const Eigen::Matrix<double, 6, 1>& v_end_effector) {
        geometry_msgs::Twist twist;
        
        // 线速度 (m/s)
        twist.linear.x = v_end_effector(0);
        twist.linear.y = v_end_effector(1);
        twist.linear.z = v_end_effector(2);
        
        // 角速度 (rad/s)
        twist.angular.x = v_end_effector(3);
        twist.angular.y = v_end_effector(4);
        twist.angular.z = v_end_effector(5);
        
        // 速度限制
        limitVelocity(twist);
        
        velocity_pub.publish(twist);
    }
    
    void publishZeroVelocity() {
        geometry_msgs::Twist twist;
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;
        velocity_pub.publish(twist);
    }
    
    void limitVelocity(geometry_msgs::Twist& twist) {
        // 限制线速度
        double linear_norm = sqrt(twist.linear.x*twist.linear.x + 
                                 twist.linear.y*twist.linear.y + 
                                 twist.linear.z*twist.linear.z);
        if (linear_norm > max_linear_vel) {
            double scale = max_linear_vel / linear_norm;
            twist.linear.x *= scale;
            twist.linear.y *= scale;
            twist.linear.z *= scale;
        }
        
        // 限制角速度
        double angular_norm = sqrt(twist.angular.x*twist.angular.x + 
                                  twist.angular.y*twist.angular.y + 
                                  twist.angular.z*twist.angular.z);
        if (angular_norm > max_angular_vel) {
            double scale = max_angular_vel / angular_norm;
            twist.angular.x *= scale;
            twist.angular.y *= scale;
            twist.angular.z *= scale;
        }
    }
    
    void run() {
        ros::Rate rate(30); // 30Hz，匹配D435i帧率
        
        ROS_INFO("Visual servo node started");
        ROS_INFO("Click on the image window to initialize blob tracking");
        
        while (ros::ok()) {
            computeVelocityCommand();
            ros::spinOnce();
            rate.sleep();
        }
        
        // 退出时发布零速度
        publishZeroVelocity();
        ROS_INFO("Visual servo node shutdown");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur10e_d435i_visual_servo");
    
    try {
        UR10eD435iVisualServo servo;
        servo.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Visual servo failed: %s", e.what());
        return -1;
    }
    
    return 0;
}