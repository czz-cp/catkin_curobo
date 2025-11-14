/**
 * UR10e with D435i Visual Servo Controller - ArUco码追踪版本（修复版）
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
 #include <visp3/vision/vpPose.h>
 #include <visp3/core/vpPixelMeterConversion.h>
 #include <visp3/core/vpPoint.h>
 #include <visp3/core/vpImageConvert.h>
 
 // OpenCV ArUco相关头文件
 #include <opencv2/opencv.hpp>
 #include <opencv2/aruco.hpp>
 #include <opencv2/aruco/dictionary.hpp>
 
 // 添加UR机器人状态解析相关的头文件
 #include <vector>
 #include <algorithm>
 
 // UR官方运动学库
 #include <ur_kinematics/ur_kin.h>
 
 class UR10eD435iVisualServo {
 private:
     ros::NodeHandle nh;
     ros::Publisher velocity_pub;
     ros::Subscriber joint_sub;
     
     tf2_ros::Buffer tf_buffer;
     tf2_ros::TransformListener tf_listener;
     
     // 图像处理相关
     vpRealSense2 g;
     vpImage<unsigned char> I; // 存储当前帧的灰度图像
    
     vpImage<unsigned char> I_depth;  

     vpImage<vpRGBa> I_color;  // 存储彩色图像用于ArUco检测
     vpCameraParameters cam;
     vpDisplayX *display;
     
     // ArUco码检测相关
     cv::Ptr<cv::aruco::Dictionary> aruco_dict;
     cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
     std::vector<int> aruco_ids;
     std::vector<std::vector<cv::Point2f>> aruco_corners;
     
     // 特征点跟踪（使用ArUco码的四个角点）
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
     ros::Time start_time; // 控制开始时间
     
     // 手眼标定参数（与graspnet代码保持一致）
     Eigen::Matrix3d handeye_rot;  // 相机到末端的旋转矩阵
     Eigen::Vector3d handeye_trans; // 相机到末端的平移向量
     
     // 关节状态
     Eigen::Matrix<double, 6, 1> q_state;
     Eigen::Matrix<double, 6, 1> dq_state;
     
     // 图像处理参数
     const int Np = 4; // 4个特征点（ArUco码的四个角点）
     double aruco_marker_size; // ArUco码尺寸（米）
     
     // 状态标志
     bool send_velocities;
     bool tracking_initialized;
     bool display_enabled;
 
 public:
     UR10eD435iVisualServo() : 
         tf_listener(tf_buffer),
         display(NULL),
         send_velocities(true),
         tracking_initialized(false){
         
         // 参数加载
         nh.param("lambda", lambda, 1.0);//0.8
         nh.param("max_linear_vel", max_linear_vel, 0.1);
         nh.param("max_angular_vel", max_angular_vel, 0.5);
         nh.param("aruco_marker_size", aruco_marker_size,0.05); // 默认10cm的ArUco码
         nh.param("display_enabled", display_enabled,true);
         
         // 初始化发布器和订阅器
         velocity_pub = nh.advertise<geometry_msgs::Twist>("/ur10e_robot/twist_controller/command", 1);
         joint_sub = nh.subscribe("/ur10e_robot/joint_states", 10, &UR10eD435iVisualServo::jointStateCallback, this);
         
         // 设置手眼标定参数（与graspnet代码保持一致）
         setupHandEyeCalibration();
         
         // 初始化相机和视觉
         initializeCamera();
         initializeArUco();
         setupObjectPoints();
         setupDesiredPoints();
         initializeAdaptiveGain();
         
         ROS_INFO("UR10e with D435i Visual Servo initialized (ArUco tracking)");
         ROS_INFO("ArUco marker size: %.3f m", aruco_marker_size);
         ROS_INFO("Hand-eye calibration loaded");
     }
     
     ~UR10eD435iVisualServo() {
         if (display) {
             delete display;
         }
     }
     
     void setupHandEyeCalibration() {
         // 使用与graspnet代码相同的手眼标定参数
         handeye_rot << 0.7701994, 0.63741788, -0.02216594,
                       -0.63772072, 0.77019341, -0.01069532,
                        0.01025467, 0.02237321, 0.99969709;
         
         handeye_trans << -0.06330511, -0.03645401, 0.05061932;
     }
     
     void initializeCamera() {
         try {
             rs2::config config;
             config.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGBA8, 30);
             //config.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30); // 启用深度流
             g.open(config);
             
             // 分别获取彩色和灰度图像
             g.acquire(I); // 先获取灰度图像
             //g.acquire(nullptr, nullptr, nullptr, I_depth.bitmap);  // 获取深度图像

             // 获取相机内参
             cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
             
             // 初始化显示
             if (display_enabled) {
                 display = new vpDisplayX(I, 100, 100, "UR10e Visual Servo - ArUco Tracking");
                 vpDisplay::setTitle(I, "UR10e Visual Servo - ArUco Tracking");
             }
             
             ROS_INFO("D435i Camera initialized");
             ROS_INFO("Camera parameters: fx=%f, fy=%f, u0=%f, v0=%f", 
                     cam.get_px(), cam.get_py(), cam.get_u0(), cam.get_v0());
                     
         } catch (const vpException &e) {
             ROS_ERROR("Camera initialization failed: %s", e.getMessage());
             throw;
         }
     }
     
  
     void initializeArUco() {
        // 使用正确的字典：DICT_4X4_50（4x4字典，最多50个标记）
        aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        aruco_params = cv::aruco::DetectorParameters::create();
        
        // 调整检测参数以适应4x4标记
        aruco_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        aruco_params->cornerRefinementWinSize = 3;  // 减小窗口大小，因为标记较小
        aruco_params->cornerRefinementMaxIterations = 30;
        aruco_params->cornerRefinementMinAccuracy = 0.1;
        
        // 设置标记尺寸（22mm = 0.022m）
        //aruco_marker_size = 0.05;  // 22mm
        
        current_points.resize(Np);
        desired_points.resize(Np);
        
        ROS_INFO("ArUco detector initialized for DICT_4X4_50, ID 17, Size: 22mm");
    }
     
     void setupObjectPoints() {
         // 设置ArUco码的3D点（以码中心为原点）
         double half_size = aruco_marker_size / 2.0;
         object_points.clear();
         object_points.push_back(vpPoint(-half_size, -half_size, 0));  // 左下
         object_points.push_back(vpPoint( half_size, -half_size, 0));  // 右下
         object_points.push_back(vpPoint( half_size,  half_size, 0));  // 右上
         object_points.push_back(vpPoint(-half_size,  half_size, 0));  // 左上
     }
     
     void setupDesiredPoints() {
         // 设置期望点位置（图像中心区域）
         int center_u = I.getWidth() / 2;
         int center_v = I.getHeight() / 2;
         int offset = static_cast<int>(aruco_marker_size * 2500);  // 根据码大小调整偏移
         ROS_WARN("ArUco Size: %.3f m, Offset: %d pixels", aruco_marker_size, offset);

         //offset = std::max(30, std::min(offset, 100));  // 限制在30-80像素之间
         
         desired_points[0] = vpImagePoint(center_v - offset, center_u - offset); // 左上
         desired_points[1] = vpImagePoint(center_v - offset, center_u + offset); // 右上  
         desired_points[2] = vpImagePoint(center_v + offset, center_u + offset); // 右下
         desired_points[3] = vpImagePoint(center_v + offset, center_u - offset); // 左下

          // 在图像上绘制期望位置（用于调试）
    if (display_enabled) {
        vpDisplay::display(I);
        
        // 绘制中心十字
        vpDisplay::displayCross(I, center_v, center_u, 15, vpColor::green, 2);
        
        // 绘制期望的矩形区域
        for (int i = 0; i < 4; i++) {
            int next_i = (i + 1) % 4;
            vpDisplay::displayLine(I, desired_points[i], desired_points[next_i], 
                                 vpColor::green, 2);
            vpDisplay::displayText(I, desired_points[i], 
                                 "D" + std::to_string(i), vpColor::green);
        }
        
        vpDisplay::flush(I);
        ros::Duration(1.0).sleep();  // 显示1秒用于调试
    }
         
         ROS_INFO("Desired points set around image center");
     }
     
     void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
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
     }
     
     
     // 检测ArUco码并提取角点
     bool detectArUcoMarkers() {
         try {
             // 将ViSP灰度图像转换为OpenCV图像
             cv::Mat cv_image(I.getHeight(), I.getWidth(), CV_8UC1, I.bitmap);
             
             // 检测ArUco码
             std::vector<std::vector<cv::Point2f>> corners;
             std::vector<int> ids;
             
             cv::aruco::detectMarkers(cv_image, aruco_dict, corners, ids, aruco_params);
             
             if (ids.empty()) {
                 tracking_initialized = false;
                 return false;
             }
             
             // 找到第一个检测到的码（可以扩展为处理多个码）
             // 只检测特定ID的标记（例如ID 17）
                int target_id = 17;
                int marker_index = 17;
        
            // 查找目标ID的索引
            /*for (size_t i = 0; i < ids.size(); i++) {
                if (ids[i] == target_id) {
                    marker_index = i;
                    break;
                }
            }
        
            // 如果没有找到目标ID，返回失败
            if (marker_index == -1) {
                tracking_initialized = false;
                return false;
            }*/
             
             // 提取角点坐标并转换为ViSP格式
             for (int i = 0; i < 4; i++) {
                 current_points[i].set_u(corners[marker_index][i].x);
                 current_points[i].set_v(corners[marker_index][i].y);
             }
             
             tracking_initialized = true;
             
             // 在图像上显示检测结果（使用灰度图像显示）
             if (display_enabled) {
                 // 在灰度图像上绘制检测结果
                 cv::aruco::drawDetectedMarkers(cv_image, corners, ids);
                 
                 // 将OpenCV图像转换回ViSP图像显示
                 vpImageConvert::convert(cv_image, I);
                 vpDisplay::display(I);
                 
                 // 显示角点坐标
                 for (int i = 0; i < 4; i++) {
                     vpDisplay::displayText(I, current_points[i], 
                                          "C" + std::to_string(i), vpColor::green);
                     vpDisplay::displayCross(I, current_points[i], 10, vpColor::red, 2);
                 }
             }
             
             return true;
             
         } catch (const cv::Exception& e) {
             ROS_WARN("ArUco detection error: %s", e.what());
             tracking_initialized = false;
             return false;
         }
     }
     
// 改进的几何深度估计
void estimateDepths(std::vector<double>& depths) {
    if (!tracking_initialized) {
        for (int i = 0; i < Np; i++) {
            depths[i] = 0.6; // 默认深度值
        }
        return;
    }
    
    try {
        // 准备3D-2D对应点
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        
        // 设置ArUco码的3D点（以码中心为原点）
        double half_size = aruco_marker_size / 2.0;
        objectPoints.push_back(cv::Point3f(-half_size, half_size, 0));  // 左上
        objectPoints.push_back(cv::Point3f(half_size, half_size, 0));   // 右上
        objectPoints.push_back(cv::Point3f(half_size, -half_size, 0));  // 右下
        objectPoints.push_back(cv::Point3f(-half_size, -half_size, 0)); // 左下
        
        // 将当前图像点转换为OpenCV格式
        for (int i = 0; i < Np; i++) {
            imagePoints.push_back(cv::Point2f(current_points[i].get_u(), 
                                            current_points[i].get_v()));
        }
        ROS_WARN("Acquired image estimateDepths!");
        
        // 相机内参矩阵
        cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 
            cam.get_px(), 0, cam.get_u0(),
            0, cam.get_py(), cam.get_v0(),
            0, 0, 1);
        
        // 畸变系数（假设没有畸变）
        cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
        
        // 使用ArUco方法估计位姿
        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        ROS_WARN("Acquired image estimateDepths!!!");
        
        // 计算每个点在相机坐标系下的3D位置
        cv::Mat R;
        cv::Rodrigues(rvec, R); // 将旋转向量转换为旋转矩阵
        
        for (int i = 0; i < Np; i++) {
            cv::Mat objectPoint = (cv::Mat_<double>(3,1) << 
                                 objectPoints[i].x, 
                                 objectPoints[i].y, 
                                 objectPoints[i].z);
            
            // 将点从物体坐标系转换到相机坐标系
            cv::Mat cameraPoint = R * objectPoint + tvec;
            
            // Z坐标就是深度值
            depths[i] = cameraPoint.at<double>(2,0);

            
            // 检查深度值是否有效
            if (depths[i] <= 0.1 || depths[i] >= 5.0) {
                ROS_WARN_THROTTLE(2.0, "Invalid depth value %.3f at point %d, using fallback", 
                                 depths[i], i);
                depths[i] = 0.6; // 使用回退值
            }
        }
        
        // 打印深度信息用于调试
        ROS_WARN("Depth values from ArUco PnP: %.3f, %.3f, %.3f, %.3f", 
                         depths[0], depths[1], depths[2], depths[3]);
        
    } catch (const cv::Exception& e) {
        ROS_WARN_THROTTLE(1.0, "ArUco depth estimation error: %s, using fallback", e.what());
        for (int i = 0; i < Np; i++) {
            depths[i] = 0.6; // 出错时使用默认深度值
        }
    }
}
     
     void computeImageJacobian(const std::vector<vpImagePoint>& points, 
                              const std::vector<double>& depths,
                              Eigen::Matrix<double, 8, 6>& L) {

            // 添加输入验证
        if (points.size() != Np || depths.size() != Np) {
            ROS_ERROR("Invalid input sizes for Jacobian computation");
            L = Eigen::Matrix<double, 8, 6>::Zero();
            return;
        }

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
     
     void pinv(Eigen::Matrix<double, 8, 6>& L, Eigen::Matrix<double, 6, 8>& pinvL, 
               double alpha0 = 0.001, double w0 = 0.0001) {
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
 
         pinvL = L.transpose() * (L * L.transpose() - alpha * Eigen::MatrixXd::Identity(8, 8)).inverse();
     }
     
     void computeVelocityCommand() {
         try {
             // 获取新图像帧（使用修复的方法）
             g.acquire(I); // 获取灰度图像
             //g.acquire(nullptr,nullptr , nullptr, I_depth.bitmap);  // 获取深度图像

            if (I.bitmap == nullptr || I.getWidth() == 0 || I.getHeight() == 0 ) {
                ROS_WARN_THROTTLE(1.0, "Invalid image acquired");
                publishZeroVelocity();
                return;
            }
             
             if (display_enabled) {
                 vpDisplay::display(I);
                 //displayStatusInformation();
             }
 
             // 检测ArUco码
             bool detection_success = detectArUcoMarkers();
             
             if (!detection_success) {
                 publishZeroVelocity();
                 if (display_enabled) {
                     vpDisplay::displayText(I, 100, 20, "No ArUco marker detected!", vpColor::red);
                     ROS_WARN("Acquired image is nulllll");
                     vpDisplay::flush(I);
                 }
                 return;
             }
             
             // 估计深度
             std::vector<double> depths(Np);
             estimateDepths(depths);
             //ROS_WARN("Acquired image estimateDepths");
             // 计算图像雅可比矩阵
             Eigen::Matrix<double, 8, 6> L_image;
             computeImageJacobian(current_points, depths, L_image);
             //ROS_WARN("Acquired image computeImageJacobian");
             
             // 计算特征误差
             Eigen::Matrix<double, 8, 1> error = computeError();
             double error_norm = error.norm();
             //ROS_WARN("Acquired image computeFeatureError");
             
             // 计算相机速度
             Eigen::Matrix<double, 6, 8> L_pinv;
             pinv(L_image, L_pinv);
             Eigen::Matrix<double, 6, 1> v_camera = computeAdaptiveVelocity(L_pinv, error);
             //ROS_WARN("Acquired image computeAdaptiveVelocity");
             // 限制Z轴速度
             //v_camera(2) = 0.0;
             
             // 转换到末端执行器坐标系
             Eigen::Matrix<double, 6, 1> v_end_effector;
             v_end_effector.head<3>() = handeye_rot * v_camera.head<3>();
             v_end_effector.tail<3>() = handeye_rot * v_camera.tail<3>();
             ROS_WARN("Error norm: %.3f ", error_norm);
             
             // 发布速度命令
             if (send_velocities && error_norm > 300.0) {
                 publishVelocityCommand(v_end_effector);
                 ROS_WARN("Error norm: %.3f, Sending velocities!!!!!", error_norm);
             } else {
                 publishZeroVelocity();
                 if (error_norm <= 10.0) {
                     ROS_INFO_THROTTLE(2.0, "Target reached! Error norm: %.3f", error_norm);
                 }
             }
             
             // 处理用户输入
             //handleUserInput();
             
             if (display_enabled) {
                 vpDisplay::flush(I);
             }
             
         } catch (const vpException &e) {
             ROS_ERROR("Visual servo error: %s", e.getMessage());
             publishZeroVelocity();
         }
     }
     
     Eigen::Matrix<double, 8, 1> computeError() {
        Eigen::Matrix<double, 8, 1> error;
        
        // 1. 计算基础的特征点误差
        for (int i = 0; i < Np; i++) {
            error(2*i) = current_points[i].get_u() - desired_points[i].get_u();
            error(2*i+1) = current_points[i].get_v() - desired_points[i].get_v();
        }
        
        // 2. 估计深度并调整误差
        std::vector<double> depths(Np);
        estimateDepths(depths);
        double avg_depth = 0.0;
        for (int i = 0; i < Np; i++) {
            avg_depth += depths[i] / Np;
        }
        
        // 3. 根据深度缩放误差（使控制在不同距离下更一致）
        double depth_scale = 0.4 / avg_depth; // 以0.5米为参考
        depth_scale = std::max(0.4, std::min(2.0, depth_scale)); // 限制范围
        
        error = error * depth_scale;
        
        // 4. 在图像上显示误差向量
        if (display_enabled && tracking_initialized) {
            for (int i = 0; i < Np; i++) {
                vpDisplay::displayLine(I, current_points[i], desired_points[i], 
                                     vpColor(255, 128, 0), 2);
            }
            
            // 显示深度信息
            std::string depth_text = "Depth: " + std::to_string(avg_depth).substr(0,4) + "m";
            vpDisplay::displayText(I, 120, 20, depth_text, vpColor::yellow);
        }
        
        ROS_INFO_THROTTLE(1.0, "Hybrid error - Depth: %.3fm, Scale: %.3f, Error norm: %.3f",
                         avg_depth, depth_scale, error.norm());
        
        return error;
    }
     
     void displayStatusInformation() {
         std::string status = send_velocities ? "CONTROL ACTIVE" : "CONTROL PAUSED";
         vpDisplay::displayText(I, 20, 20, status, vpColor::red);
         
         if (tracking_initialized) {
             vpDisplay::displayText(I, 40, 20, "ArUco Tracking: ACTIVE", vpColor::green);
         } else {
             vpDisplay::displayText(I, 40, 20, "ArUco Tracking: LOST", vpColor::yellow);
         }
         
         vpDisplay::displayText(I, 60, 20, "Left click: Start/Stop control", vpColor::blue);
         vpDisplay::displayText(I, 80, 20, "Press 'q' to quit", vpColor::blue);
     }
     
     void initializeAdaptiveGain() {
         lambda_0 = 0.8;
         lambda_inf = 0.3;
         lambda_0l = 1.0;
         mu = 0.5;
         start_time = ros::Time::now();
     }
 
     Eigen::Matrix<double, 6, 1> computeAdaptiveVelocity(
         const Eigen::Matrix<double, 6, 8>& L_pinv, 
         const Eigen::Matrix<double, 8, 1>& error) {
         
         Eigen::Matrix<double, 6, 1> v_c = -L_pinv * error;
         double v_norm = v_c.lpNorm<Eigen::Infinity>();
         double t = (ros::Time::now() - start_time).toSec();
         
         lambda = (lambda_0 - lambda_inf) * exp(-lambda_0l * v_norm / (lambda_0 - lambda_inf)) + lambda_inf;
         v_c = lambda * v_c;

         // 对不同的自由度应用不同的增益
        Eigen::Matrix<double, 6, 1> gain_vector;
        gain_vector << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0; // 降低Z轴和旋转的增益
    
        v_c = lambda * gain_vector.cwiseProduct(v_c);
    
         
         ROS_DEBUG_THROTTLE(0.5, "Adaptive gain: lambda=%.3f, v_norm=%.3f", lambda, v_norm);
         
         return v_c;
     }
 
     void handleUserInput() {
         if (!display_enabled) return;
         
         vpMouseButton::vpMouseButtonType button;
         if (vpDisplay::getClick(I, button, false)) {
             if (button == vpMouseButton::button1) {
                 send_velocities = !send_velocities;
                 ROS_INFO("Control %s", send_velocities ? "STARTED" : "STOPPED");
             }
         }
         
         // 检查键盘输入
         char key = vpDisplay::getKeyboardEvent(I);
         if (key == 'q' || key == 'Q') {
             ROS_INFO("Quit signal received");
             ros::shutdown();
         }
     }
     
     void publishVelocityCommand(const Eigen::Matrix<double, 6, 1>& v_end_effector) {
         geometry_msgs::Twist twist;
         
         twist.linear.x = v_end_effector(0);
         twist.linear.y = v_end_effector(1);
         twist.linear.z = v_end_effector(2);
         
         twist.angular.x = v_end_effector(3);
         twist.angular.y = v_end_effector(4);
         twist.angular.z = v_end_effector(5);
         
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
         double linear_norm = sqrt(twist.linear.x*twist.linear.x + 
                                  twist.linear.y*twist.linear.y + 
                                  twist.linear.z*twist.linear.z);
         if (linear_norm > max_linear_vel) {
             double scale = max_linear_vel / linear_norm;
             twist.linear.x *= scale;
             twist.linear.y *= scale;
             twist.linear.z *= scale;
         }
         
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
         ros::Rate rate(30);

         // 添加启动延迟
        ROS_INFO("Waiting 2 seconds for system initialization...");
        ros::Duration(2.0).sleep();

         
         ROS_INFO("ArUco-based Visual Servo node started");
         ROS_INFO("=== Instructions ===");
         ROS_INFO("1. Place an ArUco marker (DICT_6X6_250) in camera view");
         ROS_INFO("2. The system will automatically detect and track the marker");
         ROS_INFO("3. Left click: Start/Stop control");
         ROS_INFO("4. Press 'q': Quit program");
         
         while (ros::ok()) {
             computeVelocityCommand();
             ros::spinOnce();
             rate.sleep();
         }
         
         publishZeroVelocity();
         ROS_INFO("Visual servo node shutdown");
     }
 };
 
 int main(int argc, char** argv) {
     ros::init(argc, argv, "ur10e_d435i_visual_servo_aruco");
     
     try {
         UR10eD435iVisualServo servo;
         servo.run();
     } catch (const std::exception& e) {
         ROS_ERROR("Visual servo failed: %s", e.what());
         return -1;
     }
     
     return 0;
 }