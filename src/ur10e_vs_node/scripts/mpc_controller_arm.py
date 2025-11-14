#!/usr/bin/env python3
# mpc_controller_arm_fixed.py
import numpy as np
import casadi as ca
import rospy
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Twist
from nav_msgs.msg import Path
import tf.transformations as tf_trans

class MPCArmController:
    def __init__(self):
        # MPC parameters - 优化响应速度
        self.N = 3  # 减少预测时域，提高响应速度
        self.dt = 0.01  # 50Hz，提高控制频率
        
        # 机械臂状态维度
        self.state_dim = 6  # [x, y, z, roll, pitch, yaw]
        self.control_dim = 6  # [vx, vy, vz, wx, wy, wz]
        
        # 控制约束 - 提高速度限制
        self.linear_vel_max = 0.5  # 提高线速度限制
        self.angular_vel_max = 3.0  # 提高角速度限制
        
        # 权重矩阵 - 优化响应性
        # 位置权重更高，控制权重更低，提高响应速度
        #self.Q = np.diag([100, 100, 100, 20, 20, 20])  # 大幅增加位置权重
        #self.R = np.diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])  # 降低控制权重，提高响应性
        self.Q = np.diag([20, 20, 20, 4, 4, 4])
        self.R = np.diag([0.005, 0.005, 0.005, 0.001, 0.001, 0.001])
        
        # 当前状态和目标状态
        self.current_state = np.zeros(self.state_dim)
        self.target_state = np.zeros(self.state_dim)
        
        # MPC求解器
        self.solution = None
        self.opti = None
        
        # 参数存储
        self.X0_param = None
        self.X_ref_param = None
        
        # 初始化MPC问题
        self.setup_mpc()
        
    def setup_mpc(self):
        """设置机械臂MPC优化问题"""
        # 创建优化问题
        self.opti = ca.Opti()
        
        # 状态变量 (state_dim x N+1)
        X = self.opti.variable(self.state_dim, self.N + 1)
        
        # 控制变量 (control_dim x N)
        U = self.opti.variable(self.control_dim, self.N)
        
        # 参数 - 需要明确创建参数变量
        self.X0_param = self.opti.parameter(self.state_dim, 1)  # 初始状态
        self.X_ref_param = self.opti.parameter(self.state_dim, self.N + 1)  # 参考轨迹
        
        # 代价函数
        cost = 0
        for k in range(self.N):
            # 状态误差代价
            state_error = X[:, k] - self.X_ref_param[:, k]
            cost += ca.mtimes([state_error.T, self.Q, state_error])
            
            # 控制代价
            control = U[:, k]
            cost += ca.mtimes([control.T, self.R, control])
        
        # 终端代价
        terminal_error = X[:, self.N] - self.X_ref_param[:, self.N]
        cost += ca.mtimes([terminal_error.T, self.Q, terminal_error])
        
        self.opti.minimize(cost)
        
        # 系统动力学 (积分模型)
        for k in range(self.N):
            x_next = X[:, k] + self.arm_dynamics(U[:, k]) * self.dt
            self.opti.subject_to(X[:, k + 1] == x_next)
        
        # 初始条件
        self.opti.subject_to(X[:, 0] == self.X0_param)
        
        # 控制约束
        linear_vel = U[:3, :]  # [vx, vy, vz]
        angular_vel = U[3:, :]  # [wx, wy, wz]
        
        # 线速度约束
        for i in range(3):
            self.opti.subject_to(
                self.opti.bounded(-self.linear_vel_max, linear_vel[i, :], self.linear_vel_max)
            )
        
        # 角速度约束
        for i in range(3):
            self.opti.subject_to(
                self.opti.bounded(-self.angular_vel_max, angular_vel[i, :], self.angular_vel_max)
            )
        
        # 求解器选项 - 使用最兼容的配置
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 20,  # 适中的迭代次数
            'ipopt.tol': 1e-3,  # 适中的求解精度
            'ipopt.acceptable_tol': 1e-2,
            'ipopt.max_cpu_time': 0.05  # 限制求解时间为50ms
        }
        
        try:

            
            self.opti.solver('ipopt', opts)
        except Exception as e:
            rospy.logwarn(f"IPOPT求解器配置失败，使用默认配置: {e}")
            # 使用最基本的配置
            basic_opts = {
                'ipopt.print_level': 0,
                'print_time': 0,
                'ipopt.max_iter': 30,
                'ipopt.tol': 1e-2
            }
            self.opti.solver('ipopt', basic_opts)
        
    def arm_dynamics(self, u):
    # 增加控制增益，让相同的控制输入产生更大的状态变化
        control_gain = 1.5  # 控制增益
        dxdt = ca.vertcat(
            u[0] * control_gain,  # vx -> dx/dt
            u[1] * control_gain,  # vy -> dy/dt  
            u[2] * control_gain,  # vz -> dz/dt
            u[3] * control_gain,  # wx -> d(roll)/dt
            u[4] * control_gain,  # wy -> d(pitch)/dt
            u[5] * control_gain   # wz -> d(yaw)/dt
        )
        return dxdt
    
    def update_current_state(self, pose_stamped):
        """从PoseStamped更新当前状态"""
        # 位置
        self.current_state[0] = pose_stamped.pose.position.x
        self.current_state[1] = pose_stamped.pose.position.y
        self.current_state[2] = pose_stamped.pose.position.z
        
        # 从四元数转换为欧拉角
        quat = [
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w
        ]
        euler = tf_trans.euler_from_quaternion(quat)
        self.current_state[3] = euler[0]  # roll
        self.current_state[4] = euler[1]  # pitch
        self.current_state[5] = euler[2]  # yaw
        
        rospy.logdebug(f"当前状态更新: {self.current_state}")
    
    def set_target_state(self, target_pose):
        """设置目标状态"""
        # 位置
        self.target_state[0] = target_pose.pose.position.x
        self.target_state[1] = target_pose.pose.position.y
        self.target_state[2] = target_pose.pose.position.z
        
        # 从四元数转换为欧拉角
        quat = [
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        ]
        euler = tf_trans.euler_from_quaternion(quat)
        self.target_state[3] = euler[0]  # roll
        self.target_state[4] = euler[1]  # pitch
        self.target_state[5] = euler[2]  # yaw
        
        rospy.loginfo(f"MPC目标设置: pos=({self.target_state[0]:.3f}, {self.target_state[1]:.3f}, {self.target_state[2]:.3f})")
    
    def generate_reference_trajectory(self):
        """生成平滑的参考轨迹到目标"""
        X_ref = np.zeros((self.state_dim, self.N + 1))
        
        # 当前状态
        X_ref[:, 0] = self.current_state
        
        # 计算到目标的距离
        pos_error = np.linalg.norm(self.target_state[:3] - self.current_state[:3])
        ori_error = np.linalg.norm(self.target_state[3:] - self.current_state[3:])
        
        # 如果距离很近，直接使用目标状态
        #pos_error阈值调整为0.01米，ori_error阈值调整为0.05弧度
        if pos_error < 0.5 and ori_error < 5:
            for k in range(1, self.N + 1):
                X_ref[:, k] = self.target_state
            return X_ref
        
        # 使用平滑插值（三次多项式）
        for k in range(1, self.N + 1):
            # 使用平滑的插值函数
            t = k / self.N
            alpha = 3 * t**2 - 2 * t**3  # 平滑的S型曲线
            
            # 位置插值
            X_ref[:3, k] = (1 - alpha) * self.current_state[:3] + alpha * self.target_state[:3]
            
            # 姿态插值 - 处理角度环绕问题
            for i in range(3, 6):
                current_angle = self.current_state[i]
                target_angle = self.target_state[i]
                
                # 找到最短的旋转路径
                diff = target_angle - current_angle
                if diff > np.pi:
                    diff -= 2 * np.pi
                elif diff < -np.pi:
                    diff += 2 * np.pi
                    
                X_ref[i, k] = current_angle + alpha * diff
        
        return X_ref
    
    def solve(self):
        """求解MPC问题"""
        try:
            # 生成参考轨迹
            X_ref = self.generate_reference_trajectory()
            
            # 设置参数值 - 使用正确的参数引用
            self.opti.set_value(self.X0_param, self.current_state.reshape(-1, 1))
            self.opti.set_value(self.X_ref_param, X_ref)
            
            # 简化的热启动：仅使用上一次的控制作为初始猜测
            if self.solution is not None:
                try:
                    # 获取上一次的控制序列
                    prev_control = self.get_control_command()
                    if prev_control is not None and len(prev_control) == self.control_dim:
                        # 简单的初始猜测：使用上一次的控制
                        X_guess = np.zeros((self.state_dim, self.N + 1))
                        U_guess = np.zeros((self.control_dim, self.N))
                        
                        # 设置状态初始猜测
                        X_guess[:, 0] = self.current_state
                        for k in range(1, self.N + 1):
                            X_guess[:, k] = X_guess[:, k-1] + prev_control * self.dt
                        
                        # 设置控制初始猜测
                        for k in range(self.N):
                            U_guess[:, k] = prev_control
                        
                        # 设置初始猜测
                        initial_guess = np.concatenate([X_guess.flatten(), U_guess.flatten()])
                        self.opti.set_initial(self.opti.x, initial_guess)
                except Exception as e:
                    rospy.logdebug(f"热启动失败，使用默认初始值: {e}")
            
            # 求解
            self.solution = self.opti.solve()
            rospy.logdebug("MPC求解成功")
            return True
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"MPC求解失败: {e}")
            # 如果求解失败，尝试使用更宽松的约束重新求解
            try:
                rospy.loginfo_throttle(2.0, "尝试使用更宽松的约束重新求解...")
                
                # 尝试使用更宽松的容差重新求解
                self.opti.set_value(self.X0_param, self.current_state.reshape(-1, 1))
                self.opti.set_value(self.X_ref_param, X_ref)
                
                # 使用更宽松的求解器选项
                relaxed_opts = {
                    'ipopt.print_level': 0,
                    'print_time': 0,
                    'ipopt.max_iter': 20,
                    'ipopt.tol': 1e-1,
                    'ipopt.acceptable_tol': 1e-1
                }
                
                # 临时重新配置求解器
                self.opti.solver('ipopt', relaxed_opts)
                self.solution = self.opti.solve()
                rospy.loginfo("使用宽松约束求解成功")
                return True
                
            except Exception as e2:
                rospy.logwarn_throttle(2.0, f"降级求解也失败: {e2}")
                return False
    
    def get_control_command(self):
        """获取第一个控制命令"""
        if self.solution is None:
            return np.zeros(6)
        
        try:
            # 获取所有变量
            all_vars = self.solution.value(self.opti.x)
            
            # 提取控制变量 (状态变量在前，控制变量在后)
            state_var_size = self.state_dim * (self.N + 1)
            control_start_idx = state_var_size
            
            # 第一个控制命令
            control = all_vars[control_start_idx:control_start_idx + self.control_dim]
            
            return control.flatten()
            
        except Exception as e:
            rospy.logwarn(f"获取控制命令失败: {e}")
            return np.zeros(6)
    
    def get_control_twist(self):
        """获取控制命令作为Twist消息"""
        control = self.get_control_command()
        
        twist = Twist()
        twist.linear.x = float(control[0])
        twist.linear.y = float(control[1])
        twist.linear.z = float(control[2])
        twist.angular.x = float(control[3])
        twist.angular.y = float(control[4])
        twist.angular.z = float(control[5])
        
        return twist
    
    def get_predicted_trajectory(self):
        """获取预测的状态轨迹"""
        if self.solution is None:
            return []
        
        try:
            # 获取所有变量
            all_vars = self.solution.value(self.opti.x)
            
            # 提取状态变量
            X_pred = all_vars[:self.state_dim * (self.N + 1)].reshape(self.state_dim, self.N + 1)
            
            trajectory = []
            for k in range(self.N + 1):
                point = Point()
                point.x = float(X_pred[0, k])
                point.y = float(X_pred[1, k])
                point.z = float(X_pred[2, k])
                trajectory.append(point)
            
            return trajectory
            
        except Exception as e:
            rospy.logwarn(f"获取预测轨迹失败: {e}")
            return []
    
    def get_predicted_poses(self):
        """获取预测的完整位姿轨迹"""
        if self.solution is None:
            return []
        
        try:
            # 获取所有变量
            all_vars = self.solution.value(self.opti.x)
            
            # 提取状态变量
            X_pred = all_vars[:self.state_dim * (self.N + 1)].reshape(self.state_dim, self.N + 1)
            
            poses = []
            for k in range(self.N + 1):
                pose = PoseStamped()
                pose.header.frame_id = "base"
                pose.header.stamp = rospy.Time.now()
                
                # 位置
                pose.pose.position.x = float(X_pred[0, k])
                pose.pose.position.y = float(X_pred[1, k])
                pose.pose.position.z = float(X_pred[2, k])
                
                # 姿态
                euler = [float(X_pred[3, k]), float(X_pred[4, k]), float(X_pred[5, k])]
                quat = tf_trans.quaternion_from_euler(euler[0], euler[1], euler[2])
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                
                poses.append(pose)
            
            return poses
            
        except Exception as e:
            rospy.logwarn(f"获取预测位姿失败: {e}")
            return []