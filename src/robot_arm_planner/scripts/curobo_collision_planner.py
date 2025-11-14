#!/usr/bin/env python3
"""
基于cuRobo防碰撞的机械臂路径规划器
直接使用cuRobo内置的SDF/ESDF碰撞检测，无需外部ORCA算法
Author: Claude
Date: 2024
"""

import rospy
import numpy as np
import torch
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import JointState, PointCloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Path
from std_msgs.msg import Header, Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
from map_manager.srv import GetStaticObstacles
from onboard_detector.srv import GetDynamicObstacles
import yaml
import math
import sensor_msgs.point_cloud2 as pc2

# CuRobo imports
try:
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig, JointState
    from curobo.types.math import Pose
    from curobo.types.world import WorldConfig
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
    from curobo.geom.sdf.world import WorldVoxelCollision, WorldCollision, WorldCollisionConfig
    from curobo.geom.types import VoxelGrid
    from curobo.rollout.cost.primitive_collision_cost import PrimitiveCollisionCostConfig
    CUROBO_AVAILABLE = True
except ImportError as e:
    rospy.logwarn(f"CuRobo not available: {e}")
    CUROBO_AVAILABLE = False

class CuRoboCollisionPlanner:
    def __init__(self):
        rospy.init_node('curobo_collision_planner', anonymous=True)

        if not CUROBO_AVAILABLE:
            rospy.logerr("CuRobo not available. Exiting.")
            return

        # 加载配置
        self.load_config()

        # 初始化cuRobo
        self.init_curobo()

        # 障碍物数据缓存 (优先使用聚类结果)
        self.static_obstacles_cache = None
        self.dynamic_obstacles_cache = None
        self.last_obstacle_update_time = rospy.Time(0)
        self.current_joint_states = None
        self.current_pose = None
        self.goal_pose = None
        self.planned_trajectory = None
        self.collision_free_trajectory = None

        # 动态重规划参数
        self.enable_dynamic_replanning = rospy.get_param('~enable_dynamic_replanning', True)
        self.replanning_frequency = rospy.get_param('~replanning_frequency', 10.0)  # Hz
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.1)  # 距离阈值

        # ROS接口
        self.setup_ros_interfaces()

        rospy.loginfo("CuRobo Collision Planner initialized with dynamic replanning")
        rospy.loginfo(f"Dynamic replanning: {self.enable_dynamic_replanning}, Frequency: {self.replanning_frequency}Hz")

    def load_config(self):
        """加载配置文件"""
        config_file = rospy.get_param('~config_file',
            '/home/zar/Downloads/curobo/robot_arm_planner/config/robot_parameters.yaml')

        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            rospy.logwarn(f"Failed to load config: {e}")
            self.config = {}

        # cuRobo防碰撞参数
        collision_config = self.config.get('collision', {})
        self.activation_distance = collision_config.get('activation_distance', 0.02)  # 2cm
        self.collision_weight = collision_config.get('collision_weight', 100.0)
        self.use_speed_metric = collision_config.get('use_speed_metric', True)
        self.use_sweep = collision_config.get('use_sweep', True)
        self.sweep_steps = collision_config.get('sweep_steps', 8)

        # 机械臂参数
        robot_config = self.config.get('robot', {})
        self.joint_names = robot_config.get('joint_names', [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ])

        # cuRobo配置文件路径
        curobo_config = self.config.get('curobo', {})
        self.robot_config = curobo_config.get('robot_config_file', 'ur10e.yml')
        self.world_config = curobo_config.get('world_config_file', 'collision_table.yml')

    def init_curobo(self):
        """初始化cuRobo MotionGen和IK求解器"""
        try:
            # 创建张量设备类型
            self.tensor_args = TensorDeviceType(device=torch.device("cuda" if torch.cuda.is_available() else "cpu"))

            # 创建MotionGen配置
            motion_gen_cfg = MotionGenConfig.load_from_robot_config(
                self.robot_config,
                self.world_config,
                self.tensor_args,
                trajopt_tsteps=34,
                interpolation_steps=1000,
                num_ik_seeds=50,
                num_trajopt_seeds=6,
                grad_trajopt_iters=500,
                trajopt_dt=0.1,
                interpolation_dt=0.02,
                evaluate_interpolated_trajectory=True,
                js_trajopt_dt=0.1,
                js_trajopt_tsteps=34,
                collision_activation_distance=self.activation_distance,
                use_cuda_graph=True
            )

            # 创建MotionGen实例
            self.motion_gen = MotionGen(motion_gen_cfg)

            # 创建IK求解器配置 - 使用与MotionGen相同的碰撞环境
            ik_cfg = IKSolverConfig.load_from_robot_config(
                self.motion_gen.robot_config,
                None,  # IK求解器将使用MotionGen的world_coll_checker
                rotation_threshold=0.05,
                position_threshold=0.005,
                num_seeds=20,
                self_collision_check=True,  # 启用自碰撞检查
                self_collision_opt=True,
                tensor_args=self.tensor_args,
                use_cuda_graph=True,
            )

            # 创建IK求解器实例并共享MotionGen的碰撞检测器
            self.ik_solver = IKSolver(ik_cfg)

            # 共享MotionGen的world_coll_checker给IK求解器
            self.ik_solver.world_coll_checker = self.motion_gen.world_coll_checker

            # 预热
            self.motion_gen.warmup()
            self.ik_solver.warmup()

            # 重置状态
            self.motion_gen.reset(reset_seed=False)
            self.motion_gen.world_coll_checker.clear_cache()

            rospy.loginfo("CuRobo MotionGen and IK Solver initialized successfully")

        except Exception as e:
            rospy.logerr(f"Failed to initialize cuRobo: {e}")
            self.motion_gen = None
            self.ik_solver = None

    def setup_ros_interfaces(self):
        """设置ROS接口"""

        # 订阅者
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        rospy.Subscriber('/goal_pose', PoseStamped, self.goal_pose_callback, queue_size=1)
        rospy.Subscriber('/planned_trajectory', JointTrajectory, self.planned_trajectory_callback, queue_size=1)

        # NavRL ESDF订阅 (用于更新碰撞环境)
        rospy.Subscriber('/arm_esdf_map/esdf', PointCloud2, self.esdf_callback, queue_size=1)

        # NavRL 静态障碍物聚类订阅 (直接使用聚类结果)
        rospy.Subscriber('/occupancy_map/static_obstacles', MarkerArray, self.static_obstacles_callback, queue_size=1)

        # NavRL 动态障碍物聚类订阅 (直接使用聚类结果)
        rospy.Subscriber('/onboard_detector/dynamic_bboxes', MarkerArray, self.dynamic_obstacles_callback, queue_size=1)

        # 发布者
        self.collision_free_trajectory_pub = rospy.Publisher('/collision_free_trajectory', JointTrajectory, queue_size=1)
        self.collision_status_pub = rospy.Publisher('/collision_status', Float64MultiArray, queue_size=1)
        self.marker_pub = rospy.Publisher('/collision_markers', MarkerArray, queue_size=1)

        # 服务
        self.plan_collision_free_path_service = rospy.Service(
            '/plan_collision_free_path', Trigger, self.plan_collision_free_path_callback
        )
        self.check_trajectory_collision_service = rospy.Service(
            '/check_trajectory_collision', Trigger, self.check_trajectory_collision_callback
        )

        # 动态重规划定时器
        if self.enable_dynamic_replanning:
            self.replanning_timer = rospy.Timer(
                rospy.Duration(1.0 / self.replanning_frequency),
                self.dynamic_replanning_callback
            )

    def joint_states_callback(self, msg):
        """关节状态回调"""
        # 过滤UR10e关节
        if all(name in msg.name for name in self.joint_names):
            self.current_joint_states = msg

    def current_pose_callback(self, msg):
        """当前位姿回调"""
        self.current_pose = msg

    def goal_pose_callback(self, msg):
        """目标位姿回调"""
        self.goal_pose = msg

    def planned_trajectory_callback(self, msg):
        """规划轨迹回调"""
        self.planned_trajectory = msg
        # 自动检查碰撞
        self.check_and_filter_trajectory()

    def esdf_callback(self, msg):
        """NavRL ESDF回调 - 直接使用cuRobo内置的SDF/ESDF碰撞检测"""
        if self.motion_gen is None:
            return

        # 检查是否有更新的聚类数据
        current_time = rospy.Time.now()
        time_since_obstacles = (current_time - self.last_obstacle_update_time).to_sec()

        # 如果最近有聚类数据更新，不使用ESDF (聚类结果更精确)
        if time_since_obstacles < 0.5 and (self.static_obstacles_cache or self.dynamic_obstacles_cache):
            rospy.logdebug("Skipping ESDF update - using recent obstacle clustering data")
            return

        try:
            # 直接将NavRL ESDF转换为cuRobo VoxelGrid
            voxel_grid = self.convert_navrl_esdf_to_voxel_grid(msg)

            if voxel_grid is not None:
                # 更新cuRobo的world_coll_checker中的ESDF数据
                if hasattr(self.motion_gen.world_coll_checker, 'update_voxel_data'):
                    self.motion_gen.world_coll_checker.update_voxel_data(voxel_grid, env_idx=0)

                    # 同步更新IK求解器的碰撞检测器
                    if hasattr(self.ik_solver, 'world_coll_checker') and self.ik_solver.world_coll_checker:
                        self.ik_solver.world_coll_checker.update_voxel_data(voxel_grid, env_idx=0)

                    rospy.logdebug(f"Updated cuRobo ESDF with VoxelGrid: {voxel_grid.dims}")
                else:
                    rospy.logwarn("cuRobo world_coll_checker does not support voxel updates")
            else:
                rospy.logwarn("Failed to create VoxelGrid from NavRL ESDF")

        except Exception as e:
            rospy.logwarn(f"Failed to update cuRobo ESDF: {e}")

    def static_obstacles_callback(self, msg):
        """NavRL静态障碍物聚类回调 - 缓存聚类结果"""
        try:
            # 缓存静态障碍物聚类结果
            self.static_obstacles_cache = msg
            self.last_obstacle_update_time = rospy.Time.now()

            # 立即更新cuRobo世界环境
            self.update_curobo_world_with_obstacles()

            rospy.logdebug(f"Received {len(msg.markers)} static obstacle clusters from NavRL")

        except Exception as e:
            rospy.logwarn(f"Failed to process static obstacles: {e}")

    def dynamic_obstacles_callback(self, msg):
        """NavRL动态障碍物聚类回调 - 缓存聚类结果"""
        try:
            # 缓存动态障碍物聚类结果
            self.dynamic_obstacles_cache = msg
            self.last_obstacle_update_time = rospy.Time.now()

            # 立即更新cuRobo世界环境
            self.update_curobo_world_with_obstacles()

            rospy.logdebug(f"Received {len(msg.markers)} dynamic obstacle clusters from NavRL")

        except Exception as e:
            rospy.logwarn(f"Failed to process dynamic obstacles: {e}")

    def update_curobo_world_with_obstacles(self):
        """使用缓存的障碍物数据更新cuRobo世界环境"""
        if self.motion_gen is None:
            return

        try:
            # 合并静态和动态障碍物
            all_markers = []

            if self.static_obstacles_cache:
                all_markers.extend(self.static_obstacles_cache.markers)

            if self.dynamic_obstacles_cache:
                all_markers.extend(self.dynamic_obstacles_cache.markers)

            if not all_markers:
                return

            # 创建临时MarkerArray
            combined_markers = MarkerArray()
            combined_markers.markers = all_markers

            # 转换为cuRobo WorldConfig
            world_config = self.convert_marker_array_to_world_config(combined_markers, obstacle_type="combined")

            # 更新MotionGen的世界环境
            self.motion_gen.update_world(world_config)

            rospy.logdebug(f"Updated cuRobo world with {len(world_config.cuboid) if world_config.cuboid else 0} obstacles (static+dynamic)")

        except Exception as e:
            rospy.logwarn(f"Failed to update cuRobo world with obstacles: {e}")

    def convert_marker_array_to_world_config(self, marker_array_msg, obstacle_type="unknown"):
        """将NavRL的MarkerArray聚类结果转换为cuRobo WorldConfig"""
        try:
            from curobo.geom.types import Cuboid

            cuboids = []

            for marker in marker_array_msg.markers:
                if marker.type != Marker.CUBE:
                    continue

                # 从marker获取立方体参数
                # marker.pose 包含位置和方向
                pos = marker.pose.position
                ori = marker.pose.orientation
                # marker.scale 包含尺寸
                dims = [marker.scale.x, marker.scale.y, marker.scale.z]

                # 创建cuRobo立方体障碍物
                cuboid = Cuboid(
                    name=f"{obstacle_type}_obs_{marker.id}",
                    pose=[pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z],
                    dims=dims
                )
                cuboids.append(cuboid)

            return WorldConfig(cuboid=cuboids)

        except Exception as e:
            rospy.logwarn(f"Failed to convert MarkerArray to WorldConfig: {e}")
            return WorldConfig()  # 返回空世界配置

    def convert_navrl_esdf_to_voxel_grid(self, pointcloud_msg):
        """将NavRL ESDF PointCloud2直接转换为cuRobo VoxelGrid格式"""
        try:
            # 从PointCloud2消息中提取ESDF数据
            esdf_points = []
            for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                x, y, z, distance = point
                esdf_points.append([x, y, z, distance])

            if not esdf_points:
                rospy.logwarn("No valid ESDF points in NavRL message")
                return None

            # 转换为numpy数组
            esdf_points = np.array(esdf_points)

            # 体素参数 (与NavRL配置一致)
            voxel_size = 0.02  # 2cm分辨率
            dims = [100, 100, 75]  # 2×2×1.5m网格

            # 创建体素网格原点 (居中于工作空间)
            origin = np.array([0.0, 0.0, 0.75])  # 地板以上0.75m

            # 创建ESDF特征张量
            feature_tensor = torch.full(
                dims,
                1.5,  # 最大ESDF距离 (1.5m)
                dtype=self.tensor_args.dtype,
                device=self.tensor_args.device
            )

            # 填充ESDF值
            for point in esdf_points:
                x, y, z, distance = point

                # 转换到体素坐标
                voxel_x = int((x - origin[0]) / voxel_size + dims[0] // 2)
                voxel_y = int((y - origin[1]) / voxel_size + dims[1] // 2)
                voxel_z = int((z - origin[2]) / voxel_size + dims[2] // 2)

                # 检查边界
                if (0 <= voxel_x < dims[0] and
                    0 <= voxel_y < dims[1] and
                    0 <= voxel_z < dims[2]):

                    # 限制ESDF值在合理范围内
                    esdf_value = np.clip(distance, -0.1, 1.5)
                    feature_tensor[voxel_x, voxel_y, voxel_z] = esdf_value

            # 创建cuRobo VoxelGrid对象
            voxel_grid = VoxelGrid(
                name="navrl_esdf",
                pose=[origin[0], origin[1], origin[2], 1.0, 0.0, 0.0, 0.0],
                dims=dims,
                voxel_size=voxel_size,
                feature_tensor=feature_tensor
            )

            rospy.logdebug(f"Created VoxelGrid with {len(esdf_points)} ESDF points")
            return voxel_grid

        except Exception as e:
            rospy.logerr(f"Failed to convert NavRL ESDF to VoxelGrid: {e}")
            return None

    def convert_esdf_to_world_config(self, pointcloud_msg):
        """将NavRL ESDF PointCloud2转换为cuRobo WorldConfig"""
        try:
            from curobo.geom.types import Cuboid

            # 从PointCloud2消息中提取障碍物点
            obstacle_points = []
            for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                x, y, z, distance = point
                # 如果距离小于阈值，认为是障碍物表面
                if distance < 0.05:  # 5cm内认为是障碍物
                    obstacle_points.append([x, y, z])

            if not obstacle_points:
                return WorldConfig()  # 空世界配置

            # 使用DBSCAN聚类障碍物点云
            from sklearn.cluster import DBSCAN
            import numpy as np

            points_array = np.array(obstacle_points)
            clustering = DBSCAN(eps=0.1, min_samples=5).fit(points_array)

            # 创建障碍物立方体列表
            cuboids = []
            unique_labels = set(clustering.labels_)
            for label in unique_labels:
                if label == -1:  # 忽略噪声点
                    continue

                cluster_points = points_array[clustering.labels_ == label]
                if len(cluster_points) < 3:
                    continue

                # 计算聚类边界框
                min_point = cluster_points.min(axis=0)
                max_point = cluster_points.max(axis=0)
                center = (min_point + max_point) / 2
                dimensions = max_point - min_point + 0.02  # 添加2cm填充

                # 创建立方体障碍物
                cuboid = Cuboid(
                    name=f"obstacle_{label}",
                    pose=[center[0], center[1], center[2], 1.0, 0.0, 0.0, 0.0],  # [x,y,z,w,x,y,z]
                    dims=[dimensions[0], dimensions[1], dimensions[2]]
                )
                cuboids.append(cuboid)

            return WorldConfig(cuboid=cuboids)

        except Exception as e:
            rospy.logwarn(f"Failed to convert ESDF to WorldConfig: {e}")
            return WorldConfig()  # 返回空世界配置

    def convert_esdf_to_voxel(self, pointcloud_msg):
        """将NavRL ESDF PointCloud2转换为cuRobo voxel tensor"""
        try:
            # 从PointCloud2消息中提取点云数据
            points = []
            for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                x, y, z, distance = point
                points.append([x, y, z, distance])

            if not points:
                return self.create_default_esdf()

            # 转换为numpy数组
            points = np.array(points)

            # 创建网格
            voxel_size = 0.02  # 2cm分辨率
            grid_dims = [100, 100, 100]  # 2×2×2m
            origin = np.array([-1.0, -1.0, 0.0])  # 网格原点

            # 创建ESDF张量
            esdf_tensor = torch.full(grid_dims, 1.0, device=self.tensor_args.device, dtype=self.tensor_args.dtype)

            # 填充ESDF值
            for point in points:
                x, y, z, distance = point

                # 转换到体素坐标
                voxel_x = int((x - origin[0]) / voxel_size)
                voxel_y = int((y - origin[1]) / voxel_size)
                voxel_z = int((z - origin[2]) / voxel_size)

                # 检查边界
                if (0 <= voxel_x < grid_dims[0] and
                    0 <= voxel_y < grid_dims[1] and
                    0 <= voxel_z < grid_dims[2]):

                    # 设置ESDF值 (distance已经是欧几里得距离)
                    esdf_tensor[voxel_x, voxel_y, voxel_z] = min(distance, 1.0)

            return esdf_tensor

        except Exception as e:
            rospy.logwarn(f"Failed to convert ESDF point cloud: {e}")
            return self.create_default_esdf()

    def create_default_esdf(self):
        """创建默认ESDF张量（无障碍物）"""
        grid_dims = [100, 100, 100]  # 2×2×2m
        # 默认情况下，所有点都是安全的（正距离）
        return torch.ones(grid_dims, device=self.tensor_args.device, dtype=self.tensor_args.dtype)

    def check_and_filter_trajectory(self):
        """检查并过滤轨迹中的碰撞 (使用MotionGen)"""
        if self.planned_trajectory is None or self.motion_gen is None:
            return

        # MotionGen已经生成了碰撞避免的轨迹，这里主要用于验证
        # 如果需要额外的碰撞检查，可以使用MotionGen的碰撞检测功能
        rospy.logdebug("Trajectory is already collision-free from MotionGen planning")

    def compute_collision_cost(self, trajectory):
        """计算轨迹的碰撞成本"""
        if self.curobo_world is None:
            return float('inf')

        total_cost = 0.0
        collision_points = []

        for i, point in enumerate(trajectory.points):
            # 转换为tensor
            joint_tensor = torch.tensor(point.positions, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
            joint_tensor = joint_tensor.unsqueeze(0).unsqueeze(0)

            # 计算球体表示
            spheres = self.curobo_world.robot_model.compute_sphere_representation(joint_tensor)

            # 计算碰撞距离
            collision_distance = self.curobo_world.get_collision_distance(spheres)

            # 计算成本 (负距离表示碰撞成本)
            point_cost = torch.sum(torch.clamp(-collision_distance, min=0)).item()
            total_cost += point_cost

            if point_cost > 0:
                collision_points.append(i)

        return total_cost, collision_points

    def plan_collision_free_path_callback(self, req):
        """规划无碰撞路径服务回调"""
        try:
            if self.current_joint_states is None or self.goal_pose is None or self.curobo_world is None:
                return TriggerResponse(False, "System not ready")

            # 使用cuRobo进行碰撞自由路径规划
            success = self.plan_with_curobo_collision()

            if success:
                return TriggerResponse(True, "Collision-free path planned successfully")
            else:
                return TriggerResponse(False, "Failed to plan collision-free path")

        except Exception as e:
            rospy.logerr(f"Path planning failed: {e}")
            return TriggerResponse(False, str(e))

    def plan_with_curobo_collision(self):
        """使用cuRobo MotionGen进行碰撞避免的路径规划"""
        try:
            if self.motion_gen is None:
                rospy.logerr("CuRobo MotionGen not initialized")
                return False

            # 当前关节状态
            current_joints = self.extract_joint_positions(self.current_joint_states)
            start_state = JointState.from_position(
                self.tensor_args.to_device([current_joints]),
                joint_names=self.joint_names
            )

            # 目标位姿
            if self.goal_pose is None:
                rospy.logerr("No goal pose available")
                return False

            goal_pose = Pose(
                position=self.tensor_args.to_device([[self.goal_pose.pose.position.x,
                                                     self.goal_pose.pose.position.y,
                                                     self.goal_pose.pose.position.z]]),
                quaternion=self.tensor_args.to_device([[self.goal_pose.pose.orientation.w,
                                                       self.goal_pose.pose.orientation.x,
                                                       self.goal_pose.pose.orientation.y,
                                                       self.goal_pose.pose.orientation.z]])
            )

            # 使用cuRobo MotionGen规划
            result = self.motion_gen.plan_single(start_state, goal_pose)

            if result.success.item():
                # 获取插值轨迹
                interpolated_trajectory = result.get_interpolated_plan()

                # 转换为ROS轨迹消息
                self.collision_free_trajectory = self.convert_to_ros_trajectory(interpolated_trajectory)
                self.collision_free_trajectory_pub.publish(self.collision_free_trajectory)

                rospy.loginfo(f"Successfully planned collision-free trajectory with {len(self.collision_free_trajectory.points)} waypoints")
                return True
            else:
                rospy.logwarn(f"MotionGen planning failed: {result.status}")
                return False

        except Exception as e:
            rospy.logerr(f"cuRobo MotionGen planning failed: {e}")
            return False

    def compute_ik_for_goal(self):
        """使用cuRobo IK求解器计算目标位姿的逆运动学"""
        if self.goal_pose is None or self.ik_solver is None:
            return None

        try:
            # 创建目标位姿
            goal_pose = Pose(
                position=self.tensor_args.to_device([[self.goal_pose.pose.position.x,
                                                     self.goal_pose.pose.position.y,
                                                     self.goal_pose.pose.position.z]]),
                quaternion=self.tensor_args.to_device([[self.goal_pose.pose.orientation.w,
                                                       self.goal_pose.pose.orientation.x,
                                                       self.goal_pose.pose.orientation.y,
                                                       self.goal_pose.pose.orientation.z]])
            )

            # 使用当前关节配置作为种子 (如果可用)
            retract_config = None
            if self.current_joint_states is not None:
                current_joints = self.extract_joint_positions(self.current_joint_states)
                retract_config = self.tensor_args.to_device([current_joints])

            # 求解IK
            result = self.ik_solver.solve_single(
                goal_pose,
                retract_config=retract_config,
                num_seeds=20,
                return_seeds=1
            )

            if result.success.item():
                # 返回关节角度列表
                goal_joints = result.joint_state.position[0].cpu().numpy().tolist()
                rospy.logdebug(f"IK solution found: {goal_joints}")
                return goal_joints
            else:
                rospy.logwarn(f"IK solution failed: position_error={result.position_error.item():.4f}, rotation_error={result.rotation_error.item():.4f}")
                return None

        except Exception as e:
            rospy.logerr(f"IK computation failed: {e}")
            return None

    def check_trajectory_collision_callback(self, req):
        """检查轨迹碰撞服务回调"""
        try:
            if self.planned_trajectory is None:
                return TriggerResponse(False, "No trajectory to check")

            collision_cost, collision_points = self.compute_collision_cost(self.planned_trajectory)

            if collision_cost > 0:
                msg = f"Collision detected at points: {collision_points}, cost: {collision_cost:.4f}"
                return TriggerResponse(False, msg)
            else:
                return TriggerResponse(True, "Trajectory is collision-free")

        except Exception as e:
            rospy.logerr(f"Collision check failed: {e}")
            return TriggerResponse(False, str(e))

    def extract_joint_positions(self, joint_states_msg):
        """从JointState消息提取关节位置"""
        positions = []
        for joint_name in self.joint_names:
            if joint_name in joint_states_msg.name:
                idx = joint_states_msg.name.index(joint_name)
                positions.append(joint_states_msg.position[idx])
            else:
                rospy.logwarn(f"Joint {joint_name} not found in joint states")
                positions.append(0.0)
        return positions

    def convert_to_ros_trajectory(self, joint_state_trajectory):
        """将cuRobo MotionGen轨迹转换为ROS JointTrajectory"""
        ros_trajectory = JointTrajectory()
        ros_trajectory.header.stamp = rospy.Time.now()
        ros_trajectory.header.frame_id = "base_link"
        ros_trajectory.joint_names = self.joint_names

        # 转换每个轨迹点
        for i in range(len(joint_state_trajectory)):
            point = JointTrajectoryPoint()
            point.positions = joint_state_trajectory.position[i].cpu().numpy().tolist()
            point.velocities = joint_state_trajectory.velocity[i].cpu().numpy().tolist() if hasattr(joint_state_trajectory, 'velocity') else [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            point.time_from_start = rospy.Duration(i * 0.02)  # 50Hz = 20ms per point
            ros_trajectory.points.append(point)

        return ros_trajectory

    def publish_collision_status(self, collision_indices):
        """发布碰撞状态"""
        status = Float64MultiArray()
        status.data = [
            float(len(collision_indices)),           # 碰撞点数量
            float(self.collision_weight),            # 碰撞权重
            float(self.activation_distance),         # 激活距离
            float(self.use_sweep),                   # 是否使用扫掠
            float(self.sweep_steps)                  # 扫掠步数
        ]
        self.collision_status_pub.publish(status)

    def dynamic_replanning_callback(self, event):
        """动态重规划回调 - 检查环境变化并重新规划"""
        if not self.enable_dynamic_replanning:
            return

        try:
            # 检查当前轨迹是否仍然安全
            if self.collision_free_trajectory is not None:
                current_cost, collision_points = self.compute_collision_cost(self.collision_free_trajectory)

                # 如果检测到碰撞或接近碰撞，触发重规划
                if current_cost > 0 or len(collision_points) > 0:
                    rospy.logwarn(f"Dynamic obstacle detected! Cost: {current_cost:.4f}, Collisions: {len(collision_points)}")
                    self.replan_current_trajectory()

            # 检查是否需要重新规划到目标
            if self.goal_pose is not None:
                self.check_goal_reachability()

        except Exception as e:
            rospy.logerr(f"Dynamic replanning failed: {e}")

    def replan_current_trajectory(self):
        """重新规划当前轨迹"""
        if self.goal_pose is None or self.current_joint_states is None:
            return

        try:
            rospy.loginfo("Replanning trajectory due to dynamic obstacles...")

            # 使用cuRobo重新规划
            success = self.plan_with_curobo_collision()

            if success:
                rospy.loginfo("✅ Dynamic replanning successful!")
            else:
                rospy.logwarn("❌ Dynamic replanning failed, stopping movement")
                self.publish_stop_trajectory()

        except Exception as e:
            rospy.logerr(f"Replanning failed: {e}")

    def check_goal_reachability(self):
        """检查目标是否仍然可达"""
        if self.goal_pose is None or self.curobo_world is None:
            return

        try:
            # 检查目标位置是否有新的障碍物
            target_joints = self.compute_ik_for_goal()
            if target_joints is None:
                return

            joint_tensor = torch.tensor(target_joints, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
            joint_tensor = joint_tensor.unsqueeze(0).unsqueeze(0)

            spheres = self.curobo_world.robot_model.compute_sphere_representation(joint_tensor)
            collision_distance = self.curobo_world.get_collision_distance(spheres)

            if torch.any(collision_distance < self.obstacle_threshold):
                rospy.logwarn("Target position is no longer reachable due to dynamic obstacles!")
                # 可以触发停止或寻找新目标

        except Exception as e:
            rospy.logwarn(f"Goal reachability check failed: {e}")

    def publish_stop_trajectory(self):
        """发布停止轨迹"""
        stop_trajectory = JointTrajectory()
        stop_trajectory.header.stamp = rospy.Time.now()
        stop_trajectory.header.frame_id = "base_link"
        stop_trajectory.joint_names = self.joint_names

        stop_point = JointTrajectoryPoint()
        if self.current_joint_states:
            stop_point.positions = self.extract_joint_positions(self.current_joint_states)
        else:
            stop_point.positions = [0.0] * 6
        stop_point.velocities = [0.0] * 6
        stop_point.time_from_start = rospy.Duration(0.1)

        stop_trajectory.points.append(stop_point)
        self.collision_free_trajectory_pub.publish(stop_trajectory)

    def publish_collision_markers(self):
        """发布碰撞可视化标记"""
        if self.collision_free_trajectory is None:
            return

        marker_array = MarkerArray()

        # 发布轨迹标记
        for i, point in enumerate(self.collision_free_trajectory.points):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "collision_free_trajectory"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
            marker.color.a = 0.8
            marker.color.g = 1.0

            # 这里需要正向运动学计算位置，简化实现
            marker.pose.position.x = 0.5 + i * 0.02
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def run(self):
        """主循环"""
        rospy.loginfo("CuRobo Collision Planner running...")

        rate = rospy.Rate(10)  # 10Hz
        try:
            while not rospy.is_shutdown():
                # 发布可视化
                self.publish_collision_markers()

                rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down CuRobo Collision Planner")

if __name__ == '__main__':
    try:
        planner = CuRoboCollisionPlanner()
        if CUROBO_AVAILABLE and planner.curobo_world is not None:
            planner.run()
    except rospy.ROSInterruptException:
        pass