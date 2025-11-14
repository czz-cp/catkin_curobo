from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time

# 创建RTDE接收接口实例
rtde_c = RTDEControl("169.254.138.15")
rtde_recv = RTDEReceive("169.254.138.15")

pi = 3.1415926

# 移动到初始位置

time.sleep(1)

print("开始获取位置信息...")
try:
    while True:
        # 获取关节位置
        joint_positions = rtde_recv.getActualQ()
        
        # 获取TCP位姿
        tcp_pose = rtde_recv.getActualTCPPose()
        
        print(f"\n关节位置: {[f'{x:.4f}' for x in joint_positions]}")
        print(f"TCP位姿 - 位置 [x,y,z]: {[f'{tcp_pose[i]:.4f}' for i in range(3)]}")
        print(f"TCP位姿 - 旋转 [rx,ry,rz]: {[f'{tcp_pose[i]:.4f}' for i in range(3,6)]}")
        
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\n程序结束")