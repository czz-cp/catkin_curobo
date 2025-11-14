#!/bin/bash
# mpc_ur_arm_launcher.sh - 全权限容器启动脚本

CONTAINER_NAME="mpc_ur_arm"
IMAGE_NAME="mpc_ur_arm"
HOST_WS_PATH="/home/zar/catkin"
CONTAINER_WS_PATH="/home/vispci/catkin"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的信息
echo -e "${GREEN}🚀 启动 MPC UR 机械臂容器...${NC}"

# 检查镜像是否存在
if ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo -e "${RED}❌ 镜像 $IMAGE_NAME 不存在！${NC}"
    echo -e "${YELLOW}请先构建镜像： docker build -t $IMAGE_NAME .${NC}"
    exit 1
fi

# 停止并移除已存在的容器
echo -e "${YELLOW}🔄 清理现有容器...${NC}"
docker stop $CONTAINER_NAME >/dev/null 2>&1
docker rm $CONTAINER_NAME >/dev/null 2>&1

# 设置X11权限（确保图形显示正常工作）
xhost +local:docker >/dev/null 2>&1

# 全权限启动容器
echo -e "${GREEN}🔥 以最高权限启动容器...${NC}"
docker run -itd \
    --name $CONTAINER_NAME \
    --privileged \
    --network host \
    --ipc host \
    --pid host \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev:rw" \
    --volume="/sys:/sys:rw" \
    --volume="/proc:/proc:rw" \
    --volume="/run:/run:rw" \
    --volume="/var/run/docker.sock:/var/run/docker.sock" \
    --volume="$HOST_WS_PATH:$CONTAINER_WS_PATH:rw" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/etc/machine-id:/etc/machine-id:ro" \
    --security-opt seccomp=unconfined \
    --security-opt apparmor=unconfined \
    --cap-add=ALL \
    --ulimit memlock=-1 \
    --ulimit stack=8277716992 \
    $IMAGE_NAME \
    /bin/bash -c "tail -f /dev/null"

# 检查启动状态
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ 容器启动成功！${NC}"
    echo ""
    echo -e "${YELLOW}📋 容器信息：${NC}"
    echo -e "名称: ${GREEN}$CONTAINER_NAME${NC}"
    echo -e "镜像: ${GREEN}$IMAGE_NAME${NC}"
    echo -e "工作空间: ${GREEN}$HOST_WS_PATH → $CONTAINER_WS_PATH${NC}"
    echo -e "权限: ${RED}完全特权模式${NC}"
    echo -e "网络: ${YELLOW}主机模式${NC}"
    echo -e "图形: ${GREEN}已启用${NC}"
    echo ""
    echo -e "${YELLOW}🎯 使用命令：${NC}"
    echo -e "进入容器: ${GREEN}docker exec -it $CONTAINER_NAME bash${NC}"
    echo -e "查看日志: ${GREEN}docker logs $CONTAINER_NAME${NC}"
    echo -e "停止容器: ${GREEN}docker stop $CONTAINER_NAME${NC}"
    echo ""
    echo -e "${GREEN}🌟 容器已就绪，可以开始机械臂控制！${NC}"
else
    echo -e "${RED}❌ 容器启动失败！${NC}"
    exit 1
fi

# 自动进入容器
echo -e "${YELLOW}⏳ 3秒后自动进入容器...${NC}"
sleep 3
docker exec -it $CONTAINER_NAME /bin/bash
