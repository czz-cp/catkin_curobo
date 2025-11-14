FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1001
ENV TZ=Europe/Paris

# 1. 基础工具和依赖
RUN apt-get update \
    && apt-get install -y \
      build-essential \
      cmake \
      cmake-curses-gui \
      curl \
      gedit \
      gdb \
      git \
      iputils-ping \
      locales \
      locate \
      lsb-release \
      mercurial \
      nano \
      net-tools \
      python3 \
      python3-dev \
      python3-pip \
      python3-dbg \
      python3-empy \
      python3-numpy \
      python3-psutil \
      python3-venv \
      software-properties-common \
      sudo \
      tzdata \
      vim \
      wget \
    && apt-get clean

# 2. ViSP 第三方依赖
RUN apt-get update \
    && apt-get install -y \
      libdc1394-22-dev \
      libeigen3-dev \
      liblapack-dev \
      libopencv-dev \
      libv4l-dev \
      libx11-dev \
      libzbar-dev \
      nlohmann-json3-dev \
      libcoin-dev \
      libdmtx-dev \
      libgsl-dev \
      libjpeg-dev \
      libogre-1.9-dev \
      libois-dev \
      libpcl-dev \
      libpng-dev \
    && apt-get clean

# 3. 设置语言环境
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# 4. 添加用户
ENV USERNAME=vispci
RUN useradd -U --uid $USER_UID -ms /bin/bash ${USERNAME} \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && adduser ${USERNAME} sudo \
    && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/${USERNAME} \
    && adduser ${USERNAME} video

# 5. 切换到普通用户
USER ${USERNAME}
WORKDIR /home/$USERNAME
ENV QT_X11_NO_MITSHM=1

# 复制所有本地仓库到容器中
COPY ros_sources/rosdistro /tmp/rosdistro
COPY ros_sources/Universal_Robots_ROS_Driver /tmp/Universal_Robots_ROS_Driver
COPY ros_sources/universal_robot /tmp/universal_robot

# 6. 安装 ROS Noetic
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && sudo apt-get update \
    && sudo apt-get install -y ros-noetic-desktop-full \
    && sudo apt-get install -y python3-rosdep python3-rosinstall python3-wstool \
    && sudo rosdep init \
    && sudo mkdir -p /etc/ros/rosdep/sources.list.d \
    && sudo cp -r /tmp/rosdistro/* /etc/ros/rosdep/sources.list.d/ \
    && sudo chmod -R a+r /etc/ros/rosdep/sources.list.d/ \
    && rosdep update --rosdistro=noetic --include-eol-distros \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# 7. 安装 CasADi
RUN sudo apt-get install -y g++ gfortran liblapack-dev pkg-config \
    && pip3 install casadi

# 8. 安装 UR 机械臂驱动和 ROS 包
RUN sudo apt-get update && sudo apt-get install -y git
RUN mkdir -p ~/catkin_ws/src \
    && sudo cp -r /tmp/Universal_Robots_ROS_Driver ~/catkin_ws/src/ \
    && sudo cp -r /tmp/universal_robot ~/catkin_ws/src/

RUN cd ~/catkin_ws \
    && rosdep install --from-paths src --ignore-src -y --rosdistro noetic\
    && source /opt/ros/noetic/setup.bash \
    && catkin_make \
    && echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# 9. 安装 ViSP
ENV VISP_WS=/home/$USERNAME/visp-ws

COPY visp_sources/visp-images ${VISP_WS}/visp-images
COPY visp_sources/visp ${VISP_WS}/visp

RUN sudo mkdir -p ${VISP_WS}/visp-build \
    && sudo chmod 777 ${VISP_WS}/visp-build \
    && cd ${VISP_WS}/visp-build \
    && cmake ../visp \
    && make \
    && echo "export VISP_WS=${HOME}/visp-ws" >> ${HOME}/.bashrc \
    && echo "export VISP_INPUT_IMAGE_PATH=${HOME}/visp-ws/visp-images" >> ${HOME}/.bashrc

# 10. 设置环境变量
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc \
    && echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc

CMD ["/bin/bash"]
