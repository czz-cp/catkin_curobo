cmake_minimum_required(VERSION 3.0.2)
project(ur10e_vs_node)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# 查找 Catkin 和依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  ur_kinematics
)

# 查找系统依赖
find_package(PkgConfig REQUIRED)
#find_package(realsense2 REQUIRED)
# 手动设置 RealSense2 路径
set(REALSENSE2_INCLUDE_DIRS "/usr/include/librealsense2")
set(REALSENSE2_LIBRARIES "/usr/lib/x86_64-linux-gnu/librealsense2.so")
# 检查文件是否存在
if(EXISTS ${REALSENSE2_INCLUDE_DIRS}/rs.hpp AND EXISTS ${REALSENSE2_LIBRARIES})
    message(STATUS "Using manual RealSense2 configuration")
    include_directories(${REALSENSE2_INCLUDE_DIRS})
else()
    message(WARNING "RealSense2 not found, some features will be disabled")
endif()


find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
    set(Eigen3_LIBRARIES ${EIGEN3_LIBRARIES})  # Eigen3 是头文件库，通常没有库文件
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()

find_package(OpenMP REQUIRED)  # 可选：并行优化

# ==================== ViSP 手动配置 ====================
find_package(PkgConfig REQUIRED)
pkg_check_modules(VISP REQUIRED visp)

if(VISP_FOUND)
    message(STATUS "ViSP found via pkg-config:")
    message(STATUS "  Version: ${VISP_VERSION}")
    message(STATUS "  Includes: ${VISP_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${VISP_LIBRARIES}")
else()
    # 备用方法：手动查找
    find_path(VISP_INCLUDE_DIRS NAMES visp3/core/vpImage.h PATHS /usr/local/include)
    find_library(VISP_CORE_LIB NAMES visp_core PATHS /usr/local/lib)
    find_library(VISP_IO_LIB NAMES visp_io PATHS /usr/local/lib)
    find_library(VISP_SENSOR_LIB NAMES visp_sensor PATHS /usr/local/lib)
    find_library(VISP_BLOB_LIB NAMES visp_blob PATHS /usr/local/lib)
    find_library(VISP_GUI_LIB NAMES visp_gui PATHS /usr/local/lib)
    find_library(VISP_SENSOR_LIB visp_sensor PATHS /usr/local/lib REQUIRED)
    
    if(VISP_INCLUDE_DIRS AND VISP_CORE_LIB)
        set(VISP_FOUND TRUE)
        set(VISP_INCLUDE_DIRS ${VISP_INCLUDE_DIRS})
        set(VISP_LIBRARIES 
            ${VISP_SENSOR_LIB}
            ${VISP_CORE_LIB} 
            ${VISP_IO_LIB} 
            ${VISP_BLOB_LIB}
            ${VISP_GUI_LIB}
        )
        message(STATUS "ViSP found manually")
    else()
        message(FATAL_ERROR "ViSP not found")
    endif()
endif()

# ==================== PCL 手动配置 ====================
message(STATUS "Configuring PCL manually...")

# 设置 PCL 路径（根据您的系统路径）
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.10")
set(PCL_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")

# 查找 PCL 库文件
find_library(PCL_COMMON_LIB pcl_common
    PATHS ${PCL_LIBRARY_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

find_library(PCL_IO_LIB pcl_io
    PATHS ${PCL_LIBRARY_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

# 设置 PCL 库变量
set(PCL_LIBRARIES
    ${PCL_COMMON_LIB}
    ${PCL_IO_LIB}
)

# 验证 PCL 配置
if(PCL_COMMON_LIB AND PCL_IO_LIB AND EXISTS ${PCL_INCLUDE_DIRS}/pcl/common/common_headers.h)
    message(STATUS "PCL manually configured:")
    message(STATUS "  Include dirs: ${PCL_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${PCL_LIBRARIES}")
    set(PCL_FOUND TRUE)
else()
    message(FATAL_ERROR "PCL manual configuration failed")
endif()

# ==================== Librealsense2 配置 ====================
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 QUIET realsense2)

if(NOT REALSENSE2_FOUND)
    message(STATUS "realsense2 not found via pkg-config, trying manual search...")
    find_path(REALSENSE2_INCLUDE_DIRS 
        NAMES librealsense2/rs.hpp
        PATHS /usr/include /usr/local/include /opt/ros/noetic/include
    )
    find_library(REALSENSE2_LIBRARIES 
        NAMES realsense2
        PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib /opt/ros/noetic/lib
    )
    
    if(REALSENSE2_INCLUDE_DIRS AND REALSENSE2_LIBRARIES)
        set(REALSENSE2_FOUND TRUE)
        message(STATUS "Found realsense2 manually: ${REALSENSE2_LIBRARIES}")
    else()
        message(WARNING "realsense2 not found, some features may be disabled")
    endif()
endif()

# ==================== 包含目录 ====================
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${VISP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

if(REALSENSE2_FOUND)
    include_directories(${REALSENSE2_INCLUDE_DIRS})
endif()

# ==================== 声明 Catkin 包 ====================
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        roscpp std_msgs sensor_msgs geometry_msgs 
        tf2_ros tf2_geometry_msgs ur_kinematics
    DEPENDS 
        Eigen3
)

# ==================== 添加可执行文件 ====================
add_executable(${PROJECT_NAME}
    src/test.cpp
)

# ==================== 链接库 ====================
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${VISP_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
    ${OpenCV_LIBS}
    ${REALSENSE2_LIBRARIES}
    ${PCL_LIBRARIES}
)

# ==================== 编译选项 ====================
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -O2
    -march=native
)

# 添加特定编译器定义（如果需要）
target_compile_definitions(${PROJECT_NAME} PRIVATE
    -DVISP_HAVE_OPENCV
    -DVISP_HAVE_REALSENSE2
)

# ==================== 安装规则 ====================

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS
  /home/vispci/catkin/src/ur10e_vs_node/scripts/aruco_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  /home/vispci/catkin/src/ur10e_vs_node/scripts/mpc_controller_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  /home/vispci/catkin/src/ur10e_vs_node/scripts/tool_mpc_servoing_arm.py
  /home/vispci/catkin/src/ur10e_vs_node/scripts/tool_visual_servoing_mpc.py
  /home/vispci/catkin/src/ur10e_vs_node/scripts/start_vel_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)