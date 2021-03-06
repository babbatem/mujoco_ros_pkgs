cmake_minimum_required(VERSION 2.8.3)
project(mujoco_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  controller_manager
  control_toolbox
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  mujoco_ros_msgs
  roslint
  tf
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    tf
    cv_bridge
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} robot_hw_sim
)

## Specify additional locations of header files
include_directories( include
 ${Boost_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
)

## include mujoco using env variable.
include_directories($ENV{MUJOCO_PRO_PATH}/include)

## find libmujoco
file(GLOB LIB_MUJOCO $ENV{MUJOCO_PRO_PATH}/bin/libmujoco[0-9][0-9][0-9].so)

roslint_cpp()

add_library(robot_hw_sim src/robot_hw_sim.cpp)
target_link_libraries(robot_hw_sim ${catkin_LIBRARIES})

add_library(visualization_utils src/visualization_utils.cpp)
target_link_libraries(visualization_utils ${catkin_LIBRARIES})

## Declare a C++ library
add_executable(mujoco_ros_control src/mujoco_ros_control.cpp)
target_link_libraries(mujoco_ros_control
    ${catkin_LIBRARIES}
    glfw
    ${LIB_MUJOCO}
    robot_hw_sim
    visualization_utils
    GLEW
    glut
    X11
    GL
    GLU)

add_executable(mujoco_ros_record_test src/record_test.cpp)
target_link_libraries(mujoco_ros_record_test
    ${catkin_LIBRARIES}
    glfw
    ${LIB_MUJOCO}
    robot_hw_sim
    visualization_utils
    GLEW
    glut
    X11
    GL
    GLU)
