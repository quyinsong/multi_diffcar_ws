cmake_minimum_required(VERSION 3.0.2)
project(mydiffcar_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  urdf
  xacro
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mydiffcar_gazebo
#  CATKIN_DEPENDS gazebo_plugins gazebo_ros gazebo_ros_control urdf xacro
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
