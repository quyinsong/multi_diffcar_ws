cmake_minimum_required(VERSION 3.0.2)
project(nmpc_ctr)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nmpc_ctr
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ./include/nmpc_ctr/
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(test_nmpc_ctr_1 src/nmpc_ctr.cpp src/test_nmpc_ctr_1.cpp)
target_link_libraries(test_nmpc_ctr_1 ${catkin_LIBRARIES})
target_link_libraries(test_nmpc_ctr_1 /usr/local/lib/libcasadi.so.3.7) 

add_executable(formation_ctr_2 src/nmpc_ctr.cpp src/formation_ctr_2.cpp)
target_link_libraries(formation_ctr_2 ${catkin_LIBRARIES})
target_link_libraries(formation_ctr_2 /usr/local/lib/libcasadi.so.3.7) 

add_executable(formation_ctr_3 src/nmpc_ctr.cpp src/formation_ctr_3.cpp)
target_link_libraries(formation_ctr_3 ${catkin_LIBRARIES})
target_link_libraries(formation_ctr_3 /usr/local/lib/libcasadi.so.3.7) 
