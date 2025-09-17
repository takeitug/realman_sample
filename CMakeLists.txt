cmake_minimum_required(VERSION 3.0.2)
project(realman_sample)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf2
  trajectory_msgs
  rm_msgs
  sensor_msgs
  kdl_parser
  urdf
)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realman_sample
#  CATKIN_DEPENDS control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning_interface roscpp rospy std_msgs tf2 trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(rm_twistclient src/rm_twistclient.cpp)
target_link_libraries(rm_twistclient ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(rm_twistclient ${catkin_EXPORTED_TARGETS})

add_executable(rm_teleop src/rm_teleop.cpp)
target_link_libraries(rm_teleop ${catkin_LIBRARIES})

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES})