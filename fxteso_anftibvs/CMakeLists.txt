cmake_minimum_required(VERSION 3.0.2)
project(fxteso_anftibvs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


catkin_package(
CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf2
)

include_directories(
include
${catkin_INCLUDE_DIRS}
${EIGEN_3_INCLUDE_DIRS}
)

#Image Features processing
add_executable(image_features_node src/image_features.cpp)
target_link_libraries(image_features_node  ${catkin_LIBRARIES})
target_link_libraries(image_features_node  ${OpenCV_LIBRARIES})

#IBVS-based FXTESO
add_executable(fxteso_node src/fixed_eso.cpp)
target_link_libraries(fxteso_node	${catkin_LIBRARIES})

#IBVS-Adaptive controller
add_executable(anftibvs_ctrl_node src/aibvs_pos_ctrl.cpp)
target_link_libraries(anftibvs_ctrl_node	${catkin_LIBRARIES})

#TD Attitude
add_executable(td_attitude_node src/td_attitude.cpp)
target_link_libraries(td_attitude_node	${catkin_LIBRARIES})

#TD Attitude Desired
add_executable(td_attitude_desired_node src/td_attitude_desired.cpp)
target_link_libraries(td_attitude_desired_node	${catkin_LIBRARIES})

#VICON MAVROS COM
add_executable(vicon_mavros_node src/vicon_mavros_v2.cpp)
target_link_libraries(vicon_mavros_node ${catkin_LIBRARIES})

#Quaternion to Euler 
add_executable(quat2eul_node src/imu_quat2eul_test.cpp)
target_link_libraries(quat2eul_node ${catkin_LIBRARIES})

#Control commands to pixhawk
add_executable(ctrl2pixhawk_node src/ctrl2pixhawk.cpp)
target_link_libraries(ctrl2pixhawk_node ${catkin_LIBRARIES})

#Thrust finder test with attVel Commands
add_executable(thrustFinder_node src/tests/thrust_finder.cpp)
target_link_libraries(thrustFinder_node ${catkin_LIBRARIES})

#Offboard switching test
add_executable(offb_switch_node src/tests/offb_switch_action.cpp)
target_link_libraries(offb_switch_node ${catkin_LIBRARIES})

#Thrust finder test with attitude commands
add_executable(thrustFinder2_node src/tests/thrust_finder2.cpp)
target_link_libraries(thrustFinder2_node ${catkin_LIBRARIES})

#Offboard example for position setpoints
add_executable(position_setpoints_example_node src/tests/position_setpoints.cpp)
target_link_libraries(position_setpoints_example_node ${catkin_LIBRARIES})

#Vicon Target
add_executable(vicon_target_node src/vicon_target.cpp)
target_link_libraries(vicon_target_node ${catkin_LIBRARIES})
