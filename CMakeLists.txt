cmake_minimum_required(VERSION 3.0.2)
project(can_ros1_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(CATKIN_DEPENDS
               roscpp
               tf 
               std_msgs 
               nav_msgs 
               geometry_msgs 
               visualization_msgs
               sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_receive_node src/can_receive.cpp src/can_receive_node.cpp)
target_link_libraries(can_receive_node ${catkin_LIBRARIES})

add_executable(can_send_node src/can_send.cpp src/can_send_node.cpp)
target_link_libraries(can_send_node ${catkin_LIBRARIES})

add_executable(imu_convert_node src/imu_convert.cpp src/imu_convert_node.cpp)
target_link_libraries(imu_convert_node ${catkin_LIBRARIES})

include_directories(/usr/local/include/yaml-cpp)
target_link_libraries(can_receive_node /usr/local/lib/libyaml-cpp.a)
