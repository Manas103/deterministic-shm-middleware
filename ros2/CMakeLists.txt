cmake_minimum_required(VERSION 3.16)
project(shmmw_ros2_crossing CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ros2_crossing_pub ros2_crossing_pub.cpp ../src/perception/frame_gen.cpp)
target_include_directories(ros2_crossing_pub PRIVATE ../include)
ament_target_dependencies(ros2_crossing_pub rclcpp std_msgs)

add_executable(ros2_crossing_sub ros2_crossing_sub.cpp)
ament_target_dependencies(ros2_crossing_sub rclcpp std_msgs)
