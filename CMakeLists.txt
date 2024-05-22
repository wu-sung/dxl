cmake_minimum_required(VERSION 3.16.3)
project(dxl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(/usr/local/include/dynamixel_sdk)

# Build
add_executable(pub src/pub.cpp src/dxl.cpp)
ament_target_dependencies(pub rclcpp geometry_msgs sensor_msgs cv_bridge OpenCV)
target_link_libraries(pub dxl_x64_cpp ${OpenCV_LIBRARIES})

add_executable(sub src/sub.cpp src/dxl.cpp)
ament_target_dependencies(sub rclcpp geometry_msgs sensor_msgs cv_bridge OpenCV)
target_link_libraries(sub dxl_x64_cpp ${OpenCV_LIBRARIES})

add_executable(sub_jetson src/sub_jetson.cpp)
ament_target_dependencies(sub_jetson rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(sub_jetson ${OpenCV_LIBRARIES})

install(TARGETS
  pub
  sub
  sub_jetson  
  DESTINATION lib/${PROJECT_NAME})

ament_package()