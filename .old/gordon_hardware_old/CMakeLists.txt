cmake_minimum_required(VERSION 3.8)
project(gordon_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

find_package(pigpio REQUIRED)

add_executable(motor src/motor_node.cpp)
target_include_directories(motor PRIVATE include ${pigpio_INCLUDE_DIRS})
target_link_libraries(motor pigpiod_if2)
ament_target_dependencies(motor rclcpp)

add_executable(rotary_encoder src/rotary_encoder_node.cpp src/rotary_encoder.cpp)
target_include_directories(rotary_encoder PRIVATE include ${pigpio_INCLUDE_DIRS})
target_link_libraries(rotary_encoder pigpiod_if2)
ament_target_dependencies(rotary_encoder rclcpp std_msgs)

install(
  TARGETS rotary_encoder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
