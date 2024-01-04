cmake_minimum_required(VERSION 3.5)
project(cpp-pubsub)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#talker에 필요한 것들
add_executable(cpp-pub src/cpp-pub.cpp)
ament_target_dependencies(cpp-pub rclcpp std_msgs)

#listener에 필요한 것들
add_executable(cpp-sub src/cpp-sub.cpp)
ament_target_dependencies(cpp-sub rclcpp std_msgs)

#설치
install(TARGETS
  cpp-pub
  cpp-sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
