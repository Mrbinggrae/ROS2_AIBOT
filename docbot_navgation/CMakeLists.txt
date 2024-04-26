cmake_minimum_required(VERSION 3.8)
project(docbot_navgation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(twist_stamper src/twist_stamper.cpp)
ament_target_dependencies(twist_stamper rclcpp geometry_msgs)

install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  twist_stamper 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
