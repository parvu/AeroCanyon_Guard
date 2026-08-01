cmake_minimum_required(VERSION 3.8)
project(phy_ai_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Native C++ Physics Bridge
add_executable(physics_bridge src/physics_bridge.cpp)
target_link_libraries(physics_bridge PRIVATE rclcpp::rclcpp geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs)

# Python scripts and packages setup
ament_python_install_package(${PROJECT_NAME})

# Installing assets
install(DIRECTORY worlds launch DESTINATION share/${PROJECT_NAME})
install(TARGETS physics_bridge DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/live_inference_node.py src/teleop_drone_keyboard.py DESTINATION lib/${PROJECT_NAME})

ament_package()
