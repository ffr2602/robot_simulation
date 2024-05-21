cmake_minimum_required(VERSION 3.8)
project(robot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(gazebo_ros_omni_drive SHARED
  scripts/gazebo_ros_omni_drive/gazebo_ros_omni_drive.cpp
)
ament_target_dependencies(gazebo_ros_omni_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

ament_export_libraries(gazebo_ros_omni_drive)

install(TARGETS
gazebo_ros_omni_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch scripts description map world rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joy_control.py
  scripts/omni_controller.py
  scripts/tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
