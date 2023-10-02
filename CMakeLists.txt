cmake_minimum_required(VERSION 3.5)
project(lidar_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# gazebo_ros_joint_state_publisher example
add_library(lidar_3d_sensor SHARED
  src/lidar_3d_sensor.cpp
)
target_include_directories(lidar_3d_sensor PUBLIC include)
ament_target_dependencies(lidar_3d_sensor
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)
ament_export_libraries(lidar_3d_sensor)
if(ENABLE_PROFILER)
  target_include_directories(lidar_3d_sensor PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(lidar_3d_sensor ${ignition-common3_LIBRARIES})
endif()


install(TARGETS
        lidar_3d_sensor
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
