cmake_minimum_required(VERSION 3.14)
project(unitree_lidar_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS} 
  include
  ../../../unitree_lidar_sdk/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ../../../unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)

# Lyrical removed ament_target_dependencies. Message packages export
# ${pkg_TARGETS}; C++ packages export namespaced targets.
set(_unitree_lidar_deps
  ${PCL_LIBRARIES}
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}/libunilidar_sdk2.a"
  Threads::Threads
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
if(TARGET pcl_conversions::pcl_conversions)
  list(APPEND _unitree_lidar_deps pcl_conversions::pcl_conversions)
else()
  # Jazzy exports the headers without a CMake target.
  target_include_directories(unitree_lidar_ros2_node PRIVATE ${pcl_conversions_INCLUDE_DIRS})
endif()
target_link_libraries(unitree_lidar_ros2_node ${_unitree_lidar_deps})

install(TARGETS
unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# install rviz file and launch file
install(FILES rviz/view.rviz launch/launch.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
