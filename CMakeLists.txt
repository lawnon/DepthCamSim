#Datei: CMakeLists.txt
#Beschreibung: Installation Installation der Pakets und
#              Abhaengigkeiten.
#Autor: Chukwunonso Bob-Anyeji
#Date: 18.07.2025
#

cmake_minimum_required(VERSION 3.5)

project(depthcam_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  xacro
  image_transport_plugins
  robot_state_publisher
  ros_gz_bridge
  ros_gz_sim
  ros_gz_image
  rviz2
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY launch rviz world
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# find cpp Navigation Software Package dependences
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

#include_directories(${CMAKE_BINARY_DIR})
add_executable(depthcam_node src/depthcam_node.cpp)
target_link_libraries(depthcam_node
  gz-transport${GZ_TRANSPORT_VER}::core
  rclcpp::rclcpp
  rcpputils::rcpputils
  ${sensor_msgs_TARGETS}
)

ament_target_dependencies(depthcam_node std_msgs)

ament_package()
