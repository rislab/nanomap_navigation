cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(nanomap_navigation)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  cv_bridge
  image_transport
  sensor_msgs
  mavros_msgs
  pcl_ros
  nanomap_ros
  parameter_utils
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(orocos_kdl REQUIRED)
find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nanomap_ros
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

file(GLOB MS_SRC_FILES src/${PROJECT_NAME}/*.cpp)

add_library(nanomap_navigation ${MS_SRC_FILES})

# TODO: create a node from scratch
# add_executable(motion_selector_node nodes/motion_selector_node.cpp)
add_executable(nanomap_navigation_node nodes/nanomap_navigation_node.cpp)
target_link_libraries(nanomap_navigation PRIVATE nanomap_navigation
${catkin_LIBRARIES}
${PCL_LIBRARIES}
orocos-kdl
)