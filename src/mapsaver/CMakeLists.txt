cmake_minimum_required(VERSION 2.8.3)
project(mapsaver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -g ")

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  image_transport
  rosbag_storage
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS serialization thread timer chrono) # Find Boost

# Define catkin package
catkin_package(
  CATKIN_DEPENDS cloud_msgs rosbag_storage
  DEPENDS PCL
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS} # Include Boost headers
)

# Define executable
add_executable(mapsaver src/mapsaver.cpp)

# Add dependencies
add_dependencies(mapsaver ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)

# Link libraries
target_link_libraries(mapsaver
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
  ${Boost_LIBRARIES} # Link against Boost libraries
)
