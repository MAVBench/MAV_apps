cmake_minimum_required(VERSION 2.8.3)
project(ROSFI)

add_compile_options(-std=c++11 -fno-omit-frame-pointer)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  trajectory_msgs
  mavbench_msgs
  tf
  dynamic_reconfigure
  rosgraph_msgs
)

include_directories(SYSTEM "capstone/capstone.h")

find_library(CAPSTONE_LIB capstone)




configure_file (
  "${PROJECT_SOURCE_DIR}/src/config.h.in"
  "${PROJECT_BINARY_DIR}/src/config.h"
  )
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mavbench
#  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set (Airlib_addr  $ENV{AirSim_base_dir}/AirLib)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${common_utility_include} 
  $ENV{AirSim_base_dir}/AirLib/include
)

link_directories(
    $ENV{AirSim_base_dir}/AirLib/lib
    $ENV{AirSim_base_dir}/AirLib/deps/rpclib/lib
  /usr/local/cuda/lib64  #added by Behzad (for Behzad)
  ${Airlib_addr}/deps/MavLinkCom/lib  
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB SOURCES src/*.cpp src/*.h )

#add_executable(talker src/nodes/talker_node.cpp src/talker.cpp)
#add_executable(talker src/nodes/*.cpp src/talker.cpp)
add_executable(ROSFI ${SOURCES})

target_link_libraries(ROSFI ${catkin_LIBRARIES} ${CAPSTONE_LIB} util AirLib MavLinkCom rpc pthread)

#add_executable(listener src/listener.cpp src/nodes/listener_node.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener node_example_gencfg node_example_generate_messages_cpp)


