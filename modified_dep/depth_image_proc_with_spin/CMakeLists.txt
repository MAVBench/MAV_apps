cmake_minimum_required(VERSION 2.8)
project(depth_image_proc_with_spin)

add_definitions(-DROS)
set (common_utility_src $ENV{base_dir}/catkin_ws/src/MAV_apps/common)
find_package(catkin REQUIRED cmake_modules cv_bridge eigen_conversions image_geometry image_transport message_filters nodelet sensor_msgs stereo_msgs tf2 tf2_ros
    data_collection profile_manager)



set (Airlib_addr  $ENV{AirSim_base_dir}/AirLib)
link_directories(
    $ENV{AirSim_base_dir}/AirLib/lib
    $ENV{AirSim_base_dir}/AirLib/deps/rpclib/lib
  /usr/local/cuda/lib64  #added by Behzad (for Behzad)
  ${Airlib_addr}/deps/MavLinkCom/lib  
)



if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

find_package(Boost REQUIRED)
find_package(Eigen3 QUIET)

set (common_utility_src $ENV{base_dir}/catkin_ws/src/MAV_apps/common)
set (common_utility_include $ENV{base_dir}/catkin_ws/src/MAV_apps/common)

set (blah /home/nvidia/catkin_ws/devel/include)
message(STATUS ${common_utility_include})
message(STATUS ${catkin_INCLUDE_DIRS})

if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()
find_package(OpenCV REQUIRED)
include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} 
   ${blah} ${common_utility_include}  ${EIGEN3_INCLUDE_DIRS} 
  ${CATKIN_DEVEL_PREFIX}/include
  ${common_utility_include}/profile_manager/include
  ${common_utility_include}/data_collection/include
  $ENV{AirSim_base_dir}/AirLib/include
  )

#add_library(${PROJECT_NAME}_lib src/nodelets/convert_metric.cpp
#                             src/nodelets/crop_foremost.cpp
#                             src/nodelets/disparity.cpp
#                             src/nodelets/point_cloud_xyz.cpp
#                             src/nodelets/register.cpp
#			     ${common_utility_src}/Profiling.cpp 
#                ${common_utility_src}/common.cpp  ${common_utility_src}/Drone.cpp 
#                 )
#
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h")
#
#install(TARGETS ${PROJECT_NAME}_lib
#        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)


add_executable(depth_image_proc_with_spin src/nodelets/convert_metric.cpp
                             src/nodelets/crop_foremost.cpp
                             src/nodelets/disparity.cpp
                             src/nodelets/point_cloud_xyz.cpp
                             src/nodelets/register.cpp
			     ${common_utility_src}/Profiling.cpp 
                ${common_utility_src}/common.cpp  ${common_utility_src}/Drone.cpp 
                 )
target_link_libraries(depth_image_proc_with_spin ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
    AirLib MavLinkCom rpc pthread)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

