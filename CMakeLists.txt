cmake_minimum_required(VERSION 2.8.3)
project(ct_calibration)
add_definitions("-std=c++11")
set(CMAKE_BUILD_TYPE Release)#RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp calibration_common kinect camera_info_manager cv_bridge image_transport tf_conversions pcl_ros message_generation geometry_msgs sensor_msgs)


add_message_files(FILES
  CalibrationStatus.msg
  )

add_service_files(FILES CTSensor.srv CTTransform.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OpenMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

###################################################
## Declare things to be passed to other projects ##
###################################################
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp calibration_common camera_info_manager cv_bridge image_transport
#  DEPENDS eigen pcl opencv2
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

add_executable(ct_calibration
  src/ct_calibration.cpp              include/ct_calibration/ct_calibration.h
  src/ct_calibration_node.cpp         include/ct_calibration/ct_calibration_node.h
  src/ros_device.cpp                   include/ct_calibration/ros_device.h
  src/ct_checkerboard_extraction.cpp  include/ct_calibration/ct_checkerboard_extraction.h
)

## Add dependencies to the executable
## Specify libraries to link a library or executable target against
target_link_libraries(ct_calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Publish camera_info topic for stereo:
add_executable(stereo_publisher apps/stereo_publisher.cpp)

target_link_libraries(stereo_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)


add_executable(ct_define_reference_frame
  src/ct_calibration.cpp              include/ct_calibration/ct_calibration.h
  src/ct_define_reference_frame.cpp   include/ct_calibration/ct_define_reference_frame.h
  src/ros_device.cpp                   include/ct_calibration/ros_device.h
  src/ct_checkerboard_extraction.cpp  include/ct_calibration/ct_checkerboard_extraction.h
)

target_link_libraries(ct_define_reference_frame
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
