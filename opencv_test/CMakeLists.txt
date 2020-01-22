cmake_minimum_required(VERSION 2.8.3)
project(opencv_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  usb_cam
  cv_bridge
  sensor_msgs
  std_msgs
  image_transport
)

catkin_package(
  	CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
	include	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(opencv_test src/opencv_test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(opencv_test   ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

