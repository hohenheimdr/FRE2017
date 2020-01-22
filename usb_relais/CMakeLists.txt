cmake_minimum_required(VERSION 2.8.3)
project(usb_relais)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	msgs
  	pcl_ros

)

find_package( PCL REQUIRED )
find_package(Boost REQUIRED )


## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS} 
	${YAML_CPP_SOURCE_DIR}
	${Boost_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usb_relais
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)




link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(usb_relais src/usb_relais.cpp )
target_link_libraries(usb_relais ${catkin_LIBRARIES})

add_executable(activate_relais src/activate_relais.cpp )
target_link_libraries(activate_relais ${catkin_LIBRARIES})


add_executable(activate_relais_with_pointcloud src/activate_relais_with_pointcloud.cpp )
target_link_libraries(activate_relais_with_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})











