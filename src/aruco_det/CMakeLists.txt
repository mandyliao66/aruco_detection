cmake_minimum_required(VERSION 3.0.2)
project(marker_det)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  geometry_msgs
eigen_conversions
std_msgs

)
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

find_package( Eigen3 REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aruco
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
CATKIN_DEPENDS roscpp cv_bridge image_transport geometry_msgs eigen_conversions std_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)
## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(aruco_det src/aruco_det.cpp)
target_link_libraries(aruco_det ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(aruco_test src/aruco_test.cpp)
target_link_libraries(aruco_test ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(aruco_simple src/aruco_simple.cpp)
target_link_libraries(aruco_simple ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/aruco.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

