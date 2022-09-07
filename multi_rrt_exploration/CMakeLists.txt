cmake_minimum_required(VERSION 2.8.3)
project(multi_rrt_exploration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
)


add_message_files(
  FILES
  PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package( 

CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########
# c++11 support required
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(global_rrt_detector src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(global_rrt_detector ${catkin_LIBRARIES})

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(local_rrt_detector ${catkin_LIBRARIES})
