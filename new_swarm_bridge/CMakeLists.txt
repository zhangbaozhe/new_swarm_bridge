cmake_minimum_required(VERSION 3.0.2)
project(new_swarm_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,undefined")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra")

find_package(GameNetworkingSockets REQUIRED)
find_package(gflags REQUIRED)
find_package(pybind11 CONFIG REQUIRED)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES new_swarm_bridge
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS gflags
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  "/usr/local/include/GameNetworkingSockets"
)

add_executable(server 
  src/server.cpp)
target_link_libraries(server
  ${catkin_LIBRARIES}
  GameNetworkingSockets
  gflags)


add_executable(client
  src/client.cpp)
target_link_libraries(client
  ${catkin_LIBRARIES}
  GameNetworkingSockets
  gflags)


