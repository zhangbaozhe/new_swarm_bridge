cmake_minimum_required(VERSION 3.0.2)
project(new_swarm_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,undefined")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra")

find_package(GameNetworkingSockets REQUIRED)
find_package(gflags REQUIRED)
find_package(PythonLibs)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pybind11_catkin
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS pybind11_catkin
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




add_executable(client
  src/client.cpp)
target_link_libraries(client
  ${catkin_LIBRARIES}
  GameNetworkingSockets
  gflags)

add_library(bridge_server
  src/BridgeServer.cpp)
target_link_libraries(bridge_server
  GameNetworkingSockets)

add_executable(server_test
  src/bridge_server_test.cpp)
target_link_libraries(server_test
  ${catkin_LIBRARIES}
  bridge_server
  pybind11::embed)

pybind_add_module(Bridge
  MODULE
  src/BridgeServerPythonInterface.cpp)
target_link_libraries(Bridge
  PRIVATE
  bridge_server)

catkin_install_python(PROGRAMS scripts/test.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS Bridge
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
)
