cmake_minimum_required(VERSION 2.8.3)
project(bug_flood)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bug_flood
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(bug_flood_lib
		src/envirnment.cpp
		src/helper_functions.cpp
		src/map.cpp
		src/envirnment.cpp
)

## Declare a cpp executable
# add_executable(bug_flood_node src/bug_flood_node.cpp)

add_definitions(-std=c++11)

add_executable(env_display src/env_display.cpp)
add_dependencies(env_display bug_flood_lib)
target_link_libraries(env_display bug_flood_lib ${catkin_LIBRARIES} )

add_executable(bug_flood
		src/bug_flood.cpp
)
add_dependencies(bug_flood bug_flood_lib)
target_link_libraries(bug_flood bug_flood_lib ${catkin_LIBRARIES} )

