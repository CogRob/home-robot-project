cmake_minimum_required(VERSION 2.8.3)
project(coppeliasim_run)

set(EXECUTABLE_NAME "start_coppeliasim")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Catkin package
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp 
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
  src
)

## Declare a C++ executable
add_executable("${EXECUTABLE_NAME}" src/start_coppeliasim.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries("${EXECUTABLE_NAME}"  ${catkin_LIBRARIES} )
