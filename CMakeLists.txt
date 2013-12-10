cmake_minimum_required(VERSION 2.8.3)
project(ladder_shaper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp visualization_msgs tf icp_server_msgs std_msgs message_generation)

add_message_files(
  DIRECTORY
  msg
  FILES
  LadderState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

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
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf resource_retreiver icp_server_msgs std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fit_ladder src/fit_ladder.cpp)
target_link_libraries(fit_ladder
   ${catkin_LIBRARIES}
)
add_dependencies(fit_ladder ${catkin_EXPORTED_TARGETS})

#add_executable(load_ladder src/load_ladder.cpp)
#target_link_libraries(load_ladder
#   ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS 
  fit_ladder
#  load_ladder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
