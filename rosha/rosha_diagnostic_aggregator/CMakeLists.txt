cmake_minimum_required(VERSION 2.8.3)
project(rosha_diagnostic_aggregator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_msgs
  xmlrpcpp
  pluginlib
  rospy
  smile_lib
)

message ( "rosha_diagnostic_aggregator: catkin include dirs: " ${catkin_INCLUDE_DIRS})

message ( "rosha_diagnostic_aggregator: catkin libraries: " ${catkin_LIBRARIES})

#find_library(PVAPI NAMES "DiagnosticEngine" PATHS "/home/dominik/work/rosha_ws/src/rosha/rosha_diagnostic_aggregator/")

#message(STATUS " Hello ... ${PVAPI}")

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system regex)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES ${PVAPI}
  CATKIN_DEPENDS roscpp diagnostic_msgs xmlrpcpp pluginlib rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########


#if (CMAKE_SIZEOF_VOID_P MATCHES "8")
#        message("linking lib for a x_64 system")
#        set(BAYESIAN_LIB_PATH ${PROJECT_SOURCE_DIR}/bn_stuff/smile_linux_x64_gcc_4_4_5)
#else(CMAKE_SIZEOF_VOID_P MATCHES "8")
#       message("linking lib for a x86 system")
#        set(BAYESIAN_LIB_PATH ${PROJECT_SOURCE_DIR}/bn_stuff/smile_linux_x86_gcc_4_4_5)
#endif (CMAKE_SIZEOF_VOID_P MATCHES "8")



## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include	
  ${catkin_INCLUDE_DIRS}
 # ${BAYESIAN_LIB_PATH}
)

#link_directories(
  #${BAYESIAN_LIB_PATH}
  #/home/dominik/work/rosha_ws/src/rosha/smile_lib/lib/smile_linux_x64_gcc_4_4_5
  #libsmile.a  
  #/home/dominik/work/rosha_ws/src/rosha/rosha_diagnostic_aggregator/lib
  #/home/dominik/work/rosha_ws/devel/lib
#)

#
# Aggregator node
#
file(GLOB AGGREGATOR_NODE_FILES
        src/aggregator_node.cpp
) 
add_executable(rosha_aggregator_node ${AGGREGATOR_NODE_FILES})
set_target_properties(rosha_aggregator_node PROPERTIES COMPILE_FLAGS -Wno-write-strings -I${BAYESIAN_LIB_PATH} -ffast-math -fpermissive -g)
target_link_libraries(rosha_aggregator_node 
   ${catkin_LIBRARIES}
  rosha_diagnostic_aggregator 
  DiagnosticEngine
)

#test for precompiled libs ...
#
# DiagnosticEngine lib
#
file(GLOB DIAGNOSTIC_ENGINE_FILES
    src/DiagnosticEngine/*.cpp
    src/DiagnosticEngine/KnowledgeBase/*.cpp
    src/DiagnosticEngine/Reasoner/*.cpp
    src/AnalyzerHelper/*.cpp
)
add_library( DiagnosticEngine ${DIAGNOSTIC_ENGINE_FILES})
set_target_properties(DiagnosticEngine PROPERTIES COMPILE_FLAGS "-Wno-write-strings -ffast-math -fpermissive -g")
target_link_libraries(DiagnosticEngine
   ${catkin_LIBRARIES}
   #smile
   pthread
)

#
# basic analyzer lib
#
file (GLOB AGGREGATOR_ANALYZER_FILES
    src/status_item.cpp
    src/analyzer_group.cpp
    src/generic_analyzer.cpp
    src/discard_analyzer.cpp
    src/ignore_analyzer.cpp
    src/aggregator.cpp
  )
add_library(rosha_diagnostic_aggregator
    ${AGGREGATOR_ANALYZER_FILES}
 )
 target_link_libraries(rosha_diagnostic_aggregator
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/rosha_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS DiagnosticEngine
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
 #install(DIRECTORY include/${PROJECT_NAME}/
# install(DIRECTORY include/DiagnosticEngine/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # roshafile1
#   # roshafile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
