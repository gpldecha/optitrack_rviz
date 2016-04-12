cmake_minimum_required(VERSION 2.8.3)
project(optitrack_rviz)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp tf sensor_msgs
)

find_package(Armadillo)


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
catkin_package(
  CATKIN_DEPENDS message_runtime tf
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
  INCLUDE_DIRS include ${KALMAN_INCLUDE}
  LIBRARIES optitrack_rviz
  CATKIN_DEPENDS roscpp tf sensor_msgs
  DEPENDS roscpp tf sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library


set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")        ## Optimize
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/optitrack_rviz/listener.h
                 ${INCLUDE_DIR}/optitrack_rviz/broadcaster.h
                 ${INCLUDE_DIR}/optitrack_rviz/filter.h
                 ${INCLUDE_DIR}/optitrack_rviz/type_conversion.h
                 ${INCLUDE_DIR}/optitrack_rviz/input.h
                 ${INCLUDE_DIR}/optitrack_rviz/load.h
                 ${INCLUDE_DIR}/optitrack_rviz/print.h
                 ${INCLUDE_DIR}/optitrack_rviz/debug.h
)

set(SRC_FILES  ${SRC_DIR}/listener.cpp
               ${SRC_DIR}/broadcaster.cpp
               ${SRC_DIR}/filter.cpp
               ${SRC_DIR}/input.cpp
                ${SRC_DIR}/load.cpp
)

set(VIS_HEADER_FILES ${INCLUDE_DIR}/visualise/vis_points.h
                     ${INCLUDE_DIR}/visualise/vis_vector.h
                     ${INCLUDE_DIR}/visualise/vis_gmm.h
                     ${INCLUDE_DIR}/visualise/vis_line.h
                     ${INCLUDE_DIR}/visualise/vis_cylinder.h
                     ${INCLUDE_DIR}/visualise/vis_grid.h
                     ${INCLUDE_DIR}/visualise/colormap.h
                     ${INCLUDE_DIR}/visualise/vis_point_cloud.h

)
set(VIS_SRC_FILES  ${SRC_DIR}/vis_points.cpp
                   ${SRC_DIR}/vis_vector.cpp
                   ${SRC_DIR}/vis_gmm.cpp
                   ${SRC_DIR}/colormap.cpp
                   ${SRC_DIR}/vis_cylinder.cpp
                   ${SRC_DIR}/vis_grid.cpp
                   ${SRC_DIR}/vis_line.cpp
                   ${SRC_DIR}/vis_point_cloud.cpp

)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  ${VIS_HEADER_FILES}
 ${INCLUDE_DIR}/visualise
)


add_library(optitrack_rviz  ${SRC_FILES} ${VIS_SRC_FILES}   ${VIS_HEADER_FILES}  ${HEADER_FILES}          )
target_link_libraries(optitrack_rviz  ${catkin_LIBRARIES} armadillo)



## Declare a cpp executable
#add_executable(optitrack_broadcaster_node ${SRC_FILES} ${HEADER_FILES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(optitrack_broadcaster_node optitrack_broadcaster_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(optitrack_broadcaster_node
#   ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS optitrack_broadcaster optitrack_broadcaster_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_optitrack_broadcaster.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
