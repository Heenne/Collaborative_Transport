cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
  rospy 
         
  tf2_ros 
  tf
  tf_conversions
 
  pluginlib
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw    
  hardware_interface
  
  geometry_msgs
  multi_robot_msgs
  nav_msgs  
  realtime_tools
 
  
)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   geometry_msgs#   nav_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/lyapunov.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_robot_controller
  CATKIN_DEPENDS roscpp
  DEPENDS tf 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)


##Macro fro generating dir names
macro(generate_dirs dir_name)
  string(TOUPPER ${dir_name} string_upper)
  set(${string_upper}_SRC_PATH "src/${dir_name}")
  set(${string_upper}_INCLUDE_PATH "include/multi_robot_controller/${dir_name}")
endmacro(generate_dirs)



##Feed forward
generate_dirs(feed_forward)
add_library(orientation_feed_forward
  ${FEED_FORWARD_INCLUDE_PATH}/msg_conversion.hpp

  ${FEED_FORWARD_INCLUDE_PATH}/orientation_feed_forward.h
  ${FEED_FORWARD_SRC_PATH}/orientation_feed_forward.cpp 

  ${FEED_FORWARD_INCLUDE_PATH}/ros_orientation_feed_forward_base.h
  ${FEED_FORWARD_SRC_PATH}/ros_orientation_feed_forward_base.cpp 

  ${FEED_FORWARD_INCLUDE_PATH}/msg_orientation_feed_forward.h
  ${FEED_FORWARD_SRC_PATH}/msg_orientation_feed_forward.cpp

  ${FEED_FORWARD_INCLUDE_PATH}/tf_orientation_feed_forward.h
  ${FEED_FORWARD_SRC_PATH}/tf_orientation_feed_forward.cpp

)
target_link_libraries(orientation_feed_forward
  ${catkin_LIBRARIES}
  ${eigen3_LIBRARIES}
  ${Franka_LIBRARIES}
)

#Controller
generate_dirs(controller)
add_library(controller

  ${CONTROLLER_INCLUDE_PATH}/controller.h
  ${CONTROLLER_SRC_PATH}/controller.cpp


  ${CONTROLLER_INCLUDE_PATH}/pass_through_controller.hpp

  ${CONTROLLER_INCLUDE_PATH}/lyapunov_controller.h
  ${CONTROLLER_SRC_PATH}/lyapunov_controller.cpp

)
target_link_libraries(controller
  rigid_motion
  input
  ${catkin_LIBRARIES}
  ${eigen3_LIBRARIES}
)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
add_dependencies(controller ${PROJECT_NAME}_gencfg)

#Rigid motion
generate_dirs(rigid_motion)
add_library(rigid_motion

    ${RIGID_MOTION_INCLUDE_PATH}/constrained_rigid_motion.h
    ${RIGID_MOTION_SRC_PATH}/constrained_rigid_motion.cpp

    ${RIGID_MOTION_INCLUDE_PATH}/constrained_rigid_motion_ros.h
    ${RIGID_MOTION_SRC_PATH}/constrained_rigid_motion_ros.cpp

    ${RIGID_MOTION_INCLUDE_PATH}/constrained_rigid_motion_tf.h
    ${RIGID_MOTION_SRC_PATH}/constrained_rigid_motion_tf.cpp
)
target_link_libraries(rigid_motion  
  ${catkin_LIBRARIES}
)

#Input
generate_dirs(input)
add_library(input
  ${INPUT_INCLUDE_PATH}/input_base.hpp
  ${INPUT_INCLUDE_PATH}/input_odom.hpp
  ${INPUT_INCLUDE_PATH}/input_pose_twist.hpp
  ${INPUT_INCLUDE_PATH}/input_pose_twist_local.hpp
  ${INPUT_INCLUDE_PATH}/input_pose_odom.hpp
  ${INPUT_INCLUDE_PATH}/input_types.hpp
)
target_link_libraries(input
  ${catkin_LIBRARIES}
)
set_target_properties(input PROPERTIES LINKER_LANGUAGE CXX)


#Exceptions
add_library(exceptions
  include/${PROJECT_NAME}/necessary_param_exeption.hpp
  include/${PROJECT_NAME}/input_alloc_exception.hpp
)
set_target_properties(exceptions PROPERTIES LINKER_LANGUAGE CXX)


##NODES
set(NODES_PATH "src/nodes")
add_executable(slave_node ${NODES_PATH}/slave_node.cpp)
add_executable(master_node ${NODES_PATH}/master_node.cpp)
add_executable(rigid_transformer ${NODES_PATH}/rigid_transformer.cpp)
add_executable(feed_forward_msg_node ${NODES_PATH}/feed_forward_msg_node.cpp)
add_executable(feed_forward_tf_node ${NODES_PATH}/feed_forward_tf_node.cpp)

target_link_libraries(rigid_transformer
  rigid_motion
  input
  ${catkin_LIBRARIES}
  ${eigen3_LIBRARIES}
)

target_link_libraries(slave_node
  controller
  ${catkin_LIBRARIES}
)

target_link_libraries(master_node
  controller
  ${catkin_LIBRARIES}
)

target_link_libraries(feed_forward_msg_node
orientation_feed_forward
  ${catkin_LIBRARIES}
  ${eigen3_LIBRARIES}
)
target_link_libraries(feed_forward_tf_node
orientation_feed_forward
  ${catkin_LIBRARIES}
  ${eigen3_LIBRARIES}
)


##Testfile ("No proper unittest")
set(TESTS_PATH "tests")
add_executable(input_test ${TESTS_PATH}/input_test.cpp)
add_executable(rigid_motion_test ${TESTS_PATH}/rigid_motion_test.cpp)
add_executable(controller_test ${TESTS_PATH}/controller_test.cpp)
add_executable(feed_forward_test ${TESTS_PATH}/orientation_feed_forward_test.cpp)
add_executable(dummie_base_move_broadcaster ${TESTS_PATH}/dummie_base_move_broadcaster.cpp)

target_link_libraries(input_test
  ${catkin_LIBRARIES}
)

target_link_libraries(rigid_motion_test
  controller
  input
  rigid_motion
  ${catkin_LIBRARIES}
  
)

target_link_libraries(controller_test
  ${catkin_LIBRARIES}
  controller
)

target_link_libraries(feed_forward_test
orientation_feed_forward
  ${catkin_LIBRARIES}
  ${eigen3_LIBRARIES}
)

target_link_libraries(dummie_base_move_broadcaster
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
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_multi_robot_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
