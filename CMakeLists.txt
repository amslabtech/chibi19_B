cmake_minimum_required(VERSION 2.8.3)
project(chibi19_b)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roomba_500driver_meiji
  roscpp
  rospy
  std_msgs
  message_generation
  nav_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED) #追加
find_package(PythonLibs REQUIRED) #追加

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

##Generate messages in the 'msg' folder
add_message_files(
   FILES
   roomba.msg
 )

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
generate_messages(
   DEPENDENCIES
   std_msgs
 )

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
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
  LIBRARIES chibi19_b
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/chibi19_B.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/chibi19_B_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/my_python_script
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_chibi19_B.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(lader_scan_data src/lader_scan_data.cpp)
target_link_libraries(lader_scan_data ${catkin_LIBRARIES})

add_executable(roomba_ctrl src/roomba_ctrl.cpp)
target_link_libraries(roomba_ctrl ${catkin_LIBRARIES})

add_executable(listener_roomba_Odometry src/listener_roomba_Odometry.cpp)
target_link_libraries(listener_roomba_Odometry ${catkin_LIBRARIES})

add_executable(front_search src/front_search.cpp)
target_link_libraries(front_search ${catkin_LIBRARIES})

add_executable(roatation src/roatation.cpp)
target_link_libraries(roatation ${catkin_LIBRARIES})

add_executable(kadai2_22 src/kadai2_22.cpp)
target_link_libraries(kadai2_22 ${catkin_LIBRARIES})

##############################[Wataru-Y update 19-02-21]##########################################

add_executable(roomba_position src/roomba_position.cpp)
target_link_libraries(roomba_position ${catkin_LIBRARIES})
add_dependencies(roomba_position chibi19_b_generate_messages_cpp)

add_executable(LIDAR_Search src/LIDAR_Search.cpp)
target_link_libraries(LIDAR_Search ${catkin_LIBRARIES})
add_dependencies(LIDAR_Search chibi19_b_generate_messages_cpp)

add_executable(roomba_move src/roomba_move.cpp)
target_link_libraries(roomba_move ${catkin_LIBRARIES})
add_dependencies(roomba_move chibi19_b_generate_messages_cpp)

##############################[ichiro-t update 19-02-28]##########################################

add_executable(opencv_subscribe scripts/opencv_subscribe.py)
target_link_libraries(opencv_subscribe ${catkin_LIBRARIES})
set_target_properties(opencv_subscribe PROPERTIES LINKER_LANGUAGE Python)
if(OpenCV_FOUND)
  # インクルードパスを指定
  include_directories(${OpenCV_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
  # リンクするライブラリ指定
  target_link_libraries(opencv_subscribe ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
endif(OpenCV_FOUND)

#add_executable(opencv_subscribe scripts/opencv_subscribe.py)
#target_link_libraries(opencv_subscribe ${catkin_LIBRARIES})

add_executable(Localization src/Localization.cpp)
target_link_libraries(Localization ${catkin_LIBRARIES})
add_dependencies(Localization chibi19_b_generate_messages_cpp)

add_executable(global_path_planning src/global_path_planning.cpp)
target_link_libraries(global_path_planning ${catkin_LIBRARIES})

add_executable(DWA src/DWA.cpp)
target_link_libraries(DWA ${catkin_LIBRARIES})

add_executable(laser_test src/laser_test.cpp)
target_link_libraries(laser_test ${catkin_LIBRARIES})

add_executable(test_speed src/test_speed.cpp)
target_link_libraries(test_speed ${catkin_LIBRARIES})

add_executable(test_accel src/test_accel.cpp)
target_link_libraries(test_accel ${catkin_LIBRARIES})

add_executable(test_omega src/test_omega.cpp)
target_link_libraries(test_omega ${catkin_LIBRARIES})

add_executable(test_domega src/test_domega.cpp)
target_link_libraries(test_domega ${catkin_LIBRARIES})

add_executable(Localizer src/Localizer.cpp)
target_link_libraries(Localizer ${catkin_LIBRARIES})

add_executable(WhiteLineListener_test src/WhiteLineListener_test.cpp)
target_link_libraries(WhiteLineListener_test ${catkin_LIBRARIES})

add_executable(make_heatmap src/make_heatmap.cpp)
target_link_libraries(make_heatmap ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Local_path_planner src/Local_path_planner.cpp)
target_link_libraries(Local_path_planner ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

