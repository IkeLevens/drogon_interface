cmake_minimum_required(VERSION 2.8.3)
project(drogon_interface)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(baxter_msgs_DIR ~/workspaces/baxter_moveit/devel/share/baxter_msgs/cmake)
#set(moveit_ros_move_group_DIR /opt/ros/hydro/share/moveit_ros_move_group/cmake)
find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy baxter_core_msgs sensor_msgs std_msgs moveit_ros_planning_interface moveit_ros_manipulation)
#find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy baxter_msgs sensor_msgs std_msgs moveit_core moveit_ros_planning_interface moveit_ros_manipulation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   JointCommandMode.msg
#   GripperCommand.msg
#   JointPositions.msg
#   JointVelocities.msg
#   JointState.msg
#   EndpointState.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#	FILES
#    SolvePositionIK.srv
#    )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#	DEPENDENCIES
#		std_msgs
#	sensor_msgs
#	JointCommandMode
#	GripperCommand
#	JointPositions
#	JointState
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS message_runtime roscpp rospy sensor_msgs std_msgs moveit_core moveit_ros_planning_interface moveit_ros_manipulation
	DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
#	:~/git/sdk-examples/baxter/baxter_msgs/srv_gen/cpp/include
#	:~/git/sdk-examples/baxter/baxter_msgs/msg_gen/cpp/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(drogon_interface_library
#	src/DrogonControlInterfaceLibrary.cpp
#)

## Declare a cpp executable
add_executable(endpoints src/Endpoint.cpp)
#add_executable(drogon_interface_node src/DrogonControlInterfaceKeyboard.cpp)
add_executable(moveit_test src/test.cpp)
#add_executable(planner src/planner.cpp)
add_executable(drogon_web_server src/WebServer.cpp)
add_executable(ik_test src/iktest.cpp)
add_executable(hard_test src/hardcodetest.cpp)
add_executable(workspace src/workspace.cpp)
add_executable(lines src/lines.cpp)
#add_executable(vilmi src/vilmi.cpp)
add_executable(curves src/curves.cpp)
add_executable(brian src/Brian.cpp)
add_executable(rahul src/Rahul.cpp)
add_executable(clear src/clear.cpp)
add_executable(trjparser src/trjparser.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(drogon_interface_node drogon_interface_generate_messages_cpp baxter_msgs drogon_interface_library)

## Specify libraries to link a library or executable target against
target_link_libraries(endpoints
	${catkin_LIBRARIES}
)
#target_link_libraries(drogon_interface_node
#	drogon_interface_library
#   ${catkin_LIBRARIES}
#)
#target_link_libraries(drogon_interface_library
#	${catkin_LIBRARIES}
#)
target_link_libraries(moveit_test
	${catkin_LIBRARIES}
)
#target_link_libraries(planner
#	drogon_interface_library
#	${catkin_LIBRARIES}
#)
target_link_libraries(drogon_web_server
	${catkin_LIBRARIES}
)
target_link_libraries(ik_test
	${catkin_LIBRARIES}
)
target_link_libraries(hard_test
	${catkin_LIBRARIES}
)
target_link_libraries(workspace
	${catkin_LIBRARIES}
)
target_link_libraries(lines
	${catkin_LIBRARIES}
)
#target_link_libraries(vilmi
#	${catkin_LIBRARIES}
#)
target_link_libraries(curves
	${catkin_LIBRARIES}
)
target_link_libraries(brian
	${catkin_LIBRARIES}
)
target_link_libraries(rahul
	${catkin_LIBRARIES}
)
target_link_libraries(clear
	${catkin_LIBRARIES}
)
target_link_libraries(trjparser
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS keyboard keyboard_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_keyboard.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
