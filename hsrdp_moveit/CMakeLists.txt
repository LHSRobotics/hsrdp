cmake_minimum_required(VERSION 2.8.3)
project(hsrdp_moveit)

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS moveit_ros_planning_interface roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

#add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)

#add_executable(NoughtsAndCrossesControl src/NoughtsAndCrossesControl.cpp)

#target_link_libraries(
#  move_group_interface_tutorial
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(
#  NoughtsAndCrossesControl
#  ${catkin_LIBRARIES}
#)


#catkin_package()


#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
