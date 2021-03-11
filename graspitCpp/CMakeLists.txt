cmake_minimum_required(VERSION 3.17)
project(graspitCpp)

find_package(
        Qt4 COMPONENTS QtCore
)
find_package(
        Boost REQUIRED COMPONENTS
)
find_package(message_generation)
find_package(roscpp)
find_package(grasp_planning_graspit)
find_package(graspit_interface)
find_package(own_msg)

include_directories(
        ${graspit_interface_INCLUDE_DIRS}
        ${QT_INCLUDES}
        ${GRASPIT_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${grasp_planning_graspit_INCLUDE_DIRS}
        ${ROS_INCLUDE_DIRS}
        ${own_msg_INCLUDE_DIRS}
)

add_message_files(
        DIRECTORY /home/andreas/graspit_ws/src/graspit_interface/msg
        FILES
        Grasp.msg
)

set(CMAKE_CXX_STANDARD 11)

add_executable(graspitCpp main.cpp main.h utils/helper.cpp utils/helper.h utils/timer.h utils/LogFile.cpp utils/LogFile.h utils/Planner.cpp utils/Planner.h)

target_link_libraries(graspitCpp
        ${Boost_LIBRARIES}
        ${QT_LIBRARIES}
        ${grasp_planning_graspit_LIBRARIES}
        ${roscpp_LIBRARIES})
