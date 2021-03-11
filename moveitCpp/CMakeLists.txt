cmake_minimum_required(VERSION 3.17)
project(moveit)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        rviz
        moveit_ros_planning_interface
        genmsg
        gencpp
        moveit_tutorials
        grasp_planning_graspit
        )
find_package(
        Boost REQUIRED COMPONENTS
)

find_package(rviz_visual_tools REQUIRED COMPONENTS)
find_package(moveit_visual_tools REQUIRED COMPONENTS
        planning_interface
        move_group)
find_package(Boost REQUIRED COMPONENTS)
find_package(Qt4 COMPONENTS QtCore  REQUIRED)
find_package(own_msg)


include_directories(${moveit_INCLUDE_DIRS}
        ${moveit_tutorials_INCLUDE_DIR}
        ${grasp_planning_graspit_INCLUDE_DIRS}
        ~/moveit_ws/src/moveit_tutorials/doc/interactivity/include
        ${GRASPIT_INCLUDE_DIRS}
        ${QT_INCLUDES}
        ${ROS_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${own_msg_INCLUDE_DIRS}
        )

set(CMAKE_CXX_STANDARD 14)
add_executable(moveit main.cpp config.h ros/ros_service.cpp ros/ros_service.h gazebo/subscriber.cpp
        gazebo/subscriber.h gazebo/publisher.cpp gazebo/publisher.h
        graspit/graspit_subscriber.cpp graspit/graspit_subscriber.h math/frame_transformation.cpp
        math/frame_transformation.h main.h rviz/planner.cpp rviz/planner.h graspit/converter.cpp
        graspit/converter.h math/timer.h math/timer.cpp rviz/environment.cpp rviz/environment.h
        logfile_strings.h logging/LogFile.cpp logging/LogFile.h ros/MoveitPublisher.cpp ros/MoveitPublisher.h)
target_link_libraries(moveit
        ${Boost_LIBRARIES}
        ${moveit_visual_tools_LIBRARIES}
        ${catkin_LIBRARIES}
        ${QT_LIBRARIES}
        ${grasp_planning_graspit_LIBRARIES}
        ${roscpp_LIBRARIES})
