#!/bin/bash

# source right catkin workspace so that roslaunch knows the necessary packages
source ~/moveit_ws/devel/setup.bash # TODO source your catkin_ws here

export USED_ROBOT=youbot
# TODO adapt it to the path where your binary is build
MOVEIT_CPP=/home/${USER}/PycharmProjects/moveitCpp/cmake-build-debug/moveit 
if [ "$USED_ROBOT" = "youbot_soft" ]; then
MOVEIT_LOG=/home/${USER}/logfiles/softgripper.log
else
MOVEIT_LOG=/home/${USER}/moveit.log
fi

MOVEIT_OBJECT=/home/${USER}/objects_graspit/
MOVEIT_TABLE=/home/${USER}/objects_moveit/table.stl 
MOVEIT_PRINTER=/home/${USER}/Ender-3/printer.stl

# TODO adapt it to the path where your binary is build
GRASPIT_CPP=/home/${USER}/PycharmProjects/graspitCpp/cmake-build-debug/devel/lib/graspitCpp/graspitCpp
GRASPIT_LOG=/home/${USER}/graspIt.log
GRASPIT_WORLDS=$GRASPIT/worlds/youbot_
GRASPIT_ROBOT=$USED_ROBOT/youbot.xml
declare -a experiments=("box_center" "cylinder" "nefertiti")
declare -a positions=("left" "front" "right")

for position in "${positions[@]}" # different robot positions
do
# start rviz for moveit and to set all necessary ros parameters e.g. robot_description
roslaunch ${USED_ROBOT}_real demo.launch &
sleep 10
# start gazebo as simulation 
if [ "$USED_ROBOT" = "youbot_soft" ]; then
roslaunch youbot_gazebo youbot_world.launch robot_position:=$position use_soft_gripper:=true &
else
roslaunch youbot_gazebo youbot_world.launch robot_position:=$position &
fi
sleep 10 # wait for roslaunches to be started
    for experiment in "${experiments[@]}" # different objects to grip
    do
        $GRASPIT_CPP $GRASPIT_WORLDS$experiment.xml 70000 $GRASPIT_LOG &
        $MOVEIT_CPP $MOVEIT_OBJECT$experiment.stl $MOVEIT_LOG $position 0.28 0.28 0.11 $MOVEIT_TABLE $MOVEIT_PRINTER
        sleep 40 # so that all processes can terminate and cpu cores can cool down
    done

# kill simulation and rviz
PID=$(pgrep roslaunch)
kill $PID
sleep 3
done
