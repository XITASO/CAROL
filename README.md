# COPING

## What it is
This is a setup to plan a grasp and a motion for a robot. A planned robot motion can be validated by a simulation.
Currently only Gazebo as simulation is implemented.
These components communicate with each other via ROS topics.
These components not only do the grasp or motion planning but contain also the ability to gain information from other components and react according to these events.
E.g. if the motion planner doesn't find a valid motion for the given grasps the grasp planner will generate new grasps.
Additionally these components are not dependent on each other.
### moveitCpp
Motion planner that uses the component [MoveIt!](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)that is integrated in ROS very well.
### graspitCpp
Grasp planner that uses [GraspIt!](https://graspit-simulator.github.io/build/html/index.html) to execute the grasp planning.
### youBot
Robot specific component that includes the drivers to control a robot. In this case the Kuka YouBot will be controlled.
