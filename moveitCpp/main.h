//
// Created by andreas on 16.10.20.
//

#ifndef MOVEIT_MAIN_H
#define MOVEIT_MAIN_H
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/Grasp.h>
#include "config.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#ifdef USE_GRASPIT_SUB
#include "graspit/graspit_subscriber.h"
#endif /*USE_GRASPIT_SUB*/

#ifdef USE_FRAME_TRANSFORMATION
#include "math/frame_transformation.h"
#endif /*USE_FRAME_TRANSFORMATION*/

#ifdef USE_ROS_SERVICE
#include "ros/ros_service.h"
#endif /*USE_ROS_SERVICE*/

#ifdef USE_GAZEBO_PUBS
#include "gazebo/publisher.h"
#endif /*USE_GAZEBO_PUBS*/

#ifdef USE_GAZEBO_SUBS
#include "gazebo/subscriber.h"
#endif /*USE_GAZEBO_SUBS*/

#ifdef USE_RVIZ
#include <moveit_visual_tools/moveit_visual_tools.h>
#endif /*USE_RVIZ*/

#if defined(USE_PLAN) || defined(USE_PICK)
#include "rviz/planner.h"
#endif /*USE_PLAN*/

#ifdef USE_ENVIRONMENT
#include "rviz/environment.h"
#endif

#ifdef LOG_RESULTS
#include "logging/LogFile.h"
#endif /*LOG_RESULTS*/
#include <iostream>

/*
 * joint limits:
 * joint[0] 0.001 - 5.8982
 * joint[1] 0.001 - 2.7042
 * joint[2] -5.1826 - (-0.001)
 * joint[3] 0 - 3.5769
 * joint[4] 0 - 5.8458 */
static const std::string PLANNING_GROUP = "youbot_arm";

#endif //MOVEIT_MAIN_H
