//
// Created by andreas on 13.10.20.
//

#ifndef MOVEIT_CONFIG_H
#define MOVEIT_CONFIG_H

#define USE_GRASPIT_SUB
#define USE_FRAME_TRANSFORMATION
#define USE_CONVERT_TO_MOVEIT
//#define USE_HELP_POSE

#ifndef USE_FRAME_TRANSFORMATION
#warning "Frame transformation not used"
#endif /*!USE_FRAME_TRANSFORMATION*/

#define USE_RVIZ
#define USE_GAZEBO
#ifdef USE_GAZEBO
#define USE_GAZEBO_PUBS
#define USE_GAZEBO_SUBS
#define USE_ROS_SERVICE
#endif /*USE_GAZEBO*/

#define USE_PLAN
#define USE_MOVEIT_PUB

#define USE_ENVIRONMENT
#ifdef USE_ENVIRONMENT
#endif  /*USE_ENVIRONMENT*/

#define LOG_RESULTS
#ifdef LOG_RESULTS
#include <chrono>
#include <ctime>
#include "logfile_strings.h"
//#define EVALUATION
#endif /*LOG_RESULTS*/

#define NUMBER_JOINTS 5
#define NUMBER_FINGER 2
#define NUMBER_PARTS_GRIPPER 3
#define MAX_LENGTH_STRING 50
#define ROS_QUEUE_SIZE 50
#define BASE_FRAME "world"

#endif //MOVEIT_CONFIG_H
