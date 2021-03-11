//
// Created by andreas on 05.11.20.
//

#ifndef GRASPITCPP_MAIN_H
#define GRASPITCPP_MAIN_H

#define LOG_RESULTS

#include <cstdlib>
#include "ros/ros.h"
#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include "std_msgs/Int8.h"
#include "utils/helper.h"
#include "../definitions.h"
#include "utils/Planner.h"
#ifdef LOG_RESULTS
#include "utils/timer.h"
#include "utils/LogFile.h"
#endif /*LOG_RESULTS*/

#include <vector>
#include "std_msgs/String.h"

#define USE_PLANNER
//#define USE_STD_OUT
#define ACCEPTABLE_ENERGY_START 2.0
#define ACCEPTABLE_ENERGY_END 4.0
#define US_TO_S 1000000


#endif //GRASPITCPP_MAIN_H
