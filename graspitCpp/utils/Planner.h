//
// Created by andreas on 14.11.20.
//

#ifndef GRASPITCPP_PLANNER_H
#define GRASPITCPP_PLANNER_H

#define NUMBER_SAVED_GRASPS 20
#include "vector"
#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>

std::vector<GraspIt::EigenGraspResult> plan(SHARED_PTR<GraspIt::GraspItSceneManager> graspItMgr, int iNumberAttempts);
std::vector<GraspIt::EigenGraspResult> searchGoodResults(std::vector<GraspIt::EigenGraspResult> &allGrasps, double dAcceptableEnergy);

#endif //GRASPITCPP_PLANNER_H
