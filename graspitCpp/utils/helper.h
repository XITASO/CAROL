//
// Created by andreas on 05.11.20.
//

#ifndef GRASPITCPP_HELPER_H
#define GRASPITCPP_HELPER_H

#include "ros/ros.h"
#include <own_msg/eigenGraspResult.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>
#include "math.h"
#include <stdlib.h>

bool eigenGraspToGraspMsg(const GraspIt::EigenGraspResult& result, own_msg::eigenGraspResult &eigenResult, int iNumberGrasps);
std::string getWorldFromFilePath(std::string sFilename);
void publishAllGrasps(std::vector<GraspIt::EigenGraspResult> grasps, ros::Publisher & publisher);
bool checkGraspsForDifferences(std::vector<GraspIt::EigenGraspResult> graspVector);

#endif //GRASPITCPP_HELPER_H
