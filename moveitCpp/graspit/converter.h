//
// Created by andreas on 28.10.20.
//

#ifndef MOVEIT_CONVERTER_H
#define MOVEIT_CONVERTER_H
#include "../config.h"
#include "moveit_msgs/Grasp.h"
#include "own_msg/eigenGraspResult.h"
#include "../graspit/graspit_subscriber.h"
#include "vector"
#include <tf2_eigen/tf2_eigen.h>
#ifdef USE_CONVERT_TO_MOVEIT
class Converter
{
public:
//    Converter(GraspItSub graspItSub);
    Converter();
    std::vector<moveit_msgs::Grasp> getAllPossibleGrasps();
    Eigen::Isometry3d eigenFromOwnMsg(own_msg::eigenGraspResult result);
private:
    std::vector<own_msg::eigenGraspResult> allEigenGraspResults;
    std::vector<moveit_msgs::Grasp> allMoveItGrasps;
    geometry_msgs::PoseStamped poseFromEigenGrasp(Eigen::Isometry3d result);

};
#endif /*USE_CONVERT_TO_MOVEIT*/
#endif //MOVEIT_CONVERTER_H
