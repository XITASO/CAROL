//
// Created by andreas on 28.10.20.
//

#include "converter.h"

#include <utility>

#ifdef USE_CONVERT_TO_MOVEIT

#endif /*USE_CONVERT_TO_MOVEIT*/

#ifdef USE_PICK
std::vector<moveit_msgs::Grasp> Converter::getAllPossibleGrasps()
{
    allEigenGraspResults = graspItSub.getAllGrasps();
    if(allMoveItGrasps.empty())
    {
        allMoveItGrasps.resize(allEigenGraspResults.size());
        for(int i = 0; i < allEigenGraspResults.size(); i ++)
        {
            allMoveItGrasps[i].grasp_pose.header.frame_id = BASE_FRAME;
            allMoveItGrasps[i].grasp_pose.pose.orientation = poseFromEigenGrasp(
                    eigenFromOwnMsg(allEigenGraspResults[i])).pose.orientation;
            allMoveItGrasps[i].grasp_pose.pose.position.x = -0.40;
            allMoveItGrasps[i].grasp_pose.pose.position.y = -0.05;
            allMoveItGrasps[i].grasp_pose.pose.position.z = 0.14;

            allMoveItGrasps[i].pre_grasp_approach.direction.header.frame_id = BASE_FRAME;
            /* Direction is set as positive x axis */
            allMoveItGrasps[i].pre_grasp_approach.direction.vector.x = 1.0;
            allMoveItGrasps[i].pre_grasp_approach.min_distance = 0.095;
            allMoveItGrasps[i].pre_grasp_approach.desired_distance = 0.015;

            allMoveItGrasps[i].post_grasp_retreat.direction.header.frame_id = BASE_FRAME;
            /* Direction is set as positive z axis */
            allMoveItGrasps[i].post_grasp_retreat.direction.vector.z = 1.0;
            allMoveItGrasps[i].post_grasp_retreat.min_distance = 0.1;
            allMoveItGrasps[i].post_grasp_retreat.desired_distance = 0.25;

            // Open Gripper
            allMoveItGrasps[i].pre_grasp_posture.joint_names.resize(2);
            allMoveItGrasps[i].pre_grasp_posture.joint_names[0] = "gripper_finger_joint_l";
            allMoveItGrasps[i].pre_grasp_posture.joint_names[1] = "gripper_finger_joint_r";
            allMoveItGrasps[i].pre_grasp_posture.points.resize(1);
            allMoveItGrasps[i].pre_grasp_posture.points[0].positions.resize(2);
            allMoveItGrasps[i].pre_grasp_posture.points[0].positions[0] = GRIPPER_LIMIT_MAX;
            allMoveItGrasps[i].pre_grasp_posture.points[0].positions[1] = GRIPPER_LIMIT_MAX;

            // Close gripper
            allMoveItGrasps[i].grasp_posture.joint_names.resize(2);
            allMoveItGrasps[i].grasp_posture.joint_names[0] = "gripper_finger_joint_l";
            allMoveItGrasps[i].grasp_posture.joint_names[1] = "gripper_finger_joint_r";
            allMoveItGrasps[i].grasp_posture.points.resize(1);
            allMoveItGrasps[i].grasp_posture.points[0].positions.resize(2);
            allMoveItGrasps[i].grasp_posture.points[0].positions[0] = GRIPPER_LIMIT_MIN;
            allMoveItGrasps[i].grasp_posture.points[0].positions[1] = GRIPPER_LIMIT_MIN;

        }
    }

    return allMoveItGrasps;
}
#endif /*USE_PICK*/

/**
 * @fn geometry_msgs::PoseStamped Converter::poseFromEigenGrasp(Eigen::Isometry3d result)
 * @brief converts a Eigen::Isometry3d object to a geometry_msgs::Pose Stamped object with \refitem BASE_FRAME as frame id
 * @param result Isometry3d object
 * @return PoseStamped
 */
geometry_msgs::PoseStamped Converter::poseFromEigenGrasp(Eigen::Isometry3d result)
{
    geometry_msgs::Pose  pose = tf2::toMsg(result);
    geometry_msgs::PoseStamped returnValue;
    returnValue.pose = pose;
    returnValue.header.frame_id = BASE_FRAME;
    return returnValue;
}

/**
 * @fn Eigen::Isometry3d Converter::eigenFromOwnMsg(own_msg::eigenGraspResult result)
 * @brief converts a own_msg type to a Eigen::Isometry3d object
 * @param result object that should be converted
 * @return converted object
 */
Eigen::Isometry3d Converter::eigenFromOwnMsg(own_msg::eigenGraspResult result)
{

    Eigen::Isometry3d returnValue;
    /* create transformation matrix from object frame to gripper frame */
    for(int i = 0; i < 16; i++)
    {
        if(i > 11 && i < 15)
        {
            returnValue.data()[i] = result.matrix.data[i] / 1000; /*translation values: conversion between unit mm and m*/
        }
        else
        {
            returnValue.data()[i] = result.matrix.data[i]; /* rotation values */
        }
    }

    return returnValue;
}

/**
 * @fn Converter::Converter()
 * @brief default constructor
 */
Converter::Converter()
{

}
