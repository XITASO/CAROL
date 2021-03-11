//
// Created by andreas on 16.10.20.
//


#include "frame_transformation.h"

#ifdef USE_FRAME_TRANSFORMATION

/**
 * @fn void FrameTransformation::setObjectWorldPose(geometry_msgs::Pose & boxPose)
 * @brief set pose of the object that should be gripped in the end
 * @param boxPose pose of object
 */
void FrameTransformation::setObjectWorldPose(geometry_msgs::Pose & boxPose)
{
    this->objectPose = boxPose;
}

/**
 * @fn geometry_msgs::PoseStamped FrameTransformation::getPoseFromEigenGraspResult(own_msg::eigenGraspResult eigenGraspResult, planning_scene::PlanningScene &planningScene)
 * @brief calculates the endeffector pose in given world. Frame that is considered as goal frame is the last link of the youbot
 * as this one is equal to the frame of the gripper
 * @param eigenGraspResult contains the object to hand transformation matrix
 * @param planningScene information about the current frames in the world
 * @return goal pose for the youbot gripper
 */
geometry_msgs::PoseStamped FrameTransformation::getPoseFromEigenGraspResult(own_msg::eigenGraspResult eigenGraspResult, planning_scene::PlanningScene &planningScene)
{
    geometry_msgs::PoseStamped endeffectorPose;
    Eigen::Isometry3d tfWorldToBase = planningScene.getFrameTransform(BASE_FRAME);
    Converter converter; /* used to get object-to-hand-transformation matrix */
    Eigen::Isometry3d tfObjectToEndeffector;
    Eigen::Isometry3d tfWorldToObject; /* calculate robot base frame to object frame */

    /* transformations for debugging purpose */
    std::vector<geometry_msgs::TransformStamped> allTransformations;
    geometry_msgs::TransformStamped worldToObject;
    geometry_msgs::TransformStamped objectToEef;
    geometry_msgs::TransformStamped eefToLink;
    geometry_msgs::TransformStamped wholeWorldTransform;

    /* transformation matrix for a buffer between gripper and object because the real youbot
     * has a metallic gripper mounting rail */
    Eigen::Isometry3d translation;
    translation.setIdentity();
    translation(0,3) = 0;
    translation(1,3) = 0;
    if(bSoftGripperUsed == true)
    {
        /* gripper mounting rail not important for the robot when softgripper is used*/
        translation(2, 3) = 0;
    }
    else
    {
        translation(2, 3) = 0.012;
    }

    /* get object to hand transformation matrix */
    tfObjectToEndeffector = converter.eigenFromOwnMsg(eigenGraspResult);

    tf2::fromMsg(objectPose, tfWorldToObject);

    /* debugging tool to visualize frame transformation*/
    worldToObject = tf2::eigenToTransform(tfWorldToObject);
    worldToObject.child_frame_id = "printer";
    worldToObject.header.frame_id = BASE_FRAME;
    allTransformations.push_back(worldToObject);

    /* calculate the whole transformation */
    Eigen::Isometry3d tfWholeWorld;
    tfWholeWorld.matrix() = tfWorldToBase.inverse().matrix() * tfWorldToObject.matrix() *
            tfObjectToEndeffector.matrix() * translation.inverse().matrix();

    /* debugging tool to visualize frame transformation*/
    objectToEef = tf2::eigenToTransform(tfObjectToEndeffector);
    objectToEef.child_frame_id = "eef";
    objectToEef.header.frame_id = "printer";
    allTransformations.push_back(objectToEef);

    /* debugging tool to visualize frame transformation*/
    wholeWorldTransform = tf2::eigenToTransform(tfWholeWorld);
    wholeWorldTransform.child_frame_id = "goal_pose";
    wholeWorldTransform.header.frame_id = BASE_FRAME;
    allTransformations.push_back(wholeWorldTransform);

    tfb.sendTransform(allTransformations); /* send to transformations to tf package */
    endeffectorPose.pose = tf2::toMsg(tfWholeWorld);
    endeffectorPose.header.frame_id = planningScene.getPlanningFrame(); /* base frame of transformation */
    return endeffectorPose;
}

/***
 * @fn Eigen::Affine3d FrameTransformation::createRotationMatrix(double dAngleX, double dAngleY, double dAngleZ)
 * @param dAngleX: angle around the x-axis
 * @param dAngleY: angle around the y-axis
 * @param dAngleZ: angle around the z-axis
 * @return rotation matrix created from the three parameters
 */
Eigen::Affine3d FrameTransformation::createRotationMatrix(double dAngleX, double dAngleY, double dAngleZ)
{
    Eigen::Affine3d rx =
            Eigen::Affine3d(Eigen::AngleAxisd(dAngleX, Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d ry =
            Eigen::Affine3d(Eigen::AngleAxisd(dAngleY, Eigen::Vector3d(0, 1, 0)));
    Eigen::Affine3d rz =
            Eigen::Affine3d(Eigen::AngleAxisd(dAngleZ, Eigen::Vector3d(0, 0, 1)));
    return rz * ry * rx;
}

/***
 * @fn bool FrameTransformation::rotatePose(geometry_msgs::PoseStamped &pose, double dAngleX, double dAngleY, double dAngleZ))
 * @param pose pose that should be rotated
 * @param dAngleX: angle around the x-axis
 * @param dAngleY: angle around the y-axis
 * @param dAngleZ: angle around the z-axis
 * @return
 */
bool FrameTransformation::rotatePose(geometry_msgs::PoseStamped &pose, double dAngleX, double dAngleY, double dAngleZ)
{
    Eigen::Isometry3d startPose;
    tf2::fromMsg(pose.pose, startPose); /*geometry_msgs can not be rotated, we need other datatype*/

    /* real rotation*/
    startPose.matrix() = startPose.matrix() * createRotationMatrix(dAngleX, dAngleY, dAngleZ).matrix();

    pose.pose = tf2::toMsg(startPose);
    return true;
}

/**
 * @fn FrameTransformation::FrameTransformation(bool bSoftGripperUsed)
 * @brief Constructor that sets intern variable which contains information about the gripper used
 * @param bSoftGripperUsed
 */
FrameTransformation::FrameTransformation(bool bSoftGripperUsed)
{
    this->bSoftGripperUsed = bSoftGripperUsed;
}

#endif /*USE_FRAME_TRANSFORMATION*/
