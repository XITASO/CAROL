//
// Created by andreas on 16.10.20.
//

#ifndef MOVEIT_FRAME_TRANSFORMATION_H
#define MOVEIT_FRAME_TRANSFORMATION_H


#include "../config.h"
#ifdef USE_FRAME_TRANSFORMATION
#include "own_msg/eigenGraspResult.h"
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include "../graspit/converter.h"
//#define USE_STD_OUT
class FrameTransformation
{
public:
    FrameTransformation(bool bSoftGripperUsed);
    geometry_msgs::PoseStamped getPoseFromEigenGraspResult(own_msg::eigenGraspResult eigenGraspResult, planning_scene::PlanningScene &planningScene);
    void setObjectWorldPose(geometry_msgs::Pose & boxPose);
    bool rotatePose(geometry_msgs::PoseStamped &pose, double dAngleX, double dAngleY, double dAngleZ);
private:
    bool bSoftGripperUsed;
    geometry_msgs::Pose objectPose; /*!< object pose in world */
    Eigen::Affine3d createRotationMatrix(double dAngleX, double dAngleY, double dAngleZ);
    tf2_ros::TransformBroadcaster tfb; /*!< debugging tool to make frame transformation visible in e.g. RViz*/
};
#endif /*USE_FRAME_TRANSFORMATION*/

#endif //MOVEIT_FRAME_TRANSFORMATION_H
