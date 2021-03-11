//
// Created by andreas on 20.10.20.
//

#ifndef MOVEIT_PLANNER_H
#define MOVEIT_PLANNER_H

#include "../config.h"

#if defined(USE_PLAN) || defined(USE_PICK)
#include "moveit/move_group_interface/move_group_interface.h"
#include "../graspit/graspit_subscriber.h"
#include "../ros/MoveitPublisher.h"
#include "../math/frame_transformation.h"
#include <std_msgs/Int8.h>
#include <math.h>
#include "../../definitions.h"
#include "../math/timer.h"
#include "environment.h"
#ifdef USE_GAZEBO
#include "../ros/ros_service.h"
#endif 
#ifdef LOG_RESULTS
#include "../logging/LogFile.h"

extern LogFile logFile;
#endif /*LOG_RESULTS*/
#define MINUTES_UNTIL_ABORT 15

class Planner
{
public:
    Planner(GraspItSub *subscriber, planning_scene::PlanningScene *planningScene, moveit_msgs::CollisionObject &gripperObject, ros::NodeHandle &nodeHandle);
    ~Planner();

    bool plan(moveit::planning_interface::MoveGroupInterface &moveGroup);
    bool pickup(moveit::planning_interface::MoveGroupInterface &moveGroup);
    bool placeObject(moveit::planning_interface::MoveGroupInterface &moveGroup, geometry_msgs::Point position);

private:
    moveit::planning_interface::MoveGroupInterface::Plan myPlan; /*!< object that contains planned trajectory */
    ros::Publisher graspitSuccess; /*!< ros publisher which sends feedback to grasp planning component */
    ros::NodeHandle nodeHandle; /*!< current ros node handle so that we can create ros publisher */
    GraspItSub *subscriber; /*!< graspit subscriber which gives access to received grasps */
    planning_scene::PlanningScene *planningScene; /*!< planning scene that stores information about collision objects and their frames */
    moveit_msgs::CollisionObject gripperObject; /*!< object that should be gripped */
    double dGoalStateTranslationPrecision = 0.0015; /*!< allowed precision of the computed goal pose in translation [m]*/
    double dGoalStateOrientationPrecision = 0.04; /*!< allowed precision of the computed goal pose in orientation [rad]*/
    own_msg::eigenGraspResult choosedGrasp; /*!< grasp that lead to a successful motion planning */
    geometry_msgs::PoseStamped pickUpPose; /*!< goal pose when picking up the object */
    bool bSoftGripperUsed;
#ifdef USE_MOVEIT_PUB
    MoveitPublisher moveitPublisher = MoveitPublisher(nodeHandle);
#endif /*USE_MOVEIT_PUB*/
#ifdef USE_GAZEBO
    GazeboPublisher gazeboPublisher = GazeboPublisher(nodeHandle);
#endif /*USE_GAZEBO*/

    bool checkComputedGoalPose(moveit::planning_interface::MoveGroupInterface &moveGroup, geometry_msgs::Pose &goalPose, bool bPick);
    bool poseReachable(moveit::planning_interface::MoveGroupInterface &moveGroup, geometry_msgs::Pose &goalPose);
    bool goIntoHomePosition(moveit::planning_interface::MoveGroupInterface &moveGroup, bool bLastMotion);
    bool move(moveit::planning_interface::MoveGroupInterface &moveGroup, bool bLastMotion);
    bool detachObjectFromPlate(moveit::planning_interface::MoveGroupInterface &moveGroup);
    bool goIntoInitPosition(moveit::planning_interface::MoveGroupInterface &moveGroup);
};
#endif /*USE_PLAN*/

#endif //MOVEIT_PLANNER_H
