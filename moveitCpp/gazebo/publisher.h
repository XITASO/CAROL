//
// Created by andreas on 13.10.20.
//

#ifndef MOVEIT_PUBLISHER_H
#define MOVEIT_PUBLISHER_H
#include "../config.h"
#include "ros/ros.h"

#ifdef USE_GAZEBO_PUBS
#include <std_msgs/Float64.h>
#include <unistd.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <controller_manager_msgs/LoadController.h>
#include "controller_manager_msgs/UnloadController.h"
#include <controller_manager_msgs/SwitchController.h>
#include "subscriber.h"
#include <std_srvs/Empty.h>
#include "../rviz/environment.h"
#include "gazebo_msgs/DeleteModel.h"
typedef enum rosService {ROS_OK, ROS_ALREADY_INITIALIZED, ROS_ALREADY_STARTED, ROS_ERROR} rosService_t;

class GazeboPublisher
{
public:
    GazeboPublisher(ros::NodeHandle &nodeHandle);
    ~GazeboPublisher();
    void gazeboPublishValueForJoint(std_msgs::Float64 f64Msg, int iJointNumber);
    bool publishTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory);
    void addSuccessfulTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory);
    void goIntoHomePosition();
    static rosService_t registerAndStartControllersForGazebo();
    rosService_t resetGazeboSimulation();
    rosService_t pauseGazeboSimulation();
    rosService_t resumeGazeboSimulation();
    void addObjectIntoSimulation();
    void deleteObjectFromSimulation();
private:
    bool initGazeboPublishers(ros::NodeHandle &nodeHandle);
    void callEmptyGazeboService(const std::string& sMessage);
    void unloadControllers();
    void resetGripperJoints();

    double adHomePositionJointValues[NUMBER_JOINTS] = {0};
    std::vector<trajectory_msgs::JointTrajectoryPoint> successfulTrajectory;
    std::string sObjectName;
    ros::NodeHandle nodeHandle;
    GazeboSubscriber gazeboSubscriber = GazeboSubscriber(nodeHandle);
    static std::vector<std::string> controllerNames;

};
#endif /*USE_GAZEBO_PUBS*/

#endif //MOVEIT_PUBLISHER_H
