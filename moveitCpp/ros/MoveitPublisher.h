//
// Created by andreas on 19.11.20.
//

#ifndef MOVEIT_MOVEITPUBLISHER_H
#define MOVEIT_MOVEITPUBLISHER_H

#include "../config.h"
#ifdef USE_MOVEIT_PUB
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "own_msg/youBotJoints.h"

class MoveitPublisher
{
public:
    MoveitPublisher(ros::NodeHandle &nodeHandle);

    void publishMotion(bool lastMotion);
    void addMotion(std::vector<trajectory_msgs::JointTrajectoryPoint> &);
    void addGripperJointValues(std::vector<double> &);
private:
    ros::Publisher publisher;
    std::vector<double> gripperJointValues;
    std::vector<trajectory_msgs::JointTrajectoryPoint> motion;
};
#endif /*USE_MOVEIT_PUB*/

#endif //MOVEIT_MOVEITPUBLISHER_H
