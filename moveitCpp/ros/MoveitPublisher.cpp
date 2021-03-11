//
// Created by andreas on 19.11.20.
//

#include "MoveitPublisher.h"

/**
 * @fn MoveitPublisher::MoveitPublisher(ros::NodeHandle &nodeHandle)
 * @brief constructor
 * @param nodeHandle current node handle so that we can create a publisher for trajectory messages
 */
MoveitPublisher::MoveitPublisher(ros::NodeHandle &nodeHandle)
{
    publisher = nodeHandle.advertise<own_msg::youBotJoints>("real_youbot", 10, true);
}

/**
 * @fn void MoveitPublisher::publishMotion(bool lastMotion)
 * @brief publishes the message created so far
 * @param lastMotion if it is the last motion to publish we should tell the robot so that this task can finish after this
 */
void MoveitPublisher::publishMotion(bool lastMotion)
{
    own_msg::youBotJoints message;
    message.data = motion;
    message.gripperValues = gripperJointValues;
    message.finished = (uint8_t) lastMotion;
    publisher.publish(message);

    /* delete all values as we don't need them anymore */
    motion.clear();
    gripperJointValues.clear();
}

/**
 * @fn void MoveitPublisher::addMotion(std::vector<trajectory_msgs::JointTrajectoryPoint> & points)
 * @brief adds a motion plan with all trajectory points for the robot
 * @param points vector containing all points of the motion plan
 */
void MoveitPublisher::addMotion(std::vector<trajectory_msgs::JointTrajectoryPoint> & points)
{
    motion = points;
}

/**
 * @fn void MoveitPublisher::addGripperJointValues(std::vector<double> & jointValues)
 * @brief add joint positions of the gripper for a closing grasp
 * @param jointValues vector with the values for the gripper
 */
void MoveitPublisher::addGripperJointValues(std::vector<double> & jointValues)
{
    for (int i = 0; i < jointValues.size(); ++i)
    {
        /* as we get the values in mm and moveit or real robot need the values in m we need to convert */
        jointValues.at(i) /= 1000;
    }
    gripperJointValues = jointValues;
}
