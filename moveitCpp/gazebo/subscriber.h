//
// Created by andreas on 13.10.20.
//

#ifndef MOVEIT_SUBSCRIBER_H
#define MOVEIT_SUBSCRIBER_H
#include "../config.h"
#include "ros/ros.h"

#ifdef USE_GAZEBO_SUBS
#include <gazebo_msgs/ContactsState.h>

class GazeboSubscriber
{
public:
    GazeboSubscriber(ros::NodeHandle &nodeHandle);
    bool getCollision();
    void resetCollisionDetection();
private:
    bool initGazeboSubscriber(ros::NodeHandle &nodeHandle);
    static bool collisionDetected;
    static void gazeboCollisionCallback(gazebo_msgs::ContactsState contactsState);
    static bool bPrintedCollision;
};

#endif /*USE_GAZEBO_SUBS*/
#endif //MOVEIT_SUBSCRIBER_H
