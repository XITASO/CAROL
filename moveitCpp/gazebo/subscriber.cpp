//
// Created by andreas on 13.10.20.
//

#include "subscriber.h"
#ifdef USE_GAZEBO_SUBS
bool GazeboSubscriber::collisionDetected = false;
bool GazeboSubscriber::bPrintedCollision = false;
ros::Subscriber collisionSubs[NUMBER_JOINTS + NUMBER_PARTS_GRIPPER];

GazeboSubscriber::GazeboSubscriber(ros::NodeHandle &nodeHandle)
{
    initGazeboSubscriber(nodeHandle);
}

/**
 * @fn void gazeboCollisionCallback(gazebo_msgs::ContactsState contactsState)
 * @brief callback for gazebo collision detection topic
 * @param contactsState: message with a header and a state vector of collisions if existing
 */
void GazeboSubscriber::gazeboCollisionCallback(gazebo_msgs::ContactsState contactsState)
{
    if(contactsState.states.empty() == false)
    {
        collisionDetected = true;
        if(bPrintedCollision == false)
        {
            std::cout << contactsState.states.at(0).collision1_name << " " << contactsState.states.at(0).collision2_name << std::endl;
            bPrintedCollision = true;
        }
    }
}

bool GazeboSubscriber::getCollision()
{
    return collisionDetected;
}

void GazeboSubscriber::resetCollisionDetection()
{
    bPrintedCollision = false;
    collisionDetected = false;
}

/**
 * @fn bool initGazeboSubscriber(ros::NodeHandle &nodeHandle)
 * @brief initialization of gazebo subscribers. These subscribers get the information whether there is a collision of not for all elements of the robot
 * @param nodeHandle: ros node that should subscribe to the certain topics of the robot
 * @return true if initialization successful otherwise false
 */
bool GazeboSubscriber::initGazeboSubscriber(ros::NodeHandle &nodeHandle)
{
    for(int i = 1; i <= NUMBER_JOINTS; i++)
    {
        char acNameTopic[MAX_LENGTH_STRING] = {0};
        sprintf(acNameTopic, "/youbot/link_%d_contact", i); /* topics for arm element collisions*/
        collisionSubs[i-1] = nodeHandle.subscribe(acNameTopic, ROS_QUEUE_SIZE, gazeboCollisionCallback);
    }

    /* topics for gripper elements */
    collisionSubs[NUMBER_JOINTS] = nodeHandle.subscribe("/youbot/palm_link_contact", ROS_QUEUE_SIZE, gazeboCollisionCallback);
    collisionSubs[NUMBER_JOINTS + 1] = nodeHandle.subscribe("/youbot/finger_l_contact", ROS_QUEUE_SIZE, gazeboCollisionCallback);
    collisionSubs[NUMBER_JOINTS + 2] = nodeHandle.subscribe("/youbot/finger_r_contact", ROS_QUEUE_SIZE, gazeboCollisionCallback);
    return true;
}
#endif /*USE_GAZEBO_SUBS*/
