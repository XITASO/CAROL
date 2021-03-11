//
// Created by andreas on 19.11.20.
//

#include "MoveItSubscriber.h"

std::vector<own_msg::youBotJoints> MoveItSubscriber::allJoints;
bool MoveItSubscriber::bReady = false;

/**
 * @fn void MoveItSubscriber::moveitCallback(const own_msg::youBotJoints joint)
 * @brief callback that receives the motion plans from motion planning component. Stores all received data in a vector
 * and if the last motion plan was found intern variable bReady will be set to true
 * @param joint
 */
void MoveItSubscriber::moveitCallback(const own_msg::youBotJoints joint)
{
    allJoints.push_back(joint);
    if (joint.finished == 1)
    {
        bReady = true;
    }
}

/**
 * @fn MoveItSubscriber::MoveItSubscriber(ros::NodeHandle &nodeHandle)
 * @brief constructor, creates ros subscriber in current ros node
 * @param nodeHandle current ros node
 */
MoveItSubscriber::MoveItSubscriber(ros::NodeHandle &nodeHandle)
{
    subscriber = nodeHandle.subscribe("real_youbot", 10, moveitCallback);
}

/**
 * @fn bool MoveItSubscriber::ready()
 * @brief tells the caller if all motions plans were received
 * @return intern ready variable set by the callback function
 */
bool MoveItSubscriber::ready()
{
    return bReady;
}

/**
 * @fn bool MoveItSubscriber::getNextYouBotMotion(own_msg::youBotJoints &nextMotion)
 * @brief fills the given reference for a motion plan with received data. After last motion plan was returned, this function
 * will only create an empty youBotJoints object
 * @param nextMotion motion plan after call this is filled with points of the motion plan for the robot
 * @return true if valid data were filled in given reference
 */
bool MoveItSubscriber::getNextYouBotMotion(own_msg::youBotJoints &nextMotion)
{
    bool bReturnValue;
    if(allJoints.size() > motionsReturned)
    {
        nextMotion = allJoints.at(motionsReturned);
        motionsReturned++;
        bReturnValue = true;
        /*if (motionsReturned == allJoints.size())
        {
            motionsReturned = 0;
        }*/
    }
    else
    {
        nextMotion = own_msg::youBotJoints();
        bReturnValue = false;
    }
    return bReturnValue;
}


