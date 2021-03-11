//
// Created by andreas on 19.11.20.
//

#ifndef YOUBOT_MOVEITSUBSCRIBER_H
#define YOUBOT_MOVEITSUBSCRIBER_H
#include "ros/ros.h"
#include "own_msg/youBotJoints.h"

class MoveItSubscriber
{
public:
    MoveItSubscriber(ros::NodeHandle &nodeHandle);

    bool ready();
    bool getNextYouBotMotion(own_msg::youBotJoints &nextMotion);

private:
    ros::Subscriber subscriber; /*!< ros subscriber that receives motion plans*/
    static std::vector<own_msg::youBotJoints> allJoints; /*!< vector that contains received motion plans*/
    static bool bReady; /*!< true when all motion plans are received */
    int motionsReturned = 0; /*!< variable to remember how many motion plans were returned */

    static void moveitCallback(const own_msg::youBotJoints joints);
};

#endif //YOUBOT_MOVEITSUBSCRIBER_H
