//
// Created by andreas on 19.11.20.
//

#ifndef YOUBOT_MANIPULATOR_H
#define YOUBOT_MANIPULATOR_H

#include "youbot/YouBotManipulator.hpp"
#include "youbot/YouBotGripper.hpp"
#include "own_msg/youBotJoints.h"
#include "math.h"
#include "../../definitions.h"

class Manipulator
{
public:
    Manipulator(char ** argv);
    ~Manipulator();
    bool executeMotion(own_msg::youBotJoints &joints);
    void readAllLimits();
    void testGrippers();

private:
    bool bSoftGripperUsed = false;
    typedef struct youbotJointLimits {double dLowerLimit; double dUpperLimit;} youbotJointLimits_t;
    youbotJointLimits_t atRealJointLimits[5] = { 0 };
    youbot::YouBotManipulator * manipulator; /*!< interface for youbot */

    double getAngleToSet(double dDesiredAngle, int iJointNumber);
};


#endif //YOUBOT_MANIPULATOR_H
