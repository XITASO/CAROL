//
// Created by andreas on 19.11.20.
//

#include "Manipulator.h"

/**
 * @fn Manipulator::Manipulator(char ** argv)
 * @brief constructor
 * @param argv arguments this executable gets from user and this argv should contain the config directory for the youbot
 */
Manipulator::Manipulator(char ** argv)
{
    this->manipulator = new youbot::YouBotManipulator("youbot-manipulator", argv[1]);

    char * pcUsedRobot = getenv(USED_ROBOT_ENV);
    if(pcUsedRobot == NULL)
    {
        std::cerr << "You have to define the robot used with env variable " << USED_ROBOT_ENV << std::endl;
        std::cerr << "You can either use" << USED_ROBOT_YOUBOT << " or " << USED_ROBOT_YOUBOT_SOFT << std::endl;
        exit(-1);
    }
    else if(strcmp(pcUsedRobot, USED_ROBOT_YOUBOT) == 0)
    {
        bSoftGripperUsed = false;
    }
    else if(strcmp(pcUsedRobot, USED_ROBOT_YOUBOT_SOFT) == 0)
    {
        bSoftGripperUsed = true;
    }
    else
    {
        std::cerr << "You have to define the robot used with env variable " << USED_ROBOT_ENV << std::endl;
        std::cerr << "You can either use " << USED_ROBOT_YOUBOT << " or " << USED_ROBOT_YOUBOT_SOFT << std::endl;
        exit(-1);
    }

    manipulator->doJointCommutation();

    /* force calibration because when there was too much resistance in the last action one joint made
    * the current joint values may not be what they should be */
    manipulator->calibrateManipulator(true); /* calibrate arm */
    manipulator->calibrateGripper(true); /* calibrate gripper */

    /* we want the gripper to be open when starting the motions*/
    if(bSoftGripperUsed == true)
    {
        /* as the softgripper open and closes inverse to the gripper mounting rail we need to to that here too */
        manipulator->getArmGripper().close();
    }
    else
    {
        manipulator->getArmGripper().open();
    }
    std::cout << "Ready to take arguments for real youbot" << std::endl;;
}

/**
 * @fn bool Manipulator::executeMotion(own_msg::youBotJoints &joints)
 * @brief executes a given motion plan for the youbot
 * @param joints own message that contains the information about the joint positions
 * @return true on success, false only when data vector of joints is empty
 */
bool Manipulator::executeMotion(own_msg::youBotJoints &joints)
{
    if (joints.data.empty())
    {
        return false;
    }
    std::vector<youbot::JointAngleSetpoint> jointValues;
    for(int i = 0; i < joints.data.size(); i++)
    {
        jointValues.clear();
        for(int j = 0; j < joints.data.at(i).positions.size(); j++)
        {
            youbot::JointAngleSetpoint jointAngleSetPoint;
            double dAngleToSet;
            dAngleToSet = getAngleToSet(joints.data.at(i).positions.at(j), j);
            jointAngleSetPoint.angle = dAngleToSet * radian;
            jointValues.push_back(jointAngleSetPoint);
        }
        manipulator->setJointData(jointValues);
        if(joints.data.size() - i > 5)
        {
            SLEEP_MILLISEC(100);
        }
        else
        {
            SLEEP_MILLISEC(1000);
        }
    }
    if(joints.gripperValues.empty() == false)
    {
        if(bSoftGripperUsed == true)
        {
            /* as the softgripper open and closes inverse to the gripper mounting rail we need to to that here too */
            manipulator->getArmGripper().open();
        }
        else
        {
            std::cout << "Set gripper values" << std::endl;
            std::cout << joints.gripperValues.at(0) << std::endl;
            std::cout << joints.gripperValues.at(1) << std::endl;
            youbot::GripperBarPositionSetPoint gripperBarPositionSetPoint;
            gripperBarPositionSetPoint.barPosition = abs(joints.gripperValues.at(0)) * meter;
            manipulator->getArmGripper().getGripperBar1().setData(gripperBarPositionSetPoint);
            gripperBarPositionSetPoint.barPosition = abs(joints.gripperValues.at(1)) * meter;
            manipulator->getArmGripper().getGripperBar2().setData(gripperBarPositionSetPoint);
        }
    }
    else
    {
        if(bSoftGripperUsed == true)
        {
            /* as the softgripper open and closes inverse to the gripper mounting rail we need to to that here too */
            manipulator->getArmGripper().close();
        }
        else
        {
            manipulator->getArmGripper().open();
        }
    }
    return true;
}

/**
 * @fn Manipulator::~Manipulator()
 * @brief destructor
 */
Manipulator::~Manipulator()
{
    manipulator->getArmGripper().close();
}

void Manipulator::readAllLimits()
{
    for(int i = 1; i <= 5; i++)
    {
        youbot::JointLimitsRadian jointLimits;
        quantity<plane_angle> lowerLimit;
        quantity<plane_angle> upperLimit;
        bool bAreLimitsActive;
        manipulator->getArmJoint(i).getConfigurationParameter(jointLimits);
        jointLimits.getParameter(lowerLimit, upperLimit, bAreLimitsActive);
        atRealJointLimits[i-1].dLowerLimit = lowerLimit.value();
        atRealJointLimits[i-1].dUpperLimit = upperLimit.value();
        std::string string1;
        jointLimits.toString(string1);
        std::cout << "Joint " << i << " " << string1 << std::endl;
    }
}

void Manipulator::testGrippers()
{
    std::cout << "Close grippers" << std::endl;
    manipulator->getArmGripper().open();
    //SLEEP_SEC(2);
    //manipulator->getArmGripper().close();
}

double Manipulator::getAngleToSet(double dDesiredAngle, int iJointNumber)
{
    double dRealAngleToSet;

    /* in home position joint nr. 3 has the upper limit as position
     * as we have a zero based array joint nr. 3 information are stored in field 2*/
    if(iJointNumber == 2)
    {
        dRealAngleToSet = atRealJointLimits[iJointNumber].dUpperLimit;
        /* check if the desired angle is in the allowed limits */
        if(abs(dDesiredAngle) >= abs(atRealJointLimits[iJointNumber].dUpperLimit)
           && abs(dDesiredAngle) < abs(atRealJointLimits[iJointNumber].dLowerLimit))
        {
            dRealAngleToSet = dDesiredAngle;
        }
    }
    else
    {
        dRealAngleToSet = abs(atRealJointLimits[iJointNumber].dLowerLimit);
        /* check if the desired angle is in the allowed limits */
        if(abs(dDesiredAngle) > abs(atRealJointLimits[iJointNumber].dUpperLimit)
           && abs(dDesiredAngle) <= abs(atRealJointLimits[iJointNumber].dLowerLimit))
        {
            dRealAngleToSet = abs(dDesiredAngle);
        }
    }


    return dRealAngleToSet;
}
