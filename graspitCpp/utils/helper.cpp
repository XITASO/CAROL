//
// Created by andreas on 05.11.20.
//

#include "helper.h"

/**
 * @fn bool eigenGraspToGraspMsg(const GraspIt::EigenGraspResult& eigenGraspResult, own_msg::eigenGraspResult &ownResult, int iSavedGrasps)
 * @brief converts a graspit eigenGraspResult to a result that can be published to our interface
 * @param eigenGraspResult eigenGraspResult computed by graspit
 * @param ownResult result that will be published
 * @param iSavedGrasps number of graspit results because we will tell that to the motion planning component
 * @return true on success
 */
bool eigenGraspToGraspMsg(const GraspIt::EigenGraspResult& eigenGraspResult, own_msg::eigenGraspResult &ownResult, int iSavedGrasps)
{
    /* copy values of object-to-hand transformation matrix */
    for(int i = 0; i < 16; i++)
    {
        ownResult.matrix.data.push_back(eigenGraspResult.getObjectToHandTransform().data()[i]);
    }

    /* copy values of joints for a closing grasp */
    for(int i = 0; i < eigenGraspResult.getGraspJointDOFs().size(); i++)
    {
        ownResult.jointValues.push_back(eigenGraspResult.getGraspJointDOFs().at(i));
    }
    ownResult.energy = eigenGraspResult.getEnergy();
    ownResult.abort = 0; /* if grasp planning component doesn't find good results we can tell the motion planning component to stop */
    ownResult.numberGrasps = iSavedGrasps;
    return true;
}

/**
 * @fn std::string getWorldFromFilePath(std::string sFilename)
 * @brief isolates the world name from the whole filepath. The last string after a slash and before a dot will be
 * seen as world file name
 * @param sFilename filename with absolute/relative path
 * @return name of the world
 */
std::string getWorldFromFilePath(std::string sFilename)
{
    std::string sReturnValue;
    std::vector<std::string> results;

    boost::split(results, sFilename, [](char c){return c == '/';});

    /* here we have the filename with file ending */
    sReturnValue = results.at(results.size()-1);
    results.clear();

    /* get world name without filename*/
    boost::split(results, sReturnValue, [](char c){return c == '.';});
    return results.at(0);
}

/**
 * @fn void publishAllGrasps(std::vector<GraspIt::EigenGraspResult> allGrasps, ros::Publisher &publisher)
 * @brief publishes all grasps to the ros topic specified for the given publisher
 * @param allGrasps graspit results
 * @param publisher current ros publisher
 */
void publishAllGrasps(std::vector<GraspIt::EigenGraspResult> allGrasps, ros::Publisher &publisher)
{
    ros::Rate rate(10); /* [Hz] */
    int iSavedGrasps = allGrasps.size();
    for(int n = 0; n < iSavedGrasps; n ++)
    {
        own_msg::eigenGraspResult eigenGraspResult;
        eigenGraspToGraspMsg(allGrasps.at(n), eigenGraspResult, iSavedGrasps);
        publisher.publish(eigenGraspResult);
        /* as we don't have an async spinner we need to do the spinning by ourself so that message will arrive */
        ros::spinOnce();
        usleep(2 * 1000);
    }
    ROS_INFO("Grasps published");
}

/**
 * @fn bool checkGraspsForDifferences(std::vector<GraspIt::EigenGraspResult> graspVector)
 * @brief checks if in the given vector are grasps that really differ from each other or not
 * @param graspVector vector that contains all grasps
 * @return true if a certain difference was detected
 */
bool checkGraspsForDifferences(std::vector<GraspIt::EigenGraspResult> graspVector)
{
    double dDifferencePosition = 5; /* mm */
    double dDifferenceRotation = 0.2; /* rad */
    bool bDifferentGraspsDetected = false;
    std::vector<geometry_msgs::Pose> poses;
    for(int i = 0; i < graspVector.size(); i++)
    {
        poses.push_back(tf2::toMsg(graspVector.at(i).getObjectToHandTransform()));
    }

    for(int i = 0; i < graspVector.size(); i++)
    {
        for(int j = 1; j < graspVector.size(); j++)
        {
            if(abs(poses.at(i).orientation.x - poses.at(j).orientation.x) > dDifferenceRotation)
            {
                bDifferentGraspsDetected = true;
            }
            else if(abs(poses.at(i).orientation.y - poses.at(j).orientation.y) > dDifferenceRotation)
            {
                bDifferentGraspsDetected = true;
            }
            else if(abs(poses.at(i).orientation.z - poses.at(j).orientation.z) > dDifferenceRotation)
            {
                bDifferentGraspsDetected = true;
            }
            else if(abs(poses.at(i).orientation.w - poses.at(j).orientation.w) > dDifferenceRotation)
            {
                bDifferentGraspsDetected = true;
            }
            else if(abs(poses.at(i).position.x - poses.at(j).position.x) > dDifferencePosition)
            {
                bDifferentGraspsDetected = true;
            }
            else if(abs(poses.at(i).position.y - poses.at(j).position.y) > dDifferencePosition)
            {
                bDifferentGraspsDetected = true;
            }
            else if(abs(poses.at(i).position.z - poses.at(j).position.z) > dDifferencePosition)
            {
                bDifferentGraspsDetected = true;
            }
        }
    }
    return bDifferentGraspsDetected;
}
