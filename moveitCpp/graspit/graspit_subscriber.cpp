//
// Created by andreas on 16.10.20.
//

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include "graspit_subscriber.h"
#ifdef USE_GRASPIT_SUB

int GraspItSub::iNumberReceivedGrasps = 0;
std::mutex GraspItSub::waitForAllGrasps;
extern std::ofstream logFile;
std::vector<own_msg::eigenGraspResult> GraspItSub::receivedGrasps;

/**
 * @fn GraspItSub::GraspItSub(ros::NodeHandle &nodeHandle)
 * @brief constructor with the current ros node handle to create a subscriber
 * @param nodeHandle current ros node
 */
GraspItSub::GraspItSub(ros::NodeHandle &nodeHandle)
{
    subscriber = nodeHandle.subscribe("grasps", ROS_QUEUE_SIZE, graspItCallback);
    waitForAllGrasps.lock();
}

/**
* @fn void GraspItSub::graspItCallback(const own_msg::eigenGraspResult eigenGraspResult)
* @brief callback when a grasp is received from grasp planning component
* @param eigenGraspResult grasp to receive
*/
void GraspItSub::graspItCallback(const own_msg::eigenGraspResult eigenGraspResult)
{
    receivedGrasps.push_back(eigenGraspResult);
    iNumberReceivedGrasps++;
}

/**
* @fn own_msg::eigenGraspResult GraspItSub::getNextGraspResult()
* @brief get the next grasp result received from a grasp planning component
* @return next grasp result
*/
own_msg::eigenGraspResult GraspItSub::getNextGraspResult()
{
    /* wait on new grasp because either all grasps were already returned of none is received at all*/
    while(iGraspsReturned == iNumberReceivedGrasps || iNumberReceivedGrasps == 0)
    {
        sleep(5); /* avoid busy wait */
    }
    iGraspsReturned++;
    return receivedGrasps.at(iGraspsReturned-1);
}

/**
* @fn std::vector<own_msg::eigenGraspResult> GraspItSub::getAllGrasps()
* @brief get a vector with all received grasps so far
* @return all received grasps so far
*/
std::vector<own_msg::eigenGraspResult> GraspItSub::getAllGrasps()
{
    waitForAllGrasps.lock();
    return receivedGrasps;
}

/**
  * @fn GraspItSub::GraspItSub()
  * @brief default constructor
  */
GraspItSub::GraspItSub() = default;

#endif /*USE_GRASPIT_SUB*/
