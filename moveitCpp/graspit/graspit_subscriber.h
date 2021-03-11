//
// Created by andreas on 16.10.20.
//

#ifndef MOVEIT_GRASPIT_SUBSCRIBER_H
#define MOVEIT_GRASPIT_SUBSCRIBER_H
#include "../config.h"
#ifdef USE_GRASPIT_SUB
#include "ros/ros.h"
#include "own_msg/eigenGraspResult.h"
#include <mutex>
#include "vector"
class GraspItSub
{
public:
    GraspItSub(ros::NodeHandle &nodeHandle);
    GraspItSub();
    own_msg::eigenGraspResult getNextGraspResult();
    std::vector<own_msg::eigenGraspResult> getAllGrasps();
    static int iNumberReceivedGrasps; /*!< how many grasps were received so far */
private:
    static std::mutex waitForAllGrasps; /*!< mutex to wait for all grasps to be received,
 * works when only one grasp planning component is active running */
    static void graspItCallback(const own_msg::eigenGraspResult eigenGraspResult);
    ros::Subscriber subscriber; /*!< subscriber to receive grasps */
    static std::vector<own_msg::eigenGraspResult> receivedGrasps; /*!< vector with all received grasps */
    int iGraspsReturned = 0; /*!< number of grasps that are already returned */
};

#endif /*USE_GRASPIT_SUB*/
#endif //MOVEIT_GRASPIT_SUBSCRIBER_H
