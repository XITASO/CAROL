//
// Created by andreas on 14.11.20.
//

#include "Planner.h"

/**
 * @fn std::vector<GraspIt::EigenGraspResult> plan(boost::shared_ptr<GraspIt::GraspItSceneManager> graspItMgr, int iNumberAttempts)
 * @brief Plans best grasps for a specified world.
 * @note Don't use the planner object in a while loop because otherwise there are some background processes in progress which
 * stress the cpu up to 100% even if it's not needed at this moment
 * @param graspItMgr
 * @param iNumberAttempts
 * @return
 */
std::vector<GraspIt::EigenGraspResult> plan(boost::shared_ptr<GraspIt::GraspItSceneManager> graspItMgr, int iNumberAttempts)
{
    /* Number of times to repeat the planning process */
    int iRepeatPlanning = 1;

    /* Finalize each planning result with an "auto-grasp" to ensure there really are
     contacts between fingers and objects (sometimes, the grasp result is just very
     close to the object, but not really touching it). */
    bool bFinishWithAutoGrasp = false;
    /* Create the planner which accesses the graspIt world. */
    std::string sName = "youBotPlanner";
    SHARED_PTR<GraspIt::EigenGraspPlanner> planner(new GraspIt::EigenGraspPlanner(sName, graspItMgr));

    /* By default, the last robot loaded and the last object loaded are to be used as the hand and the
             object to grasp for the planning process. You can use the other EigenGraspPlanner::plan()
             method with more parameters to change this. */
    if (planner->plan(iNumberAttempts, iRepeatPlanning, NUMBER_SAVED_GRASPS, bFinishWithAutoGrasp) == false)
    {
        std::cerr << "Error doing the planning." << std::endl;
    }

    std::vector<GraspIt::EigenGraspResult> returnGrasps;
    planner->getResults(returnGrasps);
    return returnGrasps;
}

/**
 * @fn std::vector<GraspIt::EigenGraspResult> searchGoodResults(std::vector<GraspIt::EigenGraspResult> &allGrasps, double dAcceptableEnergy)
 * @brief searches for the best grasp results depending on the energy that is acceptable for our use
 * @param allGrasps vector with all grasps possible
 * @param dAcceptableEnergy maximum energy that is acceptable for a grasp result
 * @return vector with all grasps that fit the criteria
 */
std::vector<GraspIt::EigenGraspResult> searchGoodResults(std::vector<GraspIt::EigenGraspResult> &allGrasps, double dAcceptableEnergy)
{
    std::vector<GraspIt::EigenGraspResult> goodResults;
    /* find only good grasp results */
    for (int i = 0; i < allGrasps.size(); i++)
    {
        /* we search for grasps with a energy lower than dAcceptableEnergy because 0 is the best value we could get */
        if(allGrasps.at(i).getEnergy() < dAcceptableEnergy)
        {
            goodResults.push_back(allGrasps.at(i));
        }
        else
        {
            /* as soon as there is a grasp with a energy that we don't want there won't come more grasps we would accept*/
            break;
        }
    }
    return goodResults;
}
