#include "main.h"

int8_t i8GraspSuccess = GRASP_FAILURE;
bool bCallbackCalled = true;

/**
 * @fn void graspItSuccessCallback(const std_msgs::Int8 data)
 * @brief callback that will be called when feedback from motion planning component is received
 * @param data feedback
 */
static void graspItSuccessCallback(const std_msgs::Int8 data)
{
    bCallbackCalled = true;
    i8GraspSuccess = data.data;
}


int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <world-filename> <number attempts> <logfile>" << std::endl;
        return 1;
    }
    char * pcGraspitHome = getenv("GRASPIT");
    if(pcGraspitHome == NULL)
    {
        std::cerr << "You need to set the GRASPIT environment variable" << std::endl;
        return 1;
    }

    char * pcUsedRobot = getenv(USED_ROBOT_ENV);
    if(pcUsedRobot == NULL)
    {
        std::cerr << "You have to define the robot used with env variable " << USED_ROBOT_ENV << std::endl;
        std::cerr << "You can either use" << USED_ROBOT_YOUBOT << " or " << USED_ROBOT_YOUBOT_SOFT << std::endl;
        exit(-1);
    }

    //ros::init(argc, argv, "graspItPublisher", ros::init_options::InitOption::AnonymousName);
    ros::init(argc, argv, "graspIt");
    ros::NodeHandle nodeHandle;

    /* subscriber who received information if more grasp planning is needed */
    ros::Subscriber graspItSuccessSub = nodeHandle.subscribe("graspItSuccess", 10, graspItSuccessCallback);
    /* publisher who sends good grasp results*/
    ros::Publisher graspItPublisher = nodeHandle.advertise<own_msg::eigenGraspResult>("grasps", NUMBER_SAVED_GRASPS);
    std::string sWorldFilename = argv[1];

    /* number grasps should be checked */
    int iNumberAttempts = std::stoi(argv[2]);
    std::vector<GraspIt::EigenGraspResult> allGrasps;
#ifdef LOG_RESULTS
    Timer timer;
    LogFile logFile;
    logFile.open(argv[3]);
    logFile.write("World: ", getWorldFromFilePath(sWorldFilename));
    logFile.write("Attempts: ", iNumberAttempts);
    logFile.write("Used search space type: 0", true);
#endif /*LOG_RESULTS*/

    if (sWorldFilename.empty())
    {
        std::cerr << "You have to specify a world" << std::endl;
        return 1;
    }

    /* Create the graspIt world manager. */
    SHARED_PTR<GraspIt::GraspItSceneManager> graspItMgr(new GraspIt::GraspItSceneManagerHeadless());
    /* Load the graspIt world*/
    graspItMgr->loadWorld(sWorldFilename);

    /* expect robot name as arg */
    char acRobotFileName[256] = {0};
    snprintf(acRobotFileName, 256, "%s%s%s/%s.xml", pcGraspitHome, "/models/robots/", pcUsedRobot, pcUsedRobot);
    graspItMgr->loadRobot(acRobotFileName, "youbot");



#ifdef USE_PLANNER



    /* when we don't receive good results we should stop the program at any point in the future */
    int iFailedGetGoodResults = 0;

    double dAcceptableEnergy = ACCEPTABLE_ENERGY_START;
    bool bFirstPlan = true;

    while(i8GraspSuccess == GRASP_FAILURE)
    {
        usleep(0.2 * US_TO_S);
        ros::spinOnce();
        if(bCallbackCalled == true)
        {
            if(i8GraspSuccess == GRASP_SUCCESS || i8GraspSuccess == GRASP_ABORT)
            {
                break;
            }

            if(iFailedGetGoodResults > 3)
            {
                own_msg::eigenGraspResult abortMessage;
                abortMessage.abort = 1;
                //graspItPublisher.publish(abortMessage);
                //break;
            }
            bCallbackCalled = false; /* reset value */
#ifdef LOG_RESULTS
            timer.reset();
#endif /*LOG_RESULTS*/
            
            allGrasps = plan(graspItMgr, iNumberAttempts);

#ifdef LOG_RESULTS
            logFile.write("Time to compute Grasps [s]: ", timer.elapsed());
#endif /*LOG_RESULTS*/
            if(bFirstPlan)
            {
                bFirstPlan = false;
                dAcceptableEnergy = allGrasps.at(0).getEnergy() + ACCEPTABLE_ENERGY_START;
            }
            std::cout << "All grasps difference: " << checkGraspsForDifferences(allGrasps) << std::endl;

            std::vector<GraspIt::EigenGraspResult> goodResults;

            goodResults = searchGoodResults(allGrasps, dAcceptableEnergy);
            int iSavedGrasps = goodResults.size();
            if(iSavedGrasps == 0)
            {
                iFailedGetGoodResults++;
            }
            else
            {
                iFailedGetGoodResults = 0;
            }
            std::cout << "Good results difference: " << checkGraspsForDifferences(goodResults) << std::endl;
#ifdef LOG_RESULTS
            logFile.write("Number computed grasps: ", iSavedGrasps);
#endif /*LOG_RESULTS*/
            if(iSavedGrasps == 0)
            {
                dAcceptableEnergy += 0.5; /* accept a worse grasp result next time */
                iFailedGetGoodResults ++;
                bCallbackCalled = true;
                /* as we have no good results to publish we won't get an answer from motion
                * planning component and we need to plan again*/
                continue;
            }
#else
            GraspIt::EigenGraspResult dummyResult;
    allGrasps.push_back(dummyResult);
#endif /*USE_PLANNER*/

            /*publish grasps*/
            publishAllGrasps(goodResults, graspItPublisher);
            /* delete all grasps */
            allGrasps.clear();
            goodResults.clear();
        }
    }

#ifdef LOG_RESULTS
    logFile.close();
#endif
    return 0;
}
