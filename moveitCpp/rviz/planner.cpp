//
// Created by andreas on 20.10.20.
//

#include "planner.h"


#ifdef USE_PLAN

/**
 * @fn bool Planner::plan(moveit::planning_interface::MoveGroupInterface &moveGroup)
 * @brief plans a motion for given planning scene and object to grip.
 * If from current received grasps no valid motion plan could be planned, feedback will be send and we will wait for new grasps.
 * Reference frame is the first link of the youbot and goal reference frame is the last link of the arm
 * @param moveGroup move group interface with information about the robot arm
 * @return true on success
 */
bool Planner::plan(moveit::planning_interface::MoveGroupInterface &moveGroup)
{
    if(not goIntoInitPosition(moveGroup))
    {
        return false;
    }
    int iGraspsChecked = 0;
    int iPlannersChecked = 0; /* only important when only one grasp planning component is active */
    bool bPlanningSuccess = false;
    Timer completeTime; /* measure time for whole process */
#ifdef LOG_RESULTS
    bool bStartedTimer = false;
    double dWholeTime = 0;

    Timer timer; /* only time for motion planning*/
    double dMotionPlanningTime = 0;

    int iPlanningAttempts = 0;
#endif /*LOG_RESULTS*/
    std_msgs::Int8 feedback;
    moveGroup.setPoseReferenceFrame("arm_link_0");
    FrameTransformation frameTransformation(bSoftGripperUsed);
    frameTransformation.setObjectWorldPose(gripperObject.mesh_poses.at(0)); /* tell where object is in the world*/
    moveGroup.clearPathConstraints();
    moveGroup.setPlanningTime(7); /* max 9 seconds to find a motion plan*/
    moveGroup.allowReplanning(true);
    completeTime.reset(); /* start timer */
    while(bPlanningSuccess == false)
    {
        own_msg::eigenGraspResult nextGraspResult = subscriber->getNextGraspResult(); /* receive next grasp */
        if(nextGraspResult.abort == 1)
        {
            /* grasp planning component doesn't find good grasps so we will terminate also*/
            break;
        }
#ifdef LOG_RESULTS
        timer.reset();
#endif /*LOG_RESULTS*/
        geometry_msgs::PoseStamped goalPose = frameTransformation.getPoseFromEigenGraspResult(nextGraspResult, *planningScene);
        moveGroup.clearPoseTargets();
        moveGroup.setApproximateJointValueTarget(goalPose.pose); /* allow approximated solutions because it is only 5 DOF arm */

        if(poseReachable(moveGroup, goalPose.pose)) /* no need to plan a motion if pose isn't reachable with acceptable tolerances */
        {
#ifdef LOG_RESULTS
            iPlanningAttempts++;
#endif /*LOG_RESULTS*/
            bPlanningSuccess = (moveGroup.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        if(bPlanningSuccess)
        {
            bPlanningSuccess = checkComputedGoalPose(moveGroup, goalPose.pose, true);
            if(bPlanningSuccess)
            {
#ifdef USE_GAZEBO
                moveitPublisher.addGripperJointValues(nextGraspResult.jointValues);
#endif /*USE_GAZEBO*/
                if(move(moveGroup, false) == false)
                {
                    bPlanningSuccess = false; /* in simulation there was a collision so it is no successful grasp */
                }
                else
                {
                    /* valid motion plan found so tell grasp planning components to stop*/
#ifdef LOG_RESULTS
                    /* write value of quality criteria in log file*/
                    logFile.write(QUALITY_MEASURE, nextGraspResult.energy);
                    dMotionPlanningTime += timer.elapsed();
#endif /*LOG_RESULTS*/
#ifdef USE_MOVEIT_PUB
#ifndef USE_GAZEBO
                    moveitPublisher.addGripperJointValues(nextGraspResult.jointValues);
#endif /*!USE_GAZEBO*/
#endif /*USE_MOVEIT_PUB*/
                    /* publish that we have a success */
                    feedback.data = GRASP_SUCCESS;
                    graspitSuccess.publish(feedback);
                    choosedGrasp = nextGraspResult; /* save results for later use */
                    pickUpPose = goalPose; /* save pose for later when we need to detach the object from the surface */
                }
            }
        }
        iGraspsChecked++;
        if(iGraspsChecked == subscriber->iNumberReceivedGrasps)
        {
            if(bPlanningSuccess == false)
            {
#ifdef LOG_RESULTS
                dMotionPlanningTime += timer.elapsed();
#endif /*LOG_RESULTS*/
                iPlannersChecked++;
                /* when evaluation more grasp planning nodes are running
                 * limit is five minutes to find a valid motion plan */
                if(completeTime.elapsed() > MINUTES_UNTIL_ABORT * 60)
                {
                    /* abort condition is true so tell grasp planning components to stop */
                    feedback.data = GRASP_ABORT;
                    graspitSuccess.publish(feedback);
                    break;
                }
                else
                {
                    /* no valid motion plan found so ask for more grasps */
                    feedback.data = GRASP_FAILURE;
                    graspitSuccess.publish(feedback);
                }
            }
        }
    }
#ifdef LOG_RESULTS

    /* write all important stats in a log file*/
    dMotionPlanningTime += timer.elapsed();
    dWholeTime = completeTime.elapsed();
    logFile.write(GRASPS_CHECKED, iGraspsChecked);
    logFile.write(PLANNING_ATTEMPTS, iPlanningAttempts);
    logFile.write(PLANNING_SUCCESS, (int) bPlanningSuccess);
    logFile.write(TIME_MOTION_PLAN, dMotionPlanningTime);
    logFile.write(COMPLETE_TIME, dWholeTime);
#endif /*LOG_RESULTS*/

    return bPlanningSuccess;
}
#endif /*USE_PLAN*/

/**
 * @fn Planner(GraspItSub *subscriber, planning_scene::PlanningScene *planningScene, moveit_msgs::CollisionObject &gripperObject, ros::NodeHandle &nodeHandle)
 * @brief constructor
 * @param subscriber graspit subscriber so that we have access to received grasps
 * @param planningScene planning scene that stores information about collision objects and their frames
 * @param gripperObject object that should be gripped
 * @param nodeHandle current node handle so that we can create a ros publisher for motion planning feedback
 */
Planner::Planner(GraspItSub *subscriber, planning_scene::PlanningScene *planningScene, moveit_msgs::CollisionObject &gripperObject, ros::NodeHandle &nodeHandle)
{
    this->subscriber = subscriber;
    this->planningScene = planningScene;
    this->gripperObject = gripperObject;
    this->nodeHandle = nodeHandle;

    /* use int8 to have more options available
     * define following return values for this topic:
     * 0: no success
     * 1: success
     * -1: abort
     **/
    this->graspitSuccess = nodeHandle.advertise<std_msgs::Int8>("graspItSuccess", 5);

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
        ROS_WARN("You didn't specify a gripper used. You should do that with environment variable USED_ROBOT");
        ROS_WARN("Use now softgripper as default");
        bSoftGripperUsed = true;
    }
    gazeboPublisher.addObjectIntoSimulation();
}

/**
 * @fn ~Planner()
 * @brief delete all pointers
 */
Planner::~Planner()
= default;

/***
 * @fn bool Planner::checkComputedGoalPose(moveit::planning_interface::MoveGroupInterface &moveGroup, geometry_msgs::Pose &goalPose, bool bPick)
 * @brief check if the calculated motion plan contains the right goal pose at the end. resolution is defined as a attribute of this class.
 * When placing the object a higher tolerance in orientation is allowed because this should not be the part of which the whole process should fail
 * @param moveGroup: interface of moveit that holds information about the robot
 * @param goalPose: pose that should be reached by the motion plan
 * @param bPick: when picking up the object this has to be true
 * @return true if desired goal pose could be reached within a tolerance
 */
bool Planner::checkComputedGoalPose(moveit::planning_interface::MoveGroupInterface &moveGroup, geometry_msgs::Pose &goalPose, bool bPick)
{
    bool bReturnValue = true;
    double dGoalRotationPrecision;
    if(bPick)
    {
        dGoalRotationPrecision = dGoalStateOrientationPrecision;
    }
    else
    {
        dGoalRotationPrecision = 2 * dGoalStateOrientationPrecision;
    }

    /* prepare all needed variables */
    robot_state::RobotStatePtr kinematicState(new robot_state::RobotState(moveGroup.getRobotModel()));
    const robot_state::JointModelGroup* jointModelGroup = moveGroup.getRobotModel()->getJointModelGroup("youbot_arm");
    std::vector<double> jointValues;

    /* get joint values for last position of trajectory */
    int iSize = myPlan.trajectory_.joint_trajectory.points.size();
    for(int i = 0; i < NUMBER_JOINTS; i++)
    {
        jointValues.push_back(myPlan.trajectory_.joint_trajectory.points[iSize-1].positions[i]);
    }

    /* set joints in computed end pose */
    kinematicState->setJointGroupPositions(jointModelGroup, jointValues);

    /* compute forward kinematic */
    const Eigen::Isometry3d& endEffectorState = kinematicState->getGlobalLinkTransform(moveGroup.getEndEffectorLink());
    geometry_msgs::Pose eefPose = tf2::toMsg(endEffectorState);

    /* check if position is in allowed tolerance*/
    if(abs(goalPose.position.x - eefPose.position.x) > dGoalStateTranslationPrecision)
    {
        bReturnValue = false;
    }
    if(bReturnValue == true)
    {
        if(abs(goalPose.position.y - eefPose.position.y) > dGoalStateTranslationPrecision)
        {
            bReturnValue = false;
        }
    }
    if(bReturnValue == true)
    {
        if(abs(goalPose.position.z - eefPose.position.z) > dGoalStateTranslationPrecision)
        {
            bReturnValue = false;
        }
    }

    /* check if orientation is in allowed tolerance*/
    if(abs(goalPose.orientation.x - eefPose.orientation.x) > dGoalRotationPrecision)
    {
        bReturnValue = false;
    }
    if(bReturnValue == true)
    {
        if(abs(goalPose.orientation.y - eefPose.orientation.y) > dGoalRotationPrecision)
        {
            bReturnValue = false;
        }
    }
    if(bReturnValue == true)
    {
        if(abs(goalPose.orientation.z - eefPose.orientation.z) > dGoalRotationPrecision)
        {
            bReturnValue = false;
        }
    }

    /*reset joints to home position */
    for(int i = 0; i < 5; i++)
    {
        jointValues[i] = 0;
    }
    kinematicState->setJointGroupPositions(jointModelGroup, jointValues);

    std::cout << "Difference in x direction: " << goalPose.position.x - eefPose.position.x << std::endl;
    std::cout << "Difference in y direction: " << goalPose.position.y - eefPose.position.y << std::endl;
    std::cout << "Difference in z direction: " << goalPose.position.z - eefPose.position.z << std::endl;

    std::cout << "Difference in x orientation: " << goalPose.orientation.x - eefPose.orientation.x << std::endl;
    std::cout << "Difference in y orientation: " << goalPose.orientation.y - eefPose.orientation.y << std::endl;
    std::cout << "Difference in z orientation: " << goalPose.orientation.z - eefPose.orientation.z << std::endl;
    ROS_INFO_STREAM("Goal state " << (bReturnValue ? "reached" : "not reached"));

    return bReturnValue;
}

/***
 * @fn bool Planner::checkPosePossible(moveit::planning_interface::MoveGroupInterface &moveGroup, geometry_msgs::PointStamped goalPose)
 * @brief checks if the goal pose is reachable because otherwise you don't need to plan a motion
 * @param moveGroup interface with information about the robot
 * @param goalPose goal pose that should be reached
 * @return true if pose is possible with some approximations
 */
bool Planner::poseReachable(moveit::planning_interface::MoveGroupInterface &moveGroup,
                                geometry_msgs::Pose &goalPose)
{
    /*allow approximate solutions because of kinematic constraints of youbot*/
    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true; // optional
    robot_state::RobotState robot_state_ik(moveGroup.getRobotModel());
    auto robotModel = moveGroup.getRobotModel();

    /* get all joints in one object*/
    auto jointModelGroup = robotModel->getJointModelGroup("youbot_arm");

    /* is the calculated pose reachable with given constraints for the youbot? */
    bool bPoseReachable = moveGroup.getCurrentState()->setFromIK(
            jointModelGroup,
            goalPose,
            1,moveit::core::GroupStateValidityCallbackFn(),
            opts
    );

    if(bSoftGripperUsed)
    {
        if((abs(goalPose.position.x - gripperObject.mesh_poses.at(0).position.x) < 0.04) && (abs(goalPose.position.y - gripperObject.mesh_poses.at(0).position.y) < 0.04))
        {
            bPoseReachable = false;
        }
    }


    return bPoseReachable;
}

/**
 * @fn bool Planner::pickup(moveit::planning_interface::MoveGroupInterface &moveGroup)
 * @brief moves to the object with the plan calculated in the planning function and picks the object
 * @param moveGroup moveit interface for the movegroup with all the relevant information about the robot
 * @return true on success
 */
bool Planner::pickup(moveit::planning_interface::MoveGroupInterface &moveGroup)
{
    /* first naive way to pick up object without moving the fingers of the gripper*/
    moveGroup.attachObject(gripperObject.id, moveGroup.getEndEffectorLink());
    //detachObjectFromPlate(moveGroup);

#ifdef USE_GAZEBO 
    gazeboPublisher.deleteObjectFromSimulation();
#endif /*USE_GAZEBO*/
}

/**
 * @fn bool Planner::placeObject(moveit::planning_interface::MoveGroupInterface &moveGroup, geometry_msgs::Point position)
 * @brief Drops the object picked up from the printer down to a given point.
 * @param moveGroup moveit interface for robot
 * @param position where to place the object
 * @return true on success false otherwise
 */
bool Planner::placeObject(moveit::planning_interface::MoveGroupInterface &moveGroup, geometry_msgs::Point position)
{
    bool bDropSuccess = false;
    /* create goal pose for place */
    geometry_msgs::PoseStamped objectPose;
    objectPose.pose.position = position;
    objectPose.header.frame_id = moveGroup.getPlanningFrame();
    objectPose.pose.orientation = gripperObject.mesh_poses.at(0).orientation;

    double dYStart = objectPose.pose.position.y;
    double dXStart = objectPose.pose.position.x;
    /* get goal pose */
    FrameTransformation frameTransformation(bSoftGripperUsed);
    frameTransformation.setObjectWorldPose(objectPose.pose);
    geometry_msgs::PoseStamped goalPose = frameTransformation.getPoseFromEigenGraspResult(choosedGrasp, *planningScene);
    moveGroup.setStartStateToCurrentState();
    int iTriesInEachDirection=2;
    moveGroup.setPlanningTime(7); /* a little longer planning time leads to higher precision in goal state*/
    for(int j = 0; j<=iTriesInEachDirection; j++) /* start with desired position */
    {
        if(bDropSuccess == true)
        {
            break; /* break out of these loops as we have a good result*/
        }
        for(int i = 0; i<=iTriesInEachDirection; i ++)
        {
            moveGroup.clearPoseTargets();
            objectPose.pose.position.y = dYStart + i * 0.001; /* try different y Positions to find a reachable position */
            objectPose.pose.position.x = dXStart + j * 0.001;

            double alpha = atan2(objectPose.pose.position.x, objectPose.pose.position.y);
            objectPose.pose.orientation = gripperObject.mesh_poses.at(0).orientation; /* reset orientation */
            frameTransformation.rotatePose(objectPose, 0, 0, M_PI - alpha);

            frameTransformation.setObjectWorldPose(objectPose.pose);
            goalPose = frameTransformation.getPoseFromEigenGraspResult(choosedGrasp, *planningScene);

            if(poseReachable(moveGroup, goalPose.pose)) /* pose reachable for IK? */
            {
                /* allow approximate solutions as youbot has kinematic constraints */
                moveGroup.setGoalJointTolerance(0.005);
                moveGroup.setApproximateJointValueTarget(goalPose, moveGroup.getEndEffectorLink());
                bDropSuccess = moveGroup.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            }
            else
            {
                std::cout << "not reachable pose to place object" << std::endl;
            }

            if(bDropSuccess)
            {
                bDropSuccess = checkComputedGoalPose(moveGroup, goalPose.pose, false); /* check if desired goal pose was really reached */
            }
            if (bDropSuccess)
            {
                bDropSuccess = move(moveGroup, false);
                if(bDropSuccess)
                {
                    moveGroup.detachObject(gripperObject.id);
                    break;
                }
            }
        }
    }
    if(bDropSuccess == false)
    {
        /* if we couldn't place the object somewhere else we shouldn't move it away from the printer plate */
        moveGroup.detachObject(gripperObject.id);
    }
    goIntoHomePosition(moveGroup, true);
    return bDropSuccess;
}

/**
 * @fn bool Planner::goIntoHomePosition(moveit::planning_interface::MoveGroupInterface &moveGroup, bool bLastMotion)
 * @brief Returns into home position
 * @param moveGroup moveit interface for the robot
 * @param bLastMotion true if this is the last motion the robot should perform
 * @return true on success
 */
bool Planner::goIntoHomePosition(moveit::planning_interface::MoveGroupInterface &moveGroup, bool bLastMotion)
{
    bool bReturn;
    if(moveGroup.setNamedTarget("home") == true)
    {
        int iCountPlannings = 0;
        while(iCountPlannings < 4)
        {
            bool bPlanningSuccess = moveGroup.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            iCountPlannings++;
            if(bPlanningSuccess)
            {
                move(moveGroup, bLastMotion);
                bReturn = true;
                break;
            }
        }
        
        if(iCountPlannings >= 4)
        {
            bReturn = false;
        }
    }
    else
    {
        std::cerr << "Couldn't set 'home' as target " << std::endl;
        bReturn = false;
    }
    return bReturn;
}

/**
 * @fn bool Planner::move(moveit::planning_interface::MoveGroupInterface &moveGroup, bool bLastMotion)
 * @brief moves the robot in rviz and if simulation is used also in the simulation
 * @param moveGroup interface with all robot specific information
 * @param bLastMotion true if it is the last motion of the robot. Only important when a real robot is used
 * @return true on successful and collision free motion
 */
bool Planner::move(moveit::planning_interface::MoveGroupInterface &moveGroup, bool bLastMotion)
{
    bool bReturn;
#ifdef USE_GAZEBO
    //gazeboPublisher.resumeGazeboSimulation();
    bReturn = gazeboPublisher.publishTrajectory(myPlan.trajectory_.joint_trajectory.points);
#else
    moveit::planning_interface::MoveItErrorCode errorCode = moveGroup.move();
    if(errorCode == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        bReturn = false;
    }
#endif /*USE_GAZEBO*/
#ifdef USE_GAZEBO
    //gazeboPublisher.pauseGazeboSimulation();
#endif /*USE_GAZEBO*/
    if(bReturn == true)
    {
#ifdef USE_MOVEIT_PUB
        moveitPublisher.addMotion(myPlan.trajectory_.joint_trajectory.points);
        moveitPublisher.publishMotion(bLastMotion);
#endif /*USE_MOVEIT_PUB*/
    }
    else
    {
        std::cerr << "Collision detected" << std::endl;
        bReturn = false;
    }
    return bReturn;
}

/**
 * @fn bool Planner::detachObjectFromPlate(moveit::planning_interface::MoveGroupInterface &moveGroup)
 * @brief As the printed objects will adhere at the surface of the plate we need to detach it from there
 * @param moveGroup moveit interface with all information about the robot
 * @return true on success, false otherwise
 */
bool Planner::detachObjectFromPlate(moveit::planning_interface::MoveGroupInterface &moveGroup)
{
    FrameTransformation frameTransformation(bSoftGripperUsed);
    for(int i = 1; i < 5; i ++)
    {
        geometry_msgs::PoseStamped tempPose;
        tempPose = pickUpPose;
        frameTransformation.rotatePose(tempPose, 0, 0, M_PI + pow(-1, i) * (6 * M_PI / 180));
        if(moveGroup.setApproximateJointValueTarget(tempPose))
        {
            bool bPlanningSuccess = moveGroup.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            if (bPlanningSuccess)
            {
                move(moveGroup, false);
            }
        }
    }
}

/**
 * @fn bool Planner::goIntoInitPosition(moveit::planning_interface::MoveGroupInterface &moveGroup)
 * @brief tries to get the robot into init position. init position is either only home or if the define was enabled
 * the init position is a position that hopes to avoid a motion plan that contains collisions
 * @param moveGroup interface with all information about the robot
 * @return true if init position was reached
 */
bool Planner::goIntoInitPosition(moveit::planning_interface::MoveGroupInterface &moveGroup)
{
    bool bReturnValue = false;
    ROS_INFO("Start to go into home position");
    gazeboPublisher.goIntoHomePosition();
    ROS_INFO("Home position reached");

#ifdef USE_HELP_POSE
    moveit_msgs::CollisionObject objectToGrip = Environment::getObjectToGrip();
    geometry_msgs::PoseStamped goalPose;
    goalPose.pose = objectToGrip.mesh_poses.at(0);

    /*z-axis of endeffector should be parallel to the ground */
    FrameTransformation frameTransformation(bSoftGripperUsed);
    frameTransformation.rotatePose(goalPose, M_PI_2, 0, 0);
    frameTransformation.rotatePose(goalPose, 0, 0, M_PI_2);

    goalPose.pose.position.z += 0.08;
    goalPose.pose.position.y += 0.20;

    if(moveGroup.setApproximateJointValueTarget(goalPose))
    {
        moveit::planning_interface::MoveItErrorCode errorCode = moveGroup.plan(myPlan);
        ROS_INFO("Finished planning for workaround position");
        if(errorCode == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            bReturnValue = move(moveGroup, false); /*try to move for the first time*/
            ROS_INFO("Finished moving into workaround position");
        }
        else
        {
            ROS_WARN("Not in workaround position, simulation might fail");
        }
    }
#else
    bReturnValue = true;
#endif /*USE_HELP_POSE*/
    return bReturnValue;
}
