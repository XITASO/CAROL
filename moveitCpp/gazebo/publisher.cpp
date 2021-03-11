//
// Created by andreas on 13.10.20.
//

#include "publisher.h"
#ifdef USE_GAZEBO_PUBS

ros::Publisher gazeboPubs[NUMBER_JOINTS + NUMBER_FINGER];
std::vector<std::string> GazeboPublisher::controllerNames;

GazeboPublisher::GazeboPublisher(ros::NodeHandle &nodeHandle)
{
    /*arm joints */
    adHomePositionJointValues[0] = 0.0101;
    adHomePositionJointValues[1] = 0.0101;
    adHomePositionJointValues[2] = -0.0158;
    adHomePositionJointValues[3] = 0.0222;
    adHomePositionJointValues[4] = 0.111;
    this->nodeHandle = nodeHandle;
    initGazeboPublishers(nodeHandle);
    resetGazeboSimulation();
}

/**
 * @fn bool initGazeboPublishers(ros::NodeHandle &nodeHandle)
 * @param nodeHandle: ros node that should publish the certain topics
 * @return bool: true if initialization worked
 */
bool GazeboPublisher::initGazeboPublishers(ros::NodeHandle &nodeHandle)
{
    for(int i = 1; i <= NUMBER_JOINTS; i++)
    {
        char acNameTopic[MAX_LENGTH_STRING] =  {0};
        sprintf(acNameTopic, "youbot/joint%d_position_controller/command", i);
        /* create publishers for gazebo with given topic */
        gazeboPubs[i-1] = nodeHandle.advertise<std_msgs::Float64>(acNameTopic, ROS_QUEUE_SIZE, true);
        sleep(1); /* buffer so that topics can be fully created */
    }

    /*gazeboPubs[NUMBER_JOINTS] = nodeHandle.advertise<std_msgs::Float64>("youbot/gripper_r_position_controller/command", ROS_QUEUE_SIZE, true);
    gazeboPubs[NUMBER_JOINTS + 1] = nodeHandle.advertise<std_msgs::Float64>("youbot/gripper_l_position_controller/command", ROS_QUEUE_SIZE, true);
    sleep(1);*/
    /* go into home position */
    //goIntoHomePosition();
    //sleep(2);
    return true;
}

/**
 * @fn void gazeboPublishValueForJoint(std_msgs::Float64 f64Msg, int iJointNumber)
 * @brief publishes a value for a joint of the robot
 * @param f64Msg: message object with data to publish
 * @param iJointNumber: number of joint to get the published value
 * @return void
 */
void GazeboPublisher::gazeboPublishValueForJoint(std_msgs::Float64 f64Msg, int iJointNumber)
{
    gazeboPubs[iJointNumber].publish(f64Msg);
}

void GazeboPublisher::goIntoHomePosition()
{
    std_msgs::Float64 f64Value;
    trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint;
    std::vector<trajectory_msgs::JointTrajectoryPoint> jointTrajectoryVector;
    jointTrajectoryPoint.positions.resize(NUMBER_JOINTS);
    for(int i = 0; i < NUMBER_JOINTS; i ++)
    {
        f64Value.data = adHomePositionJointValues[i];
        jointTrajectoryPoint.positions.at(i) = adHomePositionJointValues[i];
        gazeboPublishValueForJoint(f64Value, i);
    }
    jointTrajectoryVector.push_back(jointTrajectoryPoint);
    addSuccessfulTrajectory(jointTrajectoryVector);
    gazeboSubscriber.resetCollisionDetection();
    //resetGripperJoints();
}

bool GazeboPublisher::publishTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory)
{
    bool bSuccessfulTrajectory = true;
    for(int i = 0; i<trajectory.size(); i++)
    {
        for(int j = 0; j < trajectory.at(i).positions.size(); j++) {
            std_msgs::Float64 f64Value;
            f64Value.data = trajectory.at(i).positions.at(j);
            gazeboPublishValueForJoint(f64Value, j);
        }
        usleep(250 * 1000);
        if(gazeboSubscriber.getCollision())
        {
            bSuccessfulTrajectory = false;
            gazeboSubscriber.resetCollisionDetection();
            publishTrajectory(successfulTrajectory); /* get robot back in position before collision was detected */
            //resetGripperJoints();
            gazeboSubscriber.resetCollisionDetection();
            break;
        }
    }
    if(bSuccessfulTrajectory == true)
    {
        addSuccessfulTrajectory(trajectory);
    }
    return bSuccessfulTrajectory;
}

void GazeboPublisher::addSuccessfulTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory)
{
    successfulTrajectory.reserve(successfulTrajectory.size()
                                 + distance(trajectory.begin(),
                                            trajectory.end()));
    successfulTrajectory.assign(trajectory.begin(), trajectory.end());
}

/**
 * @fn rosService_t registerAndStartControllersForGazebo()
 * @return
 */
rosService_t GazeboPublisher::registerAndStartControllersForGazebo()
{
    rosService_t tReturnValue = ROS_OK;
    controller_manager_msgs::LoadControllerRequest loadControllerRequest;
    controller_manager_msgs::LoadControllerResponse loadControllerResponse;
    controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
    controller_manager_msgs::SwitchControllerResponse switchControllerResponse;

    loadControllerRequest.name = "youbot/joint_state_controller";
    controllerNames.push_back("youbot/joint_state_controller");
    switchControllerRequest.start_controllers.push_back(loadControllerRequest.name);
    ros::service::call("/controller_manager/load_controller", loadControllerRequest, loadControllerResponse);
    if(loadControllerResponse.ok == 0)
    {
        tReturnValue = ROS_ALREADY_INITIALIZED;
    }

    /* load controllers for gripper */
    /*loadControllerRequest.name = "youbot/gripper_r_position_controller";
    controllerNames.push_back("youbot/gripper_r_position_controller");
    switchControllerRequest.start_controllers.push_back(loadControllerRequest.name);
    ros::service::call("/controller_manager/load_controller", loadControllerRequest, loadControllerResponse);

    loadControllerRequest.name = "youbot/gripper_l_position_controller";
    controllerNames.push_back("youbot/gripper_l_position_controller");
    switchControllerRequest.start_controllers.push_back(loadControllerRequest.name);
    ros::service::call("/controller_manager/load_controller", loadControllerRequest, loadControllerResponse);*/

    /* load all controllers for the single joints */
    if(tReturnValue == ROS_OK)
    {
        for(int i = 1; i <= NUMBER_JOINTS; i++)
        {
            char acControllerName[MAX_LENGTH_STRING] = {0};
            sprintf(acControllerName, "youbot/joint%d_position_controller", i);
            loadControllerRequest.name = acControllerName;
            controllerNames.push_back(acControllerName);
            switchControllerRequest.start_controllers.push_back(acControllerName);
            ros::service::call("/controller_manager/load_controller", loadControllerRequest, loadControllerResponse);
            if(loadControllerResponse.ok == 0)
            {
                tReturnValue = ROS_ALREADY_INITIALIZED;
                break; /* something went wrong exit function */
            }
        }
    }

    /* start all controllers for the single joints */
    if(tReturnValue == ROS_OK)
    {
        switchControllerRequest.strictness = 2; /* default value according to a tutorial */
        ros::service::call("controller_manager/switch_controller", switchControllerRequest, switchControllerResponse);
        if(switchControllerResponse.ok == false)
        {
            tReturnValue = ROS_ALREADY_STARTED;
        }
    }
    return tReturnValue;
}

void GazeboPublisher::callEmptyGazeboService(const std::string& sMessage)
{
    std_srvs::EmptyRequest myRequest;
    std_srvs::EmptyResponse myResponse;
    ros::service::call("gazebo" + sMessage, myRequest, myResponse);
    sleep(2);
}

rosService_t GazeboPublisher::resetGazeboSimulation()
{
    callEmptyGazeboService("/reset_world");
    publishTrajectory(successfulTrajectory); /* get robot back in position before collision was detected */
    return ROS_OK;
}

rosService_t GazeboPublisher::pauseGazeboSimulation()
{
    callEmptyGazeboService("/pause_physics");
    return ROS_OK;
}

rosService_t GazeboPublisher::resumeGazeboSimulation() {
    callEmptyGazeboService("/unpause_physics");
    return ROS_OK;
}

void GazeboPublisher::addObjectIntoSimulation()
{
    double dPositionPrinterX;
    double dPositionPrinterY;
    Environment::getPrinterPosition(dPositionPrinterX, dPositionPrinterY);
    std::string sPositionRobot = Environment::getRobotPosition();
    double dRoll, dPitch, dYaw = 0;
    if (sPositionRobot.compare("front") == 0)
    {
        dYaw = 0;
    }
    else if(sPositionRobot.compare("right") == 0)
    {
        dYaw = -M_PI_2;
    }
    else if(sPositionRobot.compare("left") == 0)
    {
        dYaw = M_PI_2;
    }
    sObjectName = Environment::getObjectName();
    char acCommand[256] = {0};
    snprintf(acCommand, 256, "rosrun gazebo_ros spawn_model -sdf "
                             "-file ~/.gazebo/models/%s/model.sdf -model %s -x %f -y %f -z %f -Y %f", sObjectName.c_str(),
                             sObjectName.c_str(), dPositionPrinterX, dPositionPrinterY, 0.11, dYaw);
    system(acCommand);
}

void GazeboPublisher::deleteObjectFromSimulation()
{
    gazebo_msgs::DeleteModelRequest deleteModelRequest;
    gazebo_msgs::DeleteModelResponse deleteModelResponse;

    deleteModelRequest.model_name = sObjectName;
    ros::service::call("gazebo/delete_model", deleteModelRequest, deleteModelResponse);
    if(deleteModelResponse.success == false)
    {
        ROS_WARN("Object was not deleted from simulation");
    }
}

GazeboPublisher::~GazeboPublisher()
{
    deleteObjectFromSimulation();
    goIntoHomePosition();
    unloadControllers();
}

void GazeboPublisher::unloadControllers()
{
    controller_manager_msgs::UnloadControllerRequest unloadControllerRequest;
    controller_manager_msgs::UnloadControllerResponse unloadControllerResponse;
    controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
    controller_manager_msgs::SwitchControllerResponse switchControllerResponse;

    switchControllerRequest.stop_controllers = controllerNames;
    switchControllerRequest.strictness = 2; /* default value according to a tutorial */
    ros::service::call("controller_manager/switch_controller", switchControllerRequest, switchControllerResponse);
    if (switchControllerResponse.ok)
    {
        for(int i = 0; i < controllerNames.size(); i++)
        {
            unloadControllerRequest.name = controllerNames[i];
            ros::service::call("/controller_manager/unload_controller", unloadControllerRequest, unloadControllerResponse);
        }
    }
}

void GazeboPublisher::resetGripperJoints()
{
    std_msgs::Float64 float64;
    float64.data = 0;
    for(int i = NUMBER_JOINTS; i <= NUMBER_JOINTS + 1; i++)
    {
        gazeboPublishValueForJoint(float64, i);
    }
}

#endif /*USE_GAZEBO_PUBS*/
