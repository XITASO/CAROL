//
// Created by andreas on 06.11.20.
//
#include "environment.h"
#include <tf2_eigen/tf2_eigen.h>
#ifdef LOG_RESULTS
extern LogFile logFile;
#endif /*LOG_RESULTS*/

moveit_msgs::CollisionObject Environment::objectToGrip;
double Environment::positionPrinterX;
double Environment::positionPrinterY;
std::string Environment::sObjectToGripName;

char* Environment::pcRobotPosition;

/**
 * @fn void Environment::init(moveit::planning_interface::MoveGroupInterface &moveGroup, char ** argv)
 * @brief init environment with all objects to be gripped and obstacles
 * @param moveGroup moveit group interface with information about frame names
 * @param argv arguments with name of object to grip
 */
void Environment::init(moveit::planning_interface::MoveGroupInterface &moveGroup, char ** argv)
{
    pcRobotPosition = argv[3];
    moveit_msgs::CollisionObject printer;
    printer.header.frame_id = moveGroup.getPlanningFrame();

    printer.id = "printer";

    shapes::Mesh* m = shapes::createMeshFromResource("file:///home/andreas/Ender-3/printer.stl");
    shape_msgs::Mesh printerMesh;
    shapes::ShapeMsg printerMeshMsg;
    shapes::constructMsgFromShape(m,printerMeshMsg);
    printerMesh = boost::get<shape_msgs::Mesh>(printerMeshMsg);

    printer.meshes.resize(1);

    printer.meshes[0] = printerMesh;
    printer.mesh_poses.resize(1);
    if(std::strcmp(pcRobotPosition, "right") == 0)
    {
        positionPrinterX = 0.0;
        if(bSoftGripperUsed)
        {
            positionPrinterY = -0.42; /* to get better results the robot needs to be further away */
        }
        else
        {
            positionPrinterY = -0.35;
        }
        orientationPrinterZ = -0.707;
        orientationPrinterW = 0.707;
    }
    else if(std::strcmp(pcRobotPosition, "front") == 0)
    {
        positionPrinterX = 0.035;
        if(bSoftGripperUsed)
        {
            positionPrinterY = -0.45; /* to get better results the robot needs to be further away */
        }
        else
        {
            positionPrinterY = -0.37;
        }

    }
    else if(std::strcmp(pcRobotPosition, "left") == 0)
    {
        positionPrinterX = 0;
        if(bSoftGripperUsed)
        {
            positionPrinterY = -0.40; /* to get better results the robot needs to be further away */
        }
        else
        {
            positionPrinterY = -0.31;
        }

        orientationPrinterZ = 0.707;
        orientationPrinterW = 0.707;
    }
    else
    {
        std::cerr << "No position of robot defined" << std::endl;
    }
#ifdef LOG_RESULTS
    std::string string = ROBOT_POSITION;
    string.append(pcRobotPosition);
    logFile.write(string, true);
#endif /*LOG_RESULTS*/
    printer.mesh_poses[0].position.x = positionPrinterX;
    printer.mesh_poses[0].position.y = positionPrinterY;
    printer.mesh_poses[0].position.z = positionPrinterZ;
    printer.mesh_poses[0].orientation.w= orientationPrinterW;
    printer.mesh_poses[0].orientation.x= 0.0;
    printer.mesh_poses[0].orientation.y= 0.0;
    printer.mesh_poses[0].orientation.z= orientationPrinterZ;

    printer.meshes.push_back(printerMesh);
    printer.mesh_poses.push_back(printer.mesh_poses[0]);
    printer.operation = printer.ADD;
    
    collisionObjects.push_back(printer);
    
    objectToGrip.header.frame_id = moveGroup.getPlanningFrame();
    objectToGrip.id = "object";

    /* if we scale the shape by a factor of 0.001 (which means a conversion from mm to m) we only need one stl file:
     * the one for grasp planning component which needs in mm and moveit which needs the objects in m */
    Eigen::Vector3d scalingVector;
    scalingVector[0] = 0.001;
    scalingVector[1] = 0.001;
    scalingVector[2] = 0.001;
    shapes::Mesh* cubeM = shapes::createMeshFromResource("file://" + std::string(argv[1]), scalingVector);
    shape_msgs::Mesh cubeMesh;
    shapes::ShapeMsg cubeMeshMsg;
    shapes::constructMsgFromShape(cubeM,cubeMeshMsg);
    cubeMesh = boost::get<shape_msgs::Mesh>(cubeMeshMsg);
    objectToGrip.meshes.resize(1);
    objectToGrip.meshes[0] = cubeMesh;
    objectToGrip.mesh_poses.resize(1);

    objectToGrip.mesh_poses[0].position.x = positionPrinterX + positionObjectX;
    objectToGrip.mesh_poses[0].position.y = positionPrinterY + positionObjectY;
    objectToGrip.mesh_poses[0].position.z = positionPrinterZ + positionObjectZ;
    objectToGrip.mesh_poses[0].orientation.w= orientationPrinterW;
    objectToGrip.mesh_poses[0].orientation.x= 0.0;
    objectToGrip.mesh_poses[0].orientation.y= 0.0;
    objectToGrip.mesh_poses[0].orientation.z= orientationPrinterZ;

    objectToGrip.meshes.push_back(cubeMesh);
    objectToGrip.mesh_poses.push_back(objectToGrip.mesh_poses[0]);
    objectToGrip.operation = objectToGrip.ADD;
    Eigen::Isometry3d eigenBoxPose;
    tf2::fromMsg(objectToGrip.mesh_poses[0], eigenBoxPose);

    collisionObjects.push_back(objectToGrip);

    moveit_msgs::CollisionObject table;
    table.header.frame_id = moveGroup.getPlanningFrame();
    table.id = "table";
    shape_msgs::SolidPrimitive tableSolid;
    tableSolid.type = tableSolid.BOX;
    tableSolid.dimensions.resize(3);
    tableSolid.dimensions[0] = 1.62;
    tableSolid.dimensions[1] = 1.0;
    tableSolid.dimensions[2] = 0.01;
    geometry_msgs::Pose tablePose;
    tablePose.orientation.y = 1.5708;
    tablePose.position.x = 0;
    tablePose.position.y = 0;
    tablePose.position.z = -0.04 + 0.03182;
    table.primitives.push_back(tableSolid);
    table.primitive_poses.push_back(tablePose);
    table.operation = table.ADD;

    collisionObjects.push_back(table);

    objectToPlace.header.frame_id = moveGroup.getPlanningFrame();
    objectToPlace.id = "boxToPlaceObject";
    shapes::Mesh* tableM = shapes::createMeshFromResource("file://" + std::string(argv[7]));
    shape_msgs::Mesh tableMesh;
    shapes::ShapeMsg tableMeshMsg;
    shapes::constructMsgFromShape(tableM,tableMeshMsg);
    tableMesh = boost::get<shape_msgs::Mesh>(tableMeshMsg);
    objectToPlace.meshes.resize(1);
    objectToPlace.meshes[0] = tableMesh;
    objectToPlace.mesh_poses.resize(1);

    objectToPlace.mesh_poses[0].position.x = 0.5;
    objectToPlace.mesh_poses[0].position.y = 0.34;
    objectToPlace.mesh_poses[0].position.z = 0.13 - HEIGHT_YOUBOT_PLATE;
    objectToPlace.mesh_poses[0].orientation.w= 1;
    objectToPlace.mesh_poses[0].orientation.x= 0.0;
    objectToPlace.mesh_poses[0].orientation.y= 0.0;
    objectToPlace.mesh_poses[0].orientation.z= 0.0;

    objectToPlace.meshes.push_back(tableMesh);
    objectToPlace.mesh_poses.push_back(objectToPlace.mesh_poses[0]);
    objectToPlace.operation = objectToPlace.ADD;

    collisionObjects.push_back(objectToPlace);
    extractObjectNameFromFilePath(argv[1]);
}

/**
 * @fn std::vector<moveit_msgs::CollisionObject> Environment::getEnvironment()
 * @brief environment with all collision objects in this world
 * @return vector with all collision objects
 */
std::vector<moveit_msgs::CollisionObject> Environment::getEnvironment() 
{
    return collisionObjects;
}

/**
 * @fn moveit_msgs::CollisionObject Environment::getObjectToGrip()
 * @brief returns object that should be gripped as a moveit collision object
 * @return object to grip
 */
moveit_msgs::CollisionObject Environment::getObjectToGrip()
{
    return objectToGrip;
}

/**
 * @fn Environment::Environment()
 * @brief constructor where environment finds out whether softgripper is used or not
 */
Environment::Environment()
{
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
}

std::string Environment::getRobotPosition()
{
    std::string returnValue(pcRobotPosition);
    return returnValue;
}

void Environment::extractObjectNameFromFilePath(char * pcFilePath)
{
    char * pcTokens = strtok(pcFilePath, "/");
    std::string sWholeString(pcTokens);
    while(pcTokens)
    {
        pcTokens = strtok(NULL, "/");
        if(pcTokens != NULL)
        {
           sWholeString = pcTokens;
        }
    }

    char * pcSubstring = ".stl";
    std::string sSubString(pcSubstring);

    int iPosFound = sWholeString.find(sSubString);
    sObjectToGripName = sWholeString.substr(0, iPosFound);
}

std::string Environment::getObjectName() {
    return sObjectToGripName;
}

void Environment::getPrinterPosition(double &dX, double &dY)
{
    dX = positionPrinterX;
    dY = positionPrinterY;

}

