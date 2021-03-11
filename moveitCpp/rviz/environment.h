//
// Created by andreas on 06.11.20.
//

#ifndef MOVEIT_ENVIRONMENT_H
#define MOVEIT_ENVIRONMENT_H

#include "../config.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include "../logging/LogFile.h"
#include "../../definitions.h"

class Environment
{
public:
    Environment();
    void init(moveit::planning_interface::MoveGroupInterface &moveGroup, char ** argv);
    std::vector<moveit_msgs::CollisionObject> getEnvironment();
    static moveit_msgs::CollisionObject getObjectToGrip();
    static std::string getRobotPosition();
    static std::string getObjectName();
    static void getPrinterPosition(double & dX, double &dY);
private:
    std::vector<moveit_msgs::CollisionObject> collisionObjects; /*!< vector with all collision objects in the environment */
    static moveit_msgs::CollisionObject objectToGrip; /*!< object which should be gripped by robot arm */
    moveit_msgs::CollisionObject objectToPlace; /*!< where to place the object */

    const double HEIGHT_YOUBOT_PLATE = 0.036;

    /* position of printer in the world
     * frame of the printer should be in the center of the plate */
    static double positionPrinterX; /*!< x position of the printer */
    static double positionPrinterY; /*!< y position of the printer */
    double positionPrinterZ = 0.105 - HEIGHT_YOUBOT_PLATE; /*!< z position of the printer */
    double orientationPrinterZ = 0; /*!< z orientation of the printer */
    double orientationPrinterW = 1.0; /*!< z orientation of the printer */
    /* object to grip is exactly in the middle of the plate from the printer
     * if not you should change the following values */
    double positionObjectX = 0; /*!< x position of the object relating to the frame of the printer which should be exactly in the middle of the printing plate*/
    double positionObjectY = 0;/*!< y position of the object relating to the frame of the printer which should be exactly in the middle of the printing plate*/
    double positionObjectZ = 0.002;/*!< z position of the object relating to the frame of the printer which should be exactly in the middle of the printing plate*/

    static char* pcRobotPosition; /*!< robot position related to the printer */
    bool bSoftGripperUsed = false; /*!< information whether softgripper of the robot is used*/
    static std::string sObjectToGripName; /*!< name of the object that should be gripped by the robot */

    void extractObjectNameFromFilePath(char *);
};


#endif //MOVEIT_ENVIRONMENT_H
