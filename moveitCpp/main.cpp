#include "main.h"
#ifdef LOG_RESULTS
LogFile logFile;
#endif /*LOG_RESULTS*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(1);
    ros::start();
    spinner.start();
#ifdef LOG_RESULTS
    logFile.open(argv[2]);
#endif /*LOG_RESULTS*/

#ifdef USE_GRASPIT_SUB
    GraspItSub graspItSub(nodeHandle);
#ifdef USE_PICK
    Converter converter(graspItSub);
#endif /*USE_PICK*/
#endif /*USE_GRASPIT_SUB*/

    moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
    bool bPlanningSuccess = true; /* true if success with planning a motion */
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;

#ifdef USE_GAZEBO_SUBS

#endif /*USE_GAZEBO_SUBS*/

#ifdef USE_GAZEBO_PUBS

#endif /*USE_GAZEBO_PUBS*/


#ifdef USE_ROS_SERVICE
    if(GazeboPublisher::registerAndStartControllersForGazebo() != ROS_OK)
    {
        ROS_WARN("Not all controllers for robot could be started");
    }
#endif /*USE_ROS_SERVICE*/

#ifdef USE_RVIZ
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visualTools("arm_joint_0");
    visualTools.deleteAllMarkers();

#ifdef LOG_RESULTS
    logFile.write(USED_OBJECT, std::string(argv[1]));
#endif /*LOG_RESULTS*/
    Environment environment;
    environment.init(moveGroup, argv);

    planning_scene::PlanningScene planningScene(moveGroup.getRobotModel());
    sleep(1); /* buffer until topic is ready to publish planning scenes */
    planningSceneInterface.addCollisionObjects(environment.getEnvironment());
#endif /*USE_RVIZ*/

#ifdef USE_PLAN
    moveit_msgs::CollisionObject gripperObject = Environment::getObjectToGrip();
    Planner planner(&graspItSub, &planningScene, gripperObject, nodeHandle);
    bPlanningSuccess = planner.plan(moveGroup);
    if(bPlanningSuccess)
    {
        planner.pickup(moveGroup);
        geometry_msgs::Point goalPosition;
        /* get position to place from sys.argv */
        goalPosition.x = std::stod(argv[4]);
        goalPosition.y = std::stod(argv[5]);
        goalPosition.z = std::stod(argv[6]);
        planner.placeObject(moveGroup, goalPosition);
    }
#ifdef LOG_RESULTS
    logFile.close();
#endif /*LOG_RESULTS*/
#elif defined(USE_PICK)
    Planner planner(&converter, &planningScene, gripperObject, nodeHandle);
    planner.pick(moveGroup);
#endif /*USE_PLAN*/

    return 0;
}
