#include "main.h"

int main(int argc, char** argv)
{
    /*init ros*/
    ros::init(argc, argv, "youbot_driver");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(1);
    ros::start();
    spinner.start();

    MoveItSubscriber moveItSubscriber(nodeHandle);
    Manipulator youbot(argv);

    youbot.readAllLimits();
    //youbot.testGrippers();
    while(moveItSubscriber.ready()==false)
    {
        sleep(5);
    }

    own_msg::youBotJoints nextMotion;
    while(moveItSubscriber.getNextYouBotMotion(nextMotion) == true)
    {
        youbot.executeMotion(nextMotion);
        SLEEP_SEC(2);
    }
    return 0;
}
