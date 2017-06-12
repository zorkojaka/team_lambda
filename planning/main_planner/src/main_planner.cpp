#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <maze_navigator/NavigateAction.h>

//#include <maze_navigator/action_const.h>

/*
    Tasks of this node:
        -interaction with navigation and speech packages
        -implements high level plan of the robot
*/
namespace main_planner
{

class MainPlanner
{
  protected:
  //  actionlib::SimpleActionClient<maze_navigator::NavigateAction> ac_;

  public:
  /*  MainPlanner() : ac_("navigation_server", true)
    {
        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started, sending goal.");
    }
*/
    void update()
    {

    }

  private:

};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_planner");
    main_planner::MainPlanner planner;

    // send a goal to the action
    /*actionlib_tutorials::FibonacciGoal goal;
    goal.order = 20;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
    */
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        planner.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}