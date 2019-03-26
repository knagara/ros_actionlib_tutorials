#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/AveragingAction.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_averaging");

  // create the action client
  actionlib::SimpleActionClient<actionlib_tutorials::AveragingAction> client("averaging");
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::AveragingGoal goal;
  goal.samples = 100;
  client.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    auto result = client.getResult();
    ROS_INFO("value: %f", result->mean);
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}