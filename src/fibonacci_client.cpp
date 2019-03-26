#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <sstream>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "fibonacci_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> client("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  client.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  client.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());

    auto result = client.getResult();
  
    std::stringstream ss;
    for(auto value : result->sequence)
    {
      ss << value << ", ";
    }
    ROS_INFO("Result: %s", ss.str().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
  }

  //exit
  return 0;
}