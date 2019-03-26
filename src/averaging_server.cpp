#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/AveragingAction.h>

class AveragingAction
{
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::AveragingAction> server_;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  actionlib_tutorials::AveragingFeedback feedback_;
  actionlib_tutorials::AveragingResult result_;
  ros::Subscriber sub_;

public:
    
  AveragingAction(std::string name) : 
    server_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    server_.registerGoalCallback(boost::bind(&AveragingAction::goalCB, this));
    server_.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
    server_.start();
  }

  ~AveragingAction(void)
  {
  }

  void goalCB()
  {
    // reset helper variables
    data_count_ = 0;
    sum_ = 0;
    sum_sq_ = 0;
    // accept the new goal
    goal_ = server_.acceptNewGoal()->samples;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    server_.setPreempted();
  }

  void analysisCB(const std_msgs::Float32::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!server_.isActive())
      return;
    
    data_count_++;
    feedback_.sample = data_count_;
    feedback_.data = msg->data;
    //compute the std_dev and mean of the data 
    sum_ += msg->data;
    feedback_.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
    server_.publishFeedback(feedback_);

    if(data_count_ > goal_) 
    {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < 5.0)
      {
        ROS_INFO("%s: Aborted (%f)", action_name_.c_str(), result_.mean);
        //set the action state to aborted
        server_.setAborted(result_);
      }
      else 
      {
        ROS_INFO("%s: Succeeded (%f)", action_name_.c_str(), result_.mean);
        // set the action state to succeeded
        server_.setSucceeded(result_);
      }
    } 
  }    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "averaging");

  AveragingAction averaging(ros::this_node::getName());
  ros::spin();

  return 0;
}