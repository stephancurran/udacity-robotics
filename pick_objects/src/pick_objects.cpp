#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <sstream>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  ros::NodeHandle n;
  ros::Publisher arrived_pub = n.advertise<std_msgs::String>("arrived", 1);std_msgs::String msg;
  
  float xs[2] = {1.0, -1.0};
  float ys[2] = {1.0, -1.0};
  float os[2] = {1.0, 1.0};

  for (int index = 0; index < sizeof(xs) / sizeof(xs[0]); index++)
  {
  
    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = xs[index];
    goal.target_pose.pose.position.y = ys[index];
    goal.target_pose.pose.orientation.w = os[index];

     // Send the goal position and orientation for the robot to reach
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      std::cout << "pick_objects: reached destination" << std::endl;
      
      std::stringstream ss;
      ss << "yes";
      msg.data = ss.str();
      arrived_pub.publish(msg);
    }
    else
    {
      std::cout << "pick_objects: failed to reach destination" << std::endl;
    }
    
    // Wait for 5 seconds at pickup point
    ros::Duration(5).sleep();
  }
  
  //std::stringstream ss;
  //ss << "no";
  //msg.data = ss.str();
  //arrived_pub.publish(msg);
  
  return 0;
}