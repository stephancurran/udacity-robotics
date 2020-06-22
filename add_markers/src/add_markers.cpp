#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include <string.h>

float pickup[2] = {1.0, 1.0};
bool carrying = false;
visualization_msgs::Marker marker;
ros::Publisher marker_pub;

/**
 * Callback for subscriber
 */
void arrivedCallback(const std_msgs::String::ConstPtr& msg)
{ 
  if (carrying) {
    if (msg->data == "yes") {
      std::cout << "add_markers: reached dropoff" << std::endl;
      std::cout << "add_markers: pausing for drop" << std::endl;
      
      // Get the robot position and drop marker there
      boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> posePointer;      
      posePointer = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", ros::Duration(10));
      
      marker.pose.position.x = posePointer->pose.pose.position.x;
      marker.pose.position.y = posePointer->pose.pose.position.y;
      
      ros::Duration(5).sleep();
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
      carrying = false;
      std::cout << "add_markers: item dropped" << std::endl;
    }
  }
  else
  {
    if (msg->data == "yes") {
      std::cout << "add_markers: reached pickup" << std::endl;
      std::cout << "add_markers: pausing for pickup" << std::endl;
      
      ros::Duration(5).sleep();
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      carrying = true;
      std::cout << "add_markers: item picked up" << std::endl;
    }
  }
}

void markerDemo() {
  /*
   * Leave the marker in place for 5 seconds, remove it, wait 5 seconds and place at dropoff point
   */
  float dropoff[2] = {-2.0, -1.0};
  
  std::cout << "add_markers demo: item at pickup point" << std::endl;
  
  ros::Duration(5).sleep();
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  std::cout << "add_markers demo: item picked up" << std::endl;
  
  ros::Duration(5).sleep();
  marker.pose.position.x = dropoff[0];
  marker.pose.position.y = dropoff[1];
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  std::cout << "add_markers demo: item dropped off" << std::endl;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  
  // Publisher for the marker
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the position of the marker. (other default values are fine)
  marker.pose.position.x = pickup[0];
  marker.pose.position.y = pickup[1];

  // Set the size of the marker
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Wait for a subscriber and publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);

  // If this is a test (called from add_markers.sh and add_markers_test.launch)
  if (strcmp(argv[1], "test") == 0) {
    markerDemo();
  } else {
    // Listen for the robot to arrive at pickup/dropoff
    ros::Subscriber sub = n.subscribe("arrived", 1, arrivedCallback);
    ros::spin();
  }
}