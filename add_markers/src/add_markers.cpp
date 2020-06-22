#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"

float pickup[3] = {1.0, 1.0, 1.0};
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
      
      boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> posePointer;      
      posePointer = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", ros::Duration(10));
      
      marker.pose.position.x = posePointer->pose.pose.position.x;
      marker.pose.position.y = posePointer->pose.pose.position.y;
      
      carrying = false;
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
      ros::Duration(5).sleep();
    }
  }
  else
  {
    if (msg->data == "yes") {
      std::cout << "add_markers: reached pickup" << std::endl;
      
      carrying = true;
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ros::Duration(5).sleep();
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Respond to the turtlebot arriving at its goal
  ros::Subscriber sub = n.subscribe("arrived", 3, arrivedCallback);
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;


    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pickup[0];
    marker.pose.position.y = pickup[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    ros::spin();
}