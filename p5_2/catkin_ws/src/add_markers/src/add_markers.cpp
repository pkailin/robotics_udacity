#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) return 0;
    ROS_WARN_ONCE("No subscribers detected on /visualization_marker, retrying...");
    ros::Duration(0.5).sleep();
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id    = "map";
  marker.ns                 = "home_service";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::CUBE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.3;
  marker.scale.y            = 0.3;
  marker.scale.z            = 0.3;
  marker.color.r            = 0.2f;
  marker.color.g            = 0.8f;
  marker.color.b            = 0.2f;
  marker.color.a            = 1.0f;

  // Phase 1: Display marker at collection point
  marker.action          = visualization_msgs::Marker::ADD;
  marker.pose.position.x = -1.0;
  marker.pose.position.y =  2.5;
  marker.pose.position.z =  0.0;
  marker.header.stamp    = ros::Time::now();
  ROS_INFO("Marker placed at collection point (-1.0, 2.5)");
  marker_pub.publish(marker);

  // Phase 2: Hold for 5 seconds before removal
  for (int i = 5; i > 0; i--) {
    ROS_INFO("Removing marker in %ds...", i);
    ros::Duration(1.0).sleep();
  }

  // Phase 3: Remove marker to simulate item picked up
  marker.action       = visualization_msgs::Marker::DELETE;
  marker.header.stamp = ros::Time::now();
  marker_pub.publish(marker);
  ROS_INFO("Collection point marker removed");

  // Phase 4: Countdown before placing delivery marker
  for (int i = 5; i > 0; i--) {
    ROS_INFO("Placing delivery marker in %ds...", i);
    ros::Duration(1.0).sleep();
  }

  // Phase 5: Display marker at delivery point
  marker.action          = visualization_msgs::Marker::ADD;
  marker.pose.position.x =  0.0;
  marker.pose.position.y =  0.0;
  marker.pose.position.z =  0.0;
  marker.header.stamp    = ros::Time::now();
  ROS_INFO("Marker placed at delivery point (0.0, 0.0)");
  marker_pub.publish(marker);

  ros::spin();
  return 0;
}
