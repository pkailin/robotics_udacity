#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

void configureMarker(visualization_msgs::Marker& m) {
  m.header.frame_id    = "map";
  m.ns                 = "zone_indicator";
  m.id                 = 0;
  m.type               = visualization_msgs::Marker::CUBE;
  m.pose.orientation.w = 1.0;
  m.scale.x            = 0.3;
  m.scale.y            = 0.3;
  m.scale.z            = 0.3;
  m.color.r            = 0.0f;
  m.color.g            = 1.0f;
  m.color.b            = 0.0f;
  m.color.a            = 1.0f;
}

void publishMarker(ros::Publisher& pub, visualization_msgs::Marker& m,
                   int action, double x, double y, const std::string& tag) {
  m.action          = action;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = 0.0;
  m.header.stamp    = ros::Time::now();

  if (action == visualization_msgs::Marker::ADD)
    ROS_INFO("Placing indicator at %s (%.1f, %.1f)", tag.c_str(), x, y);
  else
    ROS_INFO("Removing indicator from %s", tag.c_str());

  pub.publish(m);
}

void countdownLog(int seconds, const std::string& msg) {
  for (int t = seconds; t > 0; t--) {
    ROS_INFO("%s in %d...", msg.c_str(), t);
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "zone_marker_display");
  ros::NodeHandle nh;

  ros::Publisher markerPub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Wait for a subscriber (e.g. RViz) to connect
  ROS_INFO("Waiting for a subscriber on /visualization_marker...");
  while (markerPub.getNumSubscribers() < 1) {
    if (!ros::ok()) return 0;
    ros::Duration(0.5).sleep();
  }

  visualization_msgs::Marker indicator;
  configureMarker(indicator);

  // Phase 1: Display marker at pickup location
  publishMarker(markerPub, indicator,
                visualization_msgs::Marker::ADD, -4.0, 5.0, "Pickup Zone");

  // Phase 2: Countdown before hiding
  countdownLog(5, "Concealing pickup marker");

  // Phase 3: Remove marker from pickup location
  publishMarker(markerPub, indicator,
                visualization_msgs::Marker::DELETE, -4.0, 5.0, "Pickup Zone");

  // Phase 4: Countdown before showing drop-off
  countdownLog(5, "Revealing drop-off marker");

  // Phase 5: Display marker at drop-off location
  publishMarker(markerPub, indicator,
                visualization_msgs::Marker::ADD, -4.0, 4.0, "Drop-off Zone");

  ros::spin();
  return 0;
}
