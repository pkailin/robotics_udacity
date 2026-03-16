#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>

// Must match pick_objects.cpp goals
const double PICKUP_X  = -4.0, PICKUP_Y  = 5.0;
const double DROPOFF_X = -4.0, DROPOFF_Y = 4.0;
const double THRESHOLD = 0.5 * 1.1;  // 10% buffer

enum State { PICKUP_SHOW, PICKED_UP, DROPOFF_SHOW, DONE };
State state = PICKUP_SHOW;

ros::Publisher marker_pub;
ros::Time pickup_time;

double distance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void publishMarker(double x, double y, uint32_t action) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = "map";
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "add_markers";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::CUBE;
  marker.action             = action;
  marker.pose.position.x    = x;
  marker.pose.position.y    = y;
  marker.pose.position.z    = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.3;
  marker.scale.y            = 0.3;
  marker.scale.z            = 0.3;
  marker.color.r            = 0.0f;
  marker.color.g            = 1.0f;
  marker.color.b            = 0.0f;
  marker.color.a            = 1.0f;
  marker_pub.publish(marker);
}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (state == DONE) return;

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  if (state == PICKUP_SHOW) {
    if (distance(x, y, PICKUP_X, PICKUP_Y) < THRESHOLD) {
      ROS_INFO("Robot reached pickup zone. Hiding marker...");
      publishMarker(PICKUP_X, PICKUP_Y, visualization_msgs::Marker::DELETE);
      state = PICKED_UP;
      pickup_time = ros::Time::now();
    }

  } else if (state == PICKED_UP) {
    if ((ros::Time::now() - pickup_time).toSec() >= 5.0) {
      ROS_INFO("Pickup complete. Waiting for robot at drop off zone...");
      state = DROPOFF_SHOW;
    }

  } else if (state == DROPOFF_SHOW) {
    if (distance(x, y, DROPOFF_X, DROPOFF_Y) < THRESHOLD) {
      ROS_INFO("Robot reached drop off zone. Showing marker.");
      publishMarker(DROPOFF_X, DROPOFF_Y, visualization_msgs::Marker::ADD);
      state = DONE;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "add_markers_service");
  ros::NodeHandle n;

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber amcl_sub = n.subscribe("/amcl_pose", 10, amclCallback);

  // Wait for rviz to subscribe
  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) return 0;
    ROS_WARN_ONCE("Waiting for rviz to subscribe to /visualization_marker...");
    ros::Duration(0.5).sleep();
  }

  // Initially show marker at pickup zone
  ROS_INFO("Showing marker at pickup zone (%.1f, %.1f)", PICKUP_X, PICKUP_Y);
  publishMarker(PICKUP_X, PICKUP_Y, visualization_msgs::Marker::ADD);

  ros::spin();
  return 0;
}
