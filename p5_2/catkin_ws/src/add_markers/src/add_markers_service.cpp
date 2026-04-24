#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>

// Goal coordinates — must match pick_objects.cpp
const double COLLECT_X  = -1.0, COLLECT_Y  = 2.5;
const double DELIVER_X  =  0.0, DELIVER_Y  = 0.0;
const double ARRIVAL_THRESHOLD = 0.55;  // metres

enum RobotState { AWAITING_PICKUP, ITEM_COLLECTED, AWAITING_DROPOFF, COMPLETE };
RobotState state = AWAITING_PICKUP;

ros::Publisher marker_pub;
ros::Time collect_time;

double euclideanDist(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2, dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

void publishMarker(double x, double y, uint32_t action) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = "map";
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "home_service";
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
  marker.color.r            = 0.2f;
  marker.color.g            = 0.8f;
  marker.color.b            = 0.2f;
  marker.color.a            = 1.0f;
  marker_pub.publish(marker);
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (state == COMPLETE) return;

  double rx = msg->pose.pose.position.x;
  double ry = msg->pose.pose.position.y;

  switch (state) {
    case AWAITING_PICKUP:
      if (euclideanDist(rx, ry, COLLECT_X, COLLECT_Y) < ARRIVAL_THRESHOLD) {
        ROS_INFO("Collection point reached, removing marker");
        publishMarker(COLLECT_X, COLLECT_Y, visualization_msgs::Marker::DELETE);
        state = ITEM_COLLECTED;
        collect_time = ros::Time::now();
      }
      break;

    case ITEM_COLLECTED:
      if ((ros::Time::now() - collect_time).toSec() >= 5.0) {
        ROS_INFO("Item collected, navigating to delivery point");
        state = AWAITING_DROPOFF;
      }
      break;

    case AWAITING_DROPOFF:
      if (euclideanDist(rx, ry, DELIVER_X, DELIVER_Y) < ARRIVAL_THRESHOLD) {
        ROS_INFO("Delivery point reached, placing marker");
        publishMarker(DELIVER_X, DELIVER_Y, visualization_msgs::Marker::ADD);
        state = COMPLETE;
      }
      break;

    default:
      break;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "add_markers_service");
  ros::NodeHandle n;

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 10, poseCallback);

  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) return 0;
    ROS_WARN_ONCE("Waiting for RViz to subscribe to /visualization_marker...");
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Placing marker at collection point (%.1f, %.1f)", COLLECT_X, COLLECT_Y);
  publishMarker(COLLECT_X, COLLECT_Y, visualization_msgs::Marker::ADD);

  ros::spin();
  return 0;
}
