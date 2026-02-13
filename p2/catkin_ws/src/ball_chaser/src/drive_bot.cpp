#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS Publisher
ros::Publisher motor_command_publisher;

/*
 * Callback function that executes whenever the service is called
 */
bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
                          ball_chaser::DriveToTarget::Response &res)
{
    // Create Twist message
    geometry_msgs::Twist motor_command;

    // Set requested velocities
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish to /cmd_vel
    motor_command_publisher.publish(motor_command);

    // Set feedback message
    res.msg_feedback = "Velocities set - linear_x: " +
                       std::to_string(req.linear_x) +
                       " , angular_z: " +
                       std::to_string(req.angular_z);

    ROS_INFO("%s", res.msg_feedback.c_str());

    return true;
}

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "drive_bot");

    // Create NodeHandle
    ros::NodeHandle n;

    // Publisher to robot wheels
    motor_command_publisher =
        n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define the service
    ros::ServiceServer service =
        n.advertiseService("/ball_chaser/command_robot",
                           handle_drive_request);

    ROS_INFO("Ready to receive drive commands.");

    // Handle ROS communication events
    ros::spin();

    return 0;
}

