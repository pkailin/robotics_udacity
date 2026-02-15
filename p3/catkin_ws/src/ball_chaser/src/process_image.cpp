#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the requested velocities
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    bool ball_found = false;
    int white_pixel_position = 0;
    int white_pixel_count = 0;

    // Loop through each pixel in the image
    for (int i = 0; i < img.height * img.step; i += 3)
    {
        // Check if the pixel is white (RGB values all equal to 255)
        if (img.data[i] == white_pixel && 
            img.data[i+1] == white_pixel && 
            img.data[i+2] == white_pixel)
        {
            ball_found = true;
            // Calculate the column position of this pixel
            int column = (i % img.step) / 3;
            white_pixel_position += column;
            white_pixel_count++;
        }
    }

    // If white pixels were found, determine the ball's position
    if (ball_found && white_pixel_count > 0)
    {
        // Calculate average column position
        int avg_position = white_pixel_position / white_pixel_count;
        int image_width = img.width;

        // Determine if ball is on left, center, or right
        if (avg_position < image_width / 3)
        {
            // Ball is on the left - turn left
            drive_robot(0.0, 0.5);
        }
        else if (avg_position > 2 * image_width / 3)
        {
            // Ball is on the right - turn right
            drive_robot(0.0, -0.5);
        }
        else
        {
            // Ball is in the center - move forward
            drive_robot(0.5, 0.0);
        }
    }
    else
    {
        // No white ball detected - stop the robot
        drive_robot(0.0, 0.0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
