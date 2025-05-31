/*
    Author: Mohamed Newir
    Date: 3/05/2025
    File: belt_ctrl.cpp
    Description:
                    Main file for the conveyer belt control system using ROS and Arduino.
                    This file initializes the ROS node and the conveyer control system,
                    and runs the main loop to handle conveyer movement and ROS communication.
                    It includes the main function that sets up the ROS node,
                    subscribes to the current package status topic, and publishes the target direction.
                    It requests user input for the target direction and manages the conveyer control logic.
*/

/*
    used libraries:
    - iostream: For input and output operations.
    - string: For string manipulation.
    - ros/ros.h: The main ROS header file for ROS functionality.
    - std_msgs/String.h: For handling string messages in ROS.
    - std_msgs/Int8.h: For handling integer messages in ROS.
*/
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

/*
    Define the direction and package status enums
*/
enum class Direction
{
    FORWARD = 1, // Move the conveyer forward
    BACKWARD,    // Move the conveyer backward
    STOP         // Stop the conveyer
};

enum class PkgStatus
{
    NO_PKG_DETECTED = 2, // No package detected
    IN_PKG = 0,          // Package detected at the input level
    OUT_PKG = 1          // Package detected at the output level
};

/*
    Current position message callback function
    This function is called whenever a new message is received on the "/current_position" topic.
*/
std_msgs::String pkgStatus; // Global variable to hold the current position message
void cb(const std_msgs::String &pkgMsg)
{
    pkgStatus.data = pkgMsg.data;
}

/*
    Main function
    This is the entry point of the program.
*/
int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "belt_ctrl_node");
    ros::NodeHandle nh;

    // Initialize the publisher for belt_dir
    ros::Publisher pub = nh.advertise<std_msgs::Int8>("/belt_dir", 10);
    // Initialize the target direction variable
    std_msgs::Int8 targetDir;

    // Initialize the subscriber for package_status
    ros::Subscriber sub = nh.subscribe("/package_status", 10, cb);

    // Set the loop rate
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        /*
            Prompt the user to enter the target direction.
            The user can choose between forward and backword direction or to stop the conveyer belt.
            If the input is invalid, it will prompt again.
        */
        std::cout << "Please enter target direction:\n"
                  << "              1: Forward \n"
                  << "              2: Backward \n"
                  << "              3: Stop \n";
        std::cout << "Enter a number (1-3) to set the target direction: ";

        // Read user input
        std::cin >> targetDir.data;
        // Validate input
        if (targetDir.data < static_cast<int>(Direction::FORWARD) || targetDir.data > static_cast<int>(Direction::STOP))
        {
            std::cerr << "Invalid input. Please enter a number between 1 and 3.\n";
            continue; // Skip the rest of the loop and prompt again
        }

        // Publish the target level message
        pub.publish(targetDir);
        std::cout << "Target level set to: " << targetDir.data << "\n";

        // while (pkgStatus.data != "NO_PKG_DETECTED")
        // {
        //     // Check the current package status
        //     if (pkgStatus.data == "IN_PKG")
        //     {
        //         std::cout << "Current package status message received: Package detected at the input position.\n";
        //     }
        //     else if (pkgStatus.data == "OUT_PKG")
        //     {
        //         std::cout << "Current package status message received: Package detected at the output position.\n";
        //     }
        //     else if (pkgStatus.data == "NO_PKG_DETECTED")
        //     {
        //         std::cout << "Current package status message received: No package detected.\n";
        //     }
        //     else
        //     {
        //         std::cerr << "Unknown package status: " << pkgStatus.data << "\n";
        //     }
        // }

        // Sleep to maintain the loop rate
        loop_rate.sleep();
        // Process incoming messages
        ros::spinOnce();
    }

    return 0;
}
