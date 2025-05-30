/*
    Author: Mohamed Newir
    Date: 3/05/2025
    File: main.cpp
    Description:
                    Main file for the conveyer control system using ROS and Arduino.
                    This file initializes the ROS node and the conveyer control system,
                    and runs the main loop to handle conveyer movement and ROS communication.
                    It includes the main function that sets up the ROS node,
                    subscribes to the current position topic, and publishes the target level.
                    It requests user input for the target level and manages the conveyer control logic.
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
    define the current level variable
*/
enum class Level
{
    LEVEL1 = 1, // Represents Level 1
    LEVEL2,
    LEVEL3,
    NO_LEVEL // Represents no level detected
};

/*
    Current position message callback function
    This function is called whenever a new message is received on the "/current_position" topic.
*/
std_msgs::String currentPosition; // Global variable to hold the current position message
void cb(const std_msgs::String &currentPositionMsg)
{
    currentPosition.data = currentPositionMsg.data;
}

/*
    Main function
    This is the entry point of the program.
*/
int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "conveyer_ctrl_node");
    ros::NodeHandle nh;

    // Initialize the publisher for target_level
    ros::Publisher pub = nh.advertise<std_msgs::Int8>("/target_level", 10);
    // Initialize the target level variable
    std_msgs::Int8 targetLevel;

    // Initialize the subscriber for current_position
    ros::Subscriber sub = nh.subscribe("/current_position", 10, cb);

    // Set the loop rate
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        /*
            Prompt the user to enter the target level.
            The user can choose between levels 1 to 3 or stop the conveyer by entering 4.
            If the input is invalid, it will prompt again.
        */
        std::cout << "Please enter target level:\n"
                  << "1: Level 1\n"
                  << "2: Level 2\n"
                  << "3: Level 3\n"
                  << "4: No Level (Stop)\n";
        std::cin >> targetLevel.data;
        // Validate input
        if (targetLevel.data < static_cast<int>(Level::LEVEL1) || targetLevel.data > static_cast<int>(Level::NO_LEVEL))
        {
            std::cerr << "Invalid input. Please enter a number between 1 and 4.\n";
            continue; // Skip the rest of the loop and prompt again
        }

        // Publish the target level message
        pub.publish(targetLevel);
        std::cout << "Target level set to: " << targetLevel.data << "\n";

        // Wait for the current position message to be received
        while (currentPosition.data != "Stopped")
        {
            // Check if the current position message has been received
            if (!currentPosition.data.empty())
            {
                std::cout << "Waiting for current position state message...\n";
                std::cout << "Current state position: " << currentPosition.data << "\n";
            }
            else
            {
                std::cerr << "No current position state message received yet.\n";
            }

            // Sleep to avoid busy waiting
            ros::Duration(0.5).sleep();
        }

        // Print the current position
        std::cout << "Level " << targetLevel << " reached\n";

        // Sleep to maintain the loop rate
        loop_rate.sleep();
        // Process incoming messages
        ros::spinOnce();
    }

    return 0;
}
