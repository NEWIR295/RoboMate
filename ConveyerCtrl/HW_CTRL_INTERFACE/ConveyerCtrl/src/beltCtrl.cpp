/*
    Author: Mohamed Newir
    Date: 3/05/2025
    File: beltCtrl.cpp
    Description:
                Implementation file for the belt control system using ROS and Arduino.
                This file contains the definitions of functions for initializing the belt control system,
                moving the belt, checking the in/out packages using IR sensors, and handling ROS communication.
*/

#include "../include/beltCtrl.hpp"

// checkIN_OUT_pkg function prototype
int checkIN_OUT_pkg(void);

// Initialize the stepper motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define the direction and package status enums
enum direction
{
    FORWARD = 1,   // Move the belt forward
    BACKWARD = -1, // Move the belt backward
    STOP = 0       // Stop the belt
};

enum pkgStatus
{
    NO_PKG_DETECTED = 2, // No package detected
    IN_PKG = 0,          // Package detected at the input level
    OUT_PKG = 1          // Package detected at the output level
};

/* global var to hold package status */
// std_msgs::String pkgMsg;

/* Publisher for package status message */
// ros::Publisher pkg_pub("/package_status", &pkgMsg);

std_msgs::Int8 targetDir; // Global variable to hold the target direction
/* subscriber cb for target level */
void callback(const std_msgs::Int8 &dirMsg)
{
    targetDir = dirMsg;
}

/* sub to "/belt_dir" topic for desired direction either forward or reverse */
ros::Subscriber<std_msgs::Int8> belt_sub("/belt_dir", &callback);

void beltCtrlInit(void)
{
    // Initialize the publisher for package status
    // nh.advertise(pkg_pub);

    // Initialize the subscriber for target level
    nh.subscribe(belt_sub);

    // Initialize the stepper motor
    stepper.setEnablePin(ENABLE_PIN);            // Set enable pin for the stepper motor
    stepper.enableOutputs();                     // Enable the outputs for the stepper motor
    stepper.setMaxSpeed(MOTOR_MAX_SPEED);        // Set maximum speed
    stepper.setAcceleration(MOTOR_ACCELERATION); // Set acceleration

    // Initialize the IR sensor pins
    pinMode(IR_IN_PIN, INPUT);
    pinMode(IR_OUT_PIN, INPUT);
}

void beltMove(void)
{
    if (targetDir.data == FORWARD || targetDir.data == BACKWARD)
    {
        // If the target direction is set to FORWARD or BACKWARD, move the belt
        if (targetDir.data == FORWARD)
        {
            stepper.setSpeed(MOTOR_SPEED); // Set speed for forward movement
        }
        else if (targetDir.data == BACKWARD)
        {
            stepper.setSpeed(-MOTOR_SPEED); // Set speed for backward movement
        }
        stepper.runSpeed(); // Run the stepper motor at the set speed
    }
    else
    {
        // If the target direction is STOP, stop the belt
        stepper.stop(); // Stop the stepper motor
    }

    /* 
        still not sure how the pkg system will work :") 
    */
    
    // // Check for packages at the input and output sensors
    // if (checkIN_OUT_pkg() != NO_PKG_DETECTED)
    // {
    //     int pkgStatus = checkIN_OUT_pkg(); // Get the package status
    //     if (pkgStatus == IN_PKG)
    //     {
    //         pkgMsg.data = "Package Detected at Input"; // Set message for input package detection
    //     }
    //     else if (pkgStatus == OUT_PKG)
    //     {
    //         pkgMsg.data = "Package Detected at Output"; // Set message for output package detection
    //     }
    //     else
    //     {
    //         pkgMsg.data = "No Package Detected"; // Set message for no package detection
    //         stepper.stop();                      // Stop the stepper motor if no package is detected
    //     }
    // }
    // else
    // {
    //     pkgMsg.data = "No Package Detected"; // Set message for no package detection
    //     stepper.stop();                      // Stop the stepper motor if no package is detected
    // }

    // pkg_pub.publish(&pkgMsg); // Publish the package status message

}

int checkIN_OUT_pkg(void)
{
    // Read the current level from the IR sensors
    int sensorArray[] = {
        digitalRead(IR_IN_PIN), digitalRead(IR_OUT_PIN)};

    for (int i = 0; i < 2; i++)
    {
        /* Check if the sensor is triggered (LOW means object detected) */
        if (sensorArray[i] == LOW)
        {
            return i;
        }
    }
    // If no sensor is triggered, return NO_PKG_DETECTED
    return NO_PKG_DETECTED; // NO_PKG_DETECTED
}