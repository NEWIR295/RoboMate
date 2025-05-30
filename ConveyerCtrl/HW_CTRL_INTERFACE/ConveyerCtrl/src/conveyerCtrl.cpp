/*
    Author: Mohamed Newir
    Date: 3/05/2025
    File: conveyerCtrl.cpp
    Description:
                Implementation file for the conveyer control system using ROS and Arduino.
                This file contains the definitions of functions for initializing the conveyer control system,
                moving the conveyer, checking the current level using IR sensors, and handling ROS communication.
*/

#include "../include/conveyerCtrl.hpp"

/*
    Function to check the current level of the conveyer using IR sensors.
    This function reads the state of the IR sensors and returns the level detected.
    Returns:
        int: The level detected by the IR sensors, represented as an integer.
        - LEVEL1: 1
        - LEVEL2: 2
        - LEVEL3: 3
        - NO_LEVEL: 4 (if no level is detected)
*/
int checkLevel(void);

/*
    Enum to represent the different levels of the conveyer.
    This enum is used to identify which level the conveyer is currently at based on the IR sensor readings.
*/
enum Level
{
    LEVEL1 = 1, // Represents Level 1 = 1
    LEVEL2,
    LEVEL3,
    NO_LEVEL // Represents no level detected = 4
};
int currentLevel = NO_LEVEL; // Initialize current level to NO_LEVEL

/* set RTOS parameters */
unsigned long prevTime = 0; // init prev time with 0

/* global var to confirm its level */
std_msgs::String posMsg;
// std_msgs::String emptyString;

/* Publisher for position message */
ros::Publisher pub("/current_position", &posMsg);

/* global var for level target flag for IR sensors */
std_msgs::Int8 targetLevel;
/* last status of publishing */
String lastStatus = "";

/* subscriber cb for target level */
void callback(const std_msgs::Int8 &levelMsg)
{
    targetLevel = levelMsg;
}

/* sub to "/target_level" topic for desired Level */
ros::Subscriber<std_msgs::Int8> sub("/target_level", &callback);

/*
Function to initialize the conveyer control system.
This function sets up the necessary pins for motor control and IR sensors.
*/
void conveyerCtrlInit(void)
{

    nh.advertise(pub);
    nh.subscribe(sub);
    // Initialize the motor pins
    pinMode(MOTOR_PIN1, OUTPUT);
    pinMode(MOTOR_PIN2, OUTPUT);

    // Initialize the IR sensor pins
    pinMode(IR_LEVEL1_PIN, INPUT);
    pinMode(IR_LEVEL2_PIN, INPUT);
    pinMode(IR_LEVEL3_PIN, INPUT);

    // Set initial state of motors to LOW
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);

    currentLevel = checkLevel(); // Get the initial level from the IR sensors
    // emptyString.data = ""; // Initialize an empty String object
}

void conveyerMove(void)
{

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        unsigned long currentTime = millis(); // Get the current time in milliseconds

        // Check if the interval has passed
        if (currentTime - prevTime >= INTERVAL)
        {
            // Checks if there is a new target level is set
            if (targetLevel.data != NO_LEVEL)
            {
                currentLevel = checkLevel(); // Get the current level from the IR sensors

                if (currentLevel > targetLevel.data)
                {

                    // If the current level is greater than the target level, move down
                    digitalWrite(MOTOR_PIN1, LOW);
                    digitalWrite(MOTOR_PIN2, HIGH);
                    posMsg.data = "Moving Down"; // Set the position message to indicate moving down
                }
                else if (currentLevel < targetLevel.data)
                {

                    // If the current level is less than the target level, move up
                    digitalWrite(MOTOR_PIN1, HIGH);
                    digitalWrite(MOTOR_PIN2, LOW);
                    posMsg.data = "Moving Up"; // Set the position message to indicate moving up
                }
                else if (currentLevel == targetLevel.data)
                {
                    // If the current level is equal to the target level, stop the motors
                    digitalWrite(MOTOR_PIN1, LOW);
                    digitalWrite(MOTOR_PIN2, LOW);

                    /*
                        write here the conveyer belt logic for steeper control
                        If the current level is equal to the target level, stop the motors
                        This is where you can implement logic for the conveyer belt to stop or perform other actions.
                        it will include a subscriber to a topic that controls the conveyer belt, also make it separate
                        node from level control node.
                    */

                    posMsg.data = "Stopped"; // Set the position message to indicate stopped state
                }
            }
            else
            {
                // If no target level is set, stop the motors
                digitalWrite(MOTOR_PIN1, LOW);
                digitalWrite(MOTOR_PIN2, LOW);
                posMsg.data = "No Target Level"; // Set the position message to indicate no target level
            }

            /*
                Publish the position message if it has changed.
                This is done to avoid unnecessary publishing of the same message.
                It checks if the last status is different from the current position message.
            */
            if (lastStatus != posMsg.data) // Check if the position message has changed
            {
                lastStatus = posMsg.data; // Update the last status to the current position message
                pub.publish(&posMsg);     // Publish the position message
            }

            currentLevel = checkLevel(); // Get the current level from the IR sensors
            prevTime = currentTime;      // Update the previous time
        }
    }
}

int checkLevel(void)
{
    // Read the current level from the IR sensors
    int sensorArray[] = {
        digitalRead(IR_LEVEL1_PIN), digitalRead(IR_LEVEL2_PIN), digitalRead(IR_LEVEL3_PIN)};

    for (int i = 0; i < 3; i++)
    {
        if (sensorArray[i] == HIGH)
        {
            return i + 1;
        }
    }
    // If no sensor is triggered, return NO_LEVEL
    return NO_LEVEL; // NO_LEVEL
}