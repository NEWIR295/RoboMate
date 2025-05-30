/*
    Author: Mohamed Newir
    Date: 3/05/2025
    File: conveyerCtrl.hpp
    Description:
                Header file for the conveyer control system using ROS and Arduino.
                This file contains the necessary includes, definitions, and function declarations
                for the conveyer control system.
                It sets up the pins for motor control and IR sensors, initializes the ROS node,
                and provides functions for conveyer control and movement.
*/

#ifndef CONVEYERCTRL_HPP
#define CONVEYERCTRL_HPP

/*
    used libraries:
    - ros.h: For ROS communication
    - std_msgs: For standard message types like Empty, Int32, Float32, Bool, String
    - util/atomic.h: For atomic operations, ensuring thread safety in the context of ROS communication
    - Arduino.h: For basic Arduino functionality
*/
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <util/atomic.h>

/*
    set the pins for the motor control
*/
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6

/*
    set the pins for the IR sensors
*/
#define IR_LEVEL1_PIN 2
#define IR_LEVEL2_PIN 3
#define IR_LEVEL3_PIN 4

#define INTERVAL 100.0 // set rtos intervals

extern ros::NodeHandle nh; // Declare the nh object as external

/*
    functions to be implemented in the conveyerCtrl.cpp file
*/
void conveyerCtrlInit(void);
void conveyerMove(void);

#endif // CONVEYERCTRL_HPP