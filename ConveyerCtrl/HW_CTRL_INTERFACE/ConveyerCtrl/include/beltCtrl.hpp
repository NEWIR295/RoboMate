/*
    Author: Mohamed Newir
    Date: 3/05/2025
    File: beltCtrl.hpp
    Description:
                Header file for the conveyer control system using ROS and Arduino.
                This file contains the necessary includes, definitions, and function declarations
                for the belt control system.
                It sets up the pins for the stepper motor control and IR sensors, initializes the ROS node,
                and provides functions for belt control and movement.
*/

#ifndef BELT_CTRL_HPP
#define BELT_CTRL_HPP

/*
    used libraries:
    - ros.h: For ROS communication
    - std_msgs: For standard message types like Empty, Int32, Float32, Bool, String
    - util/atomic.h: For atomic operations, ensuring thread safety in the context of ROS communication
    - Arduino.h: For basic Arduino functionality
    - AccelStepper: For controlling stepper motors
*/
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <util/atomic.h>
#include <AccelStepper.h>

/*
    set the pins for the motor control
    These pins are used to control the stepper motor for the conveyer system.
*/
#define STEP_PIN 9
#define DIR_PIN 8
#define ENABLE_PIN 7
#define MOTOR_SPEED 2667       // Set the speed of the stepper motor
#define MOTOR_ACCELERATION 500 // Set the acceleration of the stepper motor
#define MOTOR_MAX_SPEED 3000   // Set the maximum speed of the stepper motor
#define MOTOR_INTERFACE_TYPE 1 // 1 for stepper motor, 0 for DC motor or i could use AccelStepper::DRIVER instead of 1

/*
    set the pins for the IR sensors
*/
#define IR_IN_PIN 2
#define IR_OUT_PIN 3

extern ros::NodeHandle nh; // Declare the nh object as external

/*
    functions to be implemented in the conveyerCtrl.cpp file
*/
void beltCtrlInit(void);
void beltMove(void);

#endif // BELT_CTRL_HPP