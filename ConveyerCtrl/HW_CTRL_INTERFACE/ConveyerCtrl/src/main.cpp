/*
      Author: Mohamed Newir
      Date: 3/05/2025
      File: main.cpp
      Description:
                    Main file for the conveyer control system using ROS and Arduino.
                    This file initializes the ROS node and the conveyer control system,
                    and runs the main loop to handle conveyer movement and ROS communication.
*/

#include <Arduino.h>
#include "../include/conveyerCtrl.hpp"

ros::NodeHandle nh;

void setup()
{
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(38400);
  // nh.getHardware()->setBaud(57600);
  nh.initNode();
  // mpu_init();
  conveyerCtrlInit();
}

void loop()
{
  // put your main code here, to run repeatedly:

  conveyerMove();
  nh.spinOnce();
}
