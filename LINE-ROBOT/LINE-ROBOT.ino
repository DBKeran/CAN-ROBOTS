#include <Arduino.h>
#include <mrm-8x8a.h>         // DISPLAY
#include <mrm-can-bus.h>       // CAN
#include <mrm-common.h>        // TO ENABLE BLUETOOTH
#include <mrm-imu.h>          // IMU
#include <mrm-lid-can-b.h>    // 2m LIDARS
#include <mrm-mot4x3.6can.h>  // MOTOR DRIVER 
#include <mrm-node.h>          // CAN
#include <mrm-robot.h>        // BASE ROBOT LIBRARY
#include <mrm-ref-can.h>      // REFLECTIVE IR SENSORS
#include <mrm-servo.h>        // SERVO MOTOR LIBRARY
#include "line-robot.h"

Robot *robot;

void setup() {
  robot = new LineRobot((char*)"LINEROBOT"); // RobotLine, RobotMaze, RobotMin, RobotSoccer, or Your custom robot. "SUMO" is Bluetooth name.
  robot->print("Start.\n\r");
}

void loop() {
  robot->refresh();
}
