#include <Arduino.h>
#include <mrm-8x8a.h>           // DISPLAY
#include <mrm-board.h>
#include <mrm-can-bus.h>        // CAN
#include <mrm-common.h>         // BLUETOOTH ENABLER
#include <mrm-imu.h>            // IMU
#include <mrm-ir-finder3.h>     // IR BALL SENSOR
#include <mrm-lid-can-b2.h>     // 4m LIDAR
#include <mrm-mot4x3.6can.h>    // MOTOR
#include <mrm-node.h>
#include <mrm-ref-can.h>        // LINE SENSORS
#include <mrm-robot.h>
#include "mrm-robot-soccer.h"


Robot *robot;

void setup() {
  robot = new RobotSoccer((char*)"SoccerBot"); 
  robot->print("Start.\n\r");
}

void loop() {
  robot->refresh();
}
 
