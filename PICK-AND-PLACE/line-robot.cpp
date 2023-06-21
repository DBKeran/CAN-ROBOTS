#pragma once
#include <mrm-8x8a.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-servo.h>
#include "line-robot.h"
#include "radionica.h"


/** Constructor
  @param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
LineRobot::LineRobot(char name[]) : Robot(name) {
  // MotorGroup class drives the motors
  motorGroup = new MotorGroupDifferential(this, mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 3);

  // All the actions will be defined here; the objects will be created.
  actionPause = new ActionPause(this);
  actionMotorShortTest = new ActionMotorShortTest(this);

  // The actions that should be displayed in menus must be added to menu-callable actions.
  actionAdd(actionMotorShortTest);

  // DISPLAY BUTTON ACTIONS
  // mrm_8x8a->actionSet(actionPause, 0); // Button 1 starts RCJ LineRobot.
  // mrm_8x8a->actionSet(actionMotorShortTest, 1); // Button 2 starts robot in evacution zone.
  mrm_8x8a->actionSet(_actionLoop, 2); // Button 3 starts user defined loop() function
  mrm_8x8a->actionSet(actionPause, 3); // Stop the robot

  // Depending on your wiring, it may be necessary to spin some motors in the other direction.
  //mrm_mot4x3_6can->directionChange(0); // Uncomment to change MOTOR 1 rotation direction
  //mrm_mot4x3_6can->directionChange(1); // Uncomment to change MOTOR 2 rotation direction
  //mrm_mot4x3_6can->directionChange(2); // Uncomment to change MOTOR 3 rotation direction
  //mrm_mot4x3_6can->directionChange(3); // Uncomment to change MOTOR 4 rotation direction

  // Servo motors. Note that some pins are not appropriate for PWM (servo)
  mrm_servo->add(18, (char*)"ServoUp", 0, 300, 0.5, 2.5); // Data for mrm-rds5060-300
  mrm_servo->add(19, (char*)"ServoR", 0, 300, 0.5, 2.5); // Data for mrm-ps-1109hb
  mrm_servo->add(17, (char*)"ServoL", 0, 300, 0.5, 2.5); // Data for mrm-ps-1109hb
  mrm_servo->add(16, (char*)"ServoD", 0, 300, 0.5, 2.5); // Data for mrm-ps-1109hb
  mrm_servo->add(27, (char*)"ServoY", 0, 180, 0.5, 2.5); // Data for mrm-ps-1109hb
  mrm_servo->add(26, (char*)"ServoX", 0, 180, 0.5, 2.5); // Data for mrm-ps-1109hb


  // DIGITAL or ANALOG I/O FOR ADDITIONAL DEVICES
  //pinMode(16, INPUT); // BUTTON (16)

  // If uncommented, robot will immediately after power-on start executing loop()
  //actionSet(_actionLoop);
  armBase();
}

/** Stores bitmaps in mrm-led8x8a.
*/
void LineRobot::bitmapsSet() {
  mrm_8x8a->alive(0, true); // Makes sure that mrm-8x8a is present and functioning. If not, issues a warning message.

  // The 2 arrays will hold red and green pixels. Both red an green pixel on - orange color.
  uint8_t red[8];
  uint8_t green[8];

  /* 1 will turn the pixel on, 0 off. 0bxxxxxxxx is a binary format of the number. Start with "0b" and list all the bits, starting from
    the most significant one (MSB). Do that for each byte of the green and red arrays.*/

  // Define Your bitmaps here.
  // Example
  green[0] = 0b00000001;
  green[1] = 0b00000011;
  green[2] = 0b00000111;
  green[3] = 0b00001111;
  green[4] = 0b00011111;
  green[5] = 0b00111111;
  green[6] = 0b01111111;
  green[7] = 0b11111111;

  red[0] = 0b11111111;
  red[1] = 0b01111111;
  red[2] = 0b00111111;
  red[3] = 0b00011111;
  red[4] = 0b00001111;
  red[5] = 0b00000111;
  red[6] = 0b00000011;
  red[7] = 0b00000001;
  mrm_8x8a->bitmapCustomStore(red, green, LED_CUSTOM);

}

void LineRobot::armBase() {
  mrm_servo->write(120, 0);   // 120 je centar, zbog napetosti kabla dozvoljen raspon od 60(desno)-180(lijevo)
  mrm_servo->write(60, 1);    // 30 je dolje min, 150 je gore max
  mrm_servo->write(50, 2);    // 135 je centar, dozvoljeno je 90 od centra s obdje strane, 45 je min smjer suprotno od žica
  mrm_servo->write(100, 3);   // 60 je min u smjeru žica, 170 je centar,
  mrm_servo->write(10, 4);    // 10 is closed, 90 is open
  mrm_servo->write(100, 5);   // 100 is closed, 10 is open 
}

void LineRobot::armUp() {
  mrm_servo->write(120, 0);
  mrm_servo->write(120, 1);
  mrm_servo->write(135, 2);
  mrm_servo->write(170, 3);
}

void LineRobot::armCatchReady() {
  mrm_servo->write(120, 0);
  mrm_servo->write(60, 1);
  mrm_servo->write(50, 2);
  mrm_servo->write(100, 3);
  mrm_servo->write(90, 4);
  mrm_servo->write(10, 5);
}


void LineRobot::armVerticalCatchReady() {
  mrm_servo->write(120, 0);
  mrm_servo->write(105, 1);
  mrm_servo->write(65, 2);
  mrm_servo->write(270, 3);
}


/** Reads push button switch
  @number - 0 to 3, push button's ordinal number
  @return - true if pressed
*/
bool LineRobot::button(uint8_t number) {
  return mrm_8x8a->switchRead(number);
}

/** Display 8x8 image
  @image - image's number
*/
void LineRobot::display(uint8_t image) {
  mrm_8x8a->bitmapCustomStoredDisplay(image);
}

/** Display 8x8 text
  @image - image's number
*/
void LineRobot::display(char* text) {
  mrm_8x8a->text(text);
}

uint16_t LineRobot::left(uint8_t sampleCount, uint8_t sigmaCount) {
  return mrm_lid_can_b->distance(0, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

uint16_t LineRobot::frontLeft(uint8_t sampleCount, uint8_t sigmaCount) {
  return mrm_lid_can_b->distance(1, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

uint16_t LineRobot::frontRight(uint8_t sampleCount, uint8_t sigmaCount) {
  return mrm_lid_can_b2->distance(2, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

uint16_t LineRobot::right(uint8_t sampleCount, uint8_t sigmaCount) {
  return mrm_lid_can_b->distance(3, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Start motors
  @param leftSpeed, in range -127 to 127
  @param right Speed, in range -127 to 127
  @param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/
void LineRobot::go(int16_t leftSpeed, int16_t rightSpeed) {
  motorGroup->go(leftSpeed, rightSpeed);
}

/** Test - go straight ahead using a defined speed.
*/
void LineRobot::goAhead() {
  const uint8_t speed = 40;
  go(speed, speed);
  end(); // This command will cancel actions and the robot will return in the default idle loop, after displaying menu.
}

/**Compass
  @return - North is 0�, clockwise are positive angles, values 0 - 360.
*/
float LineRobot::heading() {
  return mrm_imu->heading();
}

/**Pitch
  @return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0�.
*/
float LineRobot::pitch() {
  return mrm_imu->pitch();
}

/** Roll
  @return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0�.
*/
float LineRobot::roll() {
  return mrm_imu->roll();
}

/** Custom test. The function will be called many times during the test, till You issue "x" menu command.
*/

void LineRobot::loop() {
  radionica();
}

/** Test motors
*/
void LineRobot::motorShortTest() {
  go(60, 60);
  delayMs(2000);
  go(-60, -60);
  delayMs(2000);
}

/** Stop the robot
*/
void LineRobot::stop() {
  motorGroup->stop();
}

/** Store 8x8 image to 8x8 LED's internal memory
  @red - red pixels
  @green - green pixels
  @image - image's number
*/
void LineRobot::store(uint8_t red[], uint8_t green[], uint8_t image) {
  mrm_8x8a->bitmapCustomStore(red, green, image);
}

/** Line found?
  @return - true if any sensor detects black.
  @param firstTransistor - start checking from this transistor.
  @param lastTransistor - do not check after this one.
*/
bool LineRobot::lineAny(uint8_t firstTransistor, uint8_t lastTransistor) {
  return mrm_ref_can->any(true, 0, firstTransistor, lastTransistor);
}

/** Line sensor
  @param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
  @return - true if black line found
*/
bool LineRobot::line(uint8_t transistorNumber) {
  return mrm_ref_can->dark(transistorNumber);
}
