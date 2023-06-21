#include <mrm-8x8a.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b2.h>
#include <mrm-ir-finder3.h>
#include <mrm-mot4x3.6can.h>
#include "mrm-robot-soccer.h"

/** Constructor
  @param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotSoccer::RobotSoccer(char name[]) : Robot(name) {
  motorGroup = new MotorGroupStar(this, mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 3);

  // LED signs to be assigned to different actions follow. It is easier to follow action flow by checking the display.
  // LED Calibrate
  LEDSignText* signCalibrate = new LEDSignText(); // Here, a text will be displayed instead of a 8x8 bitmap.
  strcpy(signCalibrate->text, "Calibr.");

  // Actions
  pidXY = new Mrm_pid(0.5, 100, 0); // PID controller, regulates motors' speeds for linear motion in the x-y plane: 4, 100, 0 - ok.
  pidRotation = new Mrm_pid(2.0, 100, 0); // PID controller, regulates rotation around z axis
  actionCalibrate = new ActionSoccerCalibrate(this, signCalibrate);

  // The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
  // right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
  // called in the code, but only through menus. For example, ActionWallsTest test is only manu-based, and it is all right.
  // This test is not supposed to be called in code.
  actionAdd(actionCalibrate);

  // mrm_mot4x3_6can->directionChange(0); // Uncomment to change 1st wheel's rotation direction
  // mrm_mot4x3_6can->directionChange(1); // Uncomment to change 2nd wheel's rotation direction
  // mrm_mot4x3_6can->directionChange(2); // Uncomment to change 3rd wheel's rotation direction
  // mrm_mot4x3_6can->directionChange(3); // Uncomment to change 4th wheel's rotation direction

  // Buttons
  //mrm_8x8a->actionSet(actionPlay, 0); // Button 1 starts the play
  //mrm_8x8a->actionSet(actionBounce, 1); // Button 2 starts user defined bounce() function
  mrm_8x8a->actionSet(_actionLoop, 2); // Button 3 starts user defined loop() function
  mrm_8x8a->actionSet(actionCalibrate, 3);

  // Set number of phototransistors in each line sensor.
  mrm_ref_can->transistorCountSet(5, 0); // 5 instead of 6 since IR ball interferes with 6th transistor.
  mrm_ref_can->transistorCountSet(8, 1);
  mrm_ref_can->transistorCountSet(8, 2);
  mrm_ref_can->transistorCountSet(8, 3);
}

/** MAIN LOOP
*/
void RobotSoccer::loop() {

}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotSoccer::bitmapsSet() {
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

/** Reads push button switch
  @number - 0 to 3, push button's ordinal number
  @return - true if pressed
*/
bool RobotSoccer::button(uint8_t number) {
  return mrm_8x8a->switchRead(number);
}

/** Distance to wall
  @param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
        rest will be averaged. Keeps returning 0 till all the sample is read.
        If sampleCount is 0, it will not wait but will just return the last value.
  @param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
        Therefore, lower sigma number will remove more errornous readings.
  @return - in mm
*/
uint16_t RobotSoccer::front(uint8_t sampleCount, uint8_t sigmaCount) {
  return mrm_lid_can_b2->distance(0, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

uint16_t RobotSoccer::back(uint8_t sampleCount, uint8_t sigmaCount) {
  return mrm_lid_can_b2->distance(2, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

uint16_t RobotSoccer::left(uint8_t sampleCount, uint8_t sigmaCount) {
  return mrm_lid_can_b2->distance(3, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

uint16_t RobotSoccer::right(uint8_t sampleCount, uint8_t sigmaCount) {
  return mrm_lid_can_b2->distance(1, sampleCount, sigmaCount); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Read barrier
  @return - true if interrupted
*/
bool RobotSoccer::barrier() {
  return analogRead(35) < 3900; // Adjust this value
}

/** Line sensor
  @param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
  @param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
  @return - true if white line found
*/
bool RobotSoccer::line(uint8_t transistorNumber, uint8_t deviceNumber) {
  return !mrm_ref_can->dark(transistorNumber, deviceNumber);
}

bool RobotSoccer::lineAny() {
  const bool AVOID_LINE = false;
  const bool ENABLE_FRONT_SENSOR = false;
  if (AVOID_LINE)
    for (uint8_t i = (ENABLE_FRONT_SENSOR ? 0 : 1); i < 4; i++)
      if (mrm_ref_can->any(false))
        return true;
  return false;
}

/** Line sensor - brightness of the surface
  @param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
  @param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
  @return - brightness as an analog value.
*/
uint16_t RobotSoccer::brightness(uint8_t transistorNumber, uint8_t deviceNumber) {
  return mrm_ref_can->reading(transistorNumber, deviceNumber);
}

/** Calibrate all line sensors
*/
void RobotSoccer::calibrate() {
  go(0, 0, 25, 100);
  mrm_ref_can->calibrate(0);
  mrm_ref_can->calibrate(1);
  mrm_ref_can->calibrate(2);
  mrm_ref_can->calibrate(3);
  go(0, 0, 0, 0);
  end();
}

/**Compass
  @return - North is 0�, clockwise are positive angles, values 0 - 360.
*/
float RobotSoccer::heading() {
  return mrm_imu->heading();
}

/** Ball's direction
  @return - robot's front is 0�, positive angles clockwise, negative anti-clockwise. Back of the robot is 180�.
*/
int16_t RobotSoccer::ballAngle() {
  return mrm_ir_finder3->angle();
}

/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
  @param speed - 0 to 100.
  @param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
  Values between -180 and 180.
  @param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
  numbers because a value 100 turns on all the motors at maximal speed.
  @param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/
void RobotSoccer::go(float speed, float angleDegrees, float rotation, uint8_t speedLimit) {
  motorGroup->go(speed, angleDegrees, rotation, speedLimit);
}

void RobotSoccer::goAhead() {
  const uint8_t speed = 60;
  motorGroup->go(speed);
  end();
}

float RobotSoccer::headingRandom(int heading, int variation) {
  float newHeading = heading + (2 * (rand() % variation) - variation);
  if (newHeading > 180)
    newHeading -= 360;
  else if (newHeading < -180)
    newHeading += 360;
  return newHeading;
}

/** Bouncing off the lines
*/
void RobotSoccer::bounce() {
  static int directionCurrent = 45;
  const int VARIATION = 45;
  const int SPEED = 127;
  const bool AVOID_LINE = false;
  if (setup()) {
    headingToMaintain = heading();
  }

  if (AVOID_LINE && lineAny()) {
    //actionSet(actionLineAvoid);
    if (mrm_ref_can->any(false, 0)) // Front
      directionCurrent = headingRandom(-180, VARIATION);
    if (mrm_ref_can->any(false, 1)) // Right
      directionCurrent = headingRandom(-90, VARIATION);
    if (mrm_ref_can->any(false, 2)) // Back
      directionCurrent = headingRandom(0, VARIATION);
    if (mrm_ref_can->any(false, 3)) // Left
      directionCurrent = headingRandom(90, VARIATION);
  }
  else {
    uint16_t minDistance = left();
    if (right() < minDistance)
      minDistance = right();
    if (front() < minDistance)
      minDistance = front();
    if (back() < minDistance)
      minDistance = back();
    go(map(minDistance, 0, 700, 40, SPEED), directionCurrent, pidRotation->calculate(heading() - headingToMaintain));
    if (left() < 200) {
      directionCurrent = headingRandom(90, VARIATION);
      // print("Left %i\n\r", directionCurrent);
    }
    if (right() < 200) {
      directionCurrent = headingRandom(-90, VARIATION);
      // print("Right %i\n\r", directionCurrent);
    }
    if (front() < 200) {
      directionCurrent = headingRandom(180, VARIATION);
      // print("Front %i\n\r", directionCurrent);
    }
    if (back() < 200) {
      directionCurrent = headingRandom(0, VARIATION);
      // print("Back %i\n\r", directionCurrent);
    }
    // for (uint8_t i = 0; i < 4; i++)

    static uint32_t ms = 0;
    if (millis() - ms > 1000) {
      print("Target: %i, current: %i\n\r", (int)headingToMaintain, (int)heading());
      ms = millis();
    }
  }
}

void RobotSoccer::lineAvoid() {
  static TriState lineLeft;
  static TriState lineFront;
  static float escapeDirection;
  const uint16_t WALL_DISTANCE = 550;

  if (setup()) {
    print("Line avoid enter\n\r");
    lineLeft = TriState::Unknown;
    lineFront = TriState::Unknown;
  }

  // Line front?
  if (mrm_ref_can->any(false, 0) && lineFront == TriState::Unknown && front() < WALL_DISTANCE)
    lineFront = TriState::Yes, print("Front");

  // Line right?
  if (mrm_ref_can->any(false, 1) && lineLeft == TriState::Unknown && right() < WALL_DISTANCE)
    lineLeft = TriState::Opposite, print("Right");

  // Line back?
  if (mrm_ref_can->any(false, 2) && lineFront == TriState::Unknown && back() < WALL_DISTANCE)
    lineFront = TriState::Opposite, print("Back");

  // Line left?
  if (mrm_ref_can->any(false, 3) && lineLeft == TriState::Unknown && left() < WALL_DISTANCE)
    lineLeft = TriState::Yes, print("Left");

  if (lineLeft != TriState::Unknown || lineFront != TriState::Unknown) { // A line, not a false alarm
    int8_t x = 0;
    int8_t y = 0;
    if (lineFront == TriState::Yes)
      y--;
    else if (lineFront == TriState::Opposite)
      y++;
    if (lineLeft == TriState::Yes)
      x++;
    else if (lineLeft == TriState::Opposite)
      x--;
    escapeDirection = atan2(x, y) / PI * 180;
    print("Line avoid, F:%i R:%i B:%i L:%i Esc dir: %i L:%i F:%i\n\r", mrm_ref_can->any(false, 0), mrm_ref_can->any(false, 1),
          mrm_ref_can->any(false, 2), mrm_ref_can->any(false, 3), (int)escapeDirection, lineLeft, lineFront);

    go(70, escapeDirection, pidRotation->calculate(heading() - headingToMaintain), 100);
    if (!lineAny()) {
      print("Escaped\n\r");
      lineLeft = TriState::Unknown;
      lineFront = TriState::Unknown;
      //actionSet(actionIdle);
      //actionSet(actionBounce);
    }
  }
  else {}
  //actionSet(actionIdle), print("False line");
  //actionSet(actionBounce), print("False line");
}


/** Approach oppoent's goal
*/
void RobotSoccer::goalApproach() {
  if (lineAny()) {
    //actionSet(actionLineAvoid);
  }
  else if (!barrier()) {
    //actionSet(actionIdle);
  }
  else {
    float errorL = 700 - left();
    float errorR = right() - 700;
    float errorX = fabsf(errorL) > fabsf(errorR) ? errorL : errorR;
    float errorY = front();

    float direction = atan2(errorX, errorY) / PI * 180;
    go(60, direction, pidRotation->calculate(heading() - headingToMaintain));
    print("Goal approach\n\r");
  }
}

/** Go around the ball and approach it.
*/
void RobotSoccer::catchBall() {
  if (lineAny()) {
    //actionSet(actionLineAvoid);
  }
  else if (barrier()) {
    //actionSet(actionGoalApproach);
  }
  else if (mrm_ir_finder3->distance() > 100) {
    float direction = -mrm_ir_finder3->angle() - 10;
    if (fabsf(direction) > 7)
      direction += (direction > 0 ? 60 : -60);
    go(40, direction, pidRotation->calculate(heading() - headingToMaintain), 100);
    print("Catch ball, angle: %i\n\r", (int)mrm_ir_finder3->angle());
  }
  else {}
  //actionSet(actionIdle);
}

/** No ball detected - return to Your goal.
*/
void RobotSoccer::idle() {
  if (lineAny()) {
    //actionSet(actionLineAvoid);

  }
  else if (mrm_ir_finder3->distance() > 50) {
    //actionSet(actionCatch);
  }
  else {
    float errorL = 700 - left();
    float errorR = right() - 700;
    float errorX = left() > right() ? errorL : errorR;
    float errorY = 300 - back();
    float speed = 0; // Default: if no room, stay put
    if (left() > 600 || right() > 600 || back() > 500)
      speed = pidXY->calculate(fabsf(errorX) + fabsf(errorY), false, 60);
    float angularSpeed = pidRotation->calculate(heading() - headingToMaintain, false, 40);

    float direction = atan2(errorX, errorY) / PI * 180;
    go(speed, direction, angularSpeed);
    print("Idle: L:%i/%i R:%i/%i Hea:%i\n\r", left(), (int)errorL, right(), (int)errorR, (int)direction);
    //motorGroup->goToEliminateErrors(errorL > errorR ? errorL : errorR, 200 - back(), heading() - headingToMaintain, pidXY, pidRotation, true);
  }
}

/** Starts robot
*/
void RobotSoccer::play() {
  if (motorGroup == NULL) {
    print("Define robot->motorGroupStar first.\n\r");
    return;
  }
  headingToMaintain = mrm_imu->heading();
  print("Yaw: %i\n\r", (int)headingToMaintain);
  //actionSet(actionIdle);
  //actionSet(actionBounce);
}
