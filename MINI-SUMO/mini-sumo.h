#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

// DEFINIRATI KONSTANTE
// #define KONSTANTA 160 // primjer definiranja konstante KONSTANTA s nepromjenjivom vrijednosti 160


// mrm-8x8a display bitmaps.
enum ledSign {LED_CUSTOM
             };

/* All the Action-classes have to be forward declared here (before MiniSumo) as MiniSumo declaration uses them. The other option would be
  not to declare them here, but in that case Action-objects in MiniSumo will have to be declared as ActionBase class, forcing downcast later in code, if
  derived functions are used.*/
class ActionPause;
class ActionMotorShortTest;

/** Robot for MiniSumo, a class derived from the base Robot class.
*/
class MiniSumo : public Robot {

    // GLOBALNE VARIJABLE
    // int broj = 0; // primjer globalne varijable broj s početnom vrijednosti 0

    // Actions' declarations
    ActionPause* actionPause;
    ActionMotorShortTest* actionMotorShortTest;

    MotorGroupDifferential* motorGroup = NULL; // Class that conveys commands to motors.

  public:
    /** Constructor
      @param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
    */
    MiniSumo(char name[] = (char*)"SUMO"); // Maximum 15 characters

    /** Stores bitmaps in mrm-led8x8a.
    */
    void bitmapsSet();

    /** Reads push button switch
      @number - 0 to 3, push button's ordinal number
      @return - true if pressed
    */
    bool button(uint8_t number);

    /** Display 8x8 image
      @image - image's number
    */
    void display(uint8_t image);

    /** Display 8x8 text
      @image - image's number
    */
    void display(char* text);

    /** Distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
      @param sampleCount - Number or readings. 40% of the raeadings, with extreme values, will be discarded and the
    				rest will be averaged. Keeps returning 0 till all the sample is read.
    				If sampleCount is 0, it will not wait but will just return the last value.
      @param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
    				Therefore, lower sigma number will remove more errornous readings.
      @return - distance in mm
    */

    uint16_t left(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);
    
    uint16_t frontLeft(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

    uint16_t frontRight(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

    uint16_t right(uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

    /** Start motors
      @param leftSpeed, in range -127 to 127
      @param right Speed, in range -127 to 127
      @param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
    */
    void go(int16_t leftSpeed, int16_t rightSpeed);

    /** Custom test
    */
    void loop();

    /** Custom test u odvojenom headeru
     */
    void radionica();

    /** Test - go straight ahead using a defined speed.
    */
    void goAhead();

    /** Test motors
    */
    void motorShortTest();

    /**Compass
      @return - Clockwise are positive angles, values 0 - 360.
    */
    float heading();

    /**Pitch
      @return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0�.
    */
    float pitch();

    /** Roll
      @return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0�.
    */
    float roll();

    /** Stop the robot
    */
    void stop();

    /** Store 8x8 image to 8x8 LED's internal memory
      @red - red pixels
      @green - green pixels
      @image - image's number
    */
    void store(uint8_t red[], uint8_t green[], uint8_t image);

};

/** Actions serve a few purposes.
  - They encapsulate in classes actions robot has to perform. So, we have classes for robot's parts, but here also for non-material terms.
  - No global variables are used. When an information should be shared between one (but called repeatedly) or more functions, it will be stored inside the action object. For example, all the
  start conditions will be in the object itself.
  - You can use inheritance to indicate relationships between actions, which indeed exist. For example, a movement can be movement straight ahead or turning.
  - You can use in a consistent way actions defined for the base robot, without its code being exposed here.
  - The actions are included in menus just by including a parameter in the constructor call.
  - Buttons can be used to start actions, as well as menu commands. Menus are displayed both in the connected PC and a Bluetooth device, like a mobile phone, and any of the 2 can be used to
  issue commands.
*/

/** Actions specific for a MiniSumo robot will be defined here. They are all derived from ActionBase class (inheriting its methods and properties).
  They also all feature perform() function, the one that is called to do what they are supposed to. Their constructors always call base class' constructors [ActionBase(...)] with 3 or more
  parameters specified here.
  First parameter is robot and is always the same.
  The second one is a 3-letter shortcut that is displayed in command menu. For example "lin" will be displayed for starting the Rescue Line run. When action is not supposed to be started from menu,
  it can be an empty string.
  The third parameter is a name of the action, again displayed in menu. For "lin", the name is "RCJ Line", causing menu entry "line - RCJ Line" to be displayed. Again, use empty string
  for no-menu actions.
  The fourth pareameter is menu level. When omitted, the action will not be a part of the menu. Use 1 otherwise. Higher level numbers will display the action in submenues, not used here.
*/

// ****************** Generic actions

class ActionPause : public ActionBase {
    void perform() {
      ((MiniSumo*)_robot)->stop();
    }
  public:
    ActionPause(Robot* robot) : ActionBase(robot, "sto", "Stop") {}
};

class ActionMotorShortTest : public ActionBase {
    void perform() {
      ((MiniSumo*)_robot)->motorShortTest();
    }
  public:
    ActionMotorShortTest(Robot* robot) : ActionBase(robot, "msh", "Motor short test") {}
};
