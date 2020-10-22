#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LFDrive = motor(PORT1, ratio18_1, false);
motor RFDrive = motor(PORT2, ratio18_1, true);
motor LBDrive = motor(PORT10, ratio18_1, false);
motor RBDrive = motor(PORT11, ratio18_1, true);
motor LeftIntake = motor(PORT4, ratio18_1, false);
motor RightIntake = motor(PORT5, ratio18_1, true);
motor TopRoller = motor(PORT20, ratio18_1, false);
motor LiftMotor = motor(PORT18, ratio36_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}