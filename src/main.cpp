/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LFDrive              motor         1               
// RFDrive              motor         2               
// LBDrive              motor         10              
// RBDrive              motor         11              
// LeftIntake           motor         4               
// RightIntake          motor         5               
// TopRoller            motor         20              
// LiftMotor            motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "driver-functions.h"

using namespace vex;

// A global instance of competition
competition Competition;

//tasks
task liftTask;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  
  //create a task to control the lift with a PID function
  liftTask = task(liftPID);

  while (1) {

    /* CHASSIS */

    double driveAmt = Controller1.Axis3.value();
    double turnAmt = Controller1.Axis1.value();
    double strafeAmt = Controller1.Axis4.value();

    LFDrive.spin(directionType::fwd, driveAmt - turnAmt + strafeAmt, velocityUnits::pct);
    RFDrive.spin(directionType::fwd, driveAmt + turnAmt - strafeAmt, velocityUnits::pct);
    LBDrive.spin(directionType::fwd, driveAmt - turnAmt - strafeAmt, velocityUnits::pct);
    RBDrive.spin(directionType::fwd, driveAmt + turnAmt + strafeAmt, velocityUnits::pct);


    /* LIFT */

    //the number of degrees the lift should target for the up and down states
      //liftBottomTarget is more than 0 so that the lift can coast down the last little bit and not press against the hard stops
    double liftBottomTarget = 10;
    double liftTopTarget = 100;

    if(Controller1.ButtonL1.pressing()) {
      liftState = 1;
      liftTarget = liftTopTarget;
      runLiftPID = true;
    }
    else if(Controller1.ButtonL2.pressing()) {
      liftState = 0;
      liftTarget = liftBottomTarget;

    }


    /* INTAKES */

    //If the tray is down, run all intakes with each other
    if(liftState == 0) {
      
      if(Controller1.ButtonR1.pressing()) {
        LeftIntake.spin(directionType::fwd, 100, velocityUnits::pct);
        RightIntake.spin(directionType::fwd, 100, velocityUnits::pct);
        TopRoller.spin(directionType::fwd, 100, velocityUnits::pct);
      }
      else if(Controller1.ButtonR2.pressing()) {
        LeftIntake.spin(directionType::rev, 100, velocityUnits::pct);
        RightIntake.spin(directionType::rev, 100, velocityUnits::pct);
        TopRoller.spin(directionType::rev, 100, velocityUnits::pct);
      }

    }
    //If the tray is UP, only intake with the side rollers and only outtake with the top roller
    else {

      if(Controller1.ButtonR1.pressing()) {
        LeftIntake.spin(directionType::fwd, 100, velocityUnits::pct);
        RightIntake.spin(directionType::fwd, 100, velocityUnits::pct);
      }
      else if(Controller1.ButtonR2.pressing()) {
        TopRoller.spin(directionType::rev, 100, velocityUnits::pct);
      }

    }
    

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
