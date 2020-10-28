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
// LFDrive              motor         11              
// RFDrive              motor         1               
// LBDrive              motor         20              
// RBDrive              motor         10              
// LeftIntake           motor         13              
// RightIntake          motor         3               
// TopRoller            motor         9               
// LiftMotor            motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"


using namespace vex;

// A global instance of competition
competition Competition;

//tasks


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
  LiftMotor.resetRotation();
  
  //create a task to control the lift with a PID function

  double error = 0;
  double previousError = 0;

  //The max number of degrees that the error can be
  double maxError = 1;

  double integral = 0;
  //within how many degrees of the target the integral activates
  double integralBound = 10;

  double derivative = 0;

  double kP = 0.7;
  double kI = 0.0;
  double kD = 0.0;

  double power = 0.0;

  double driveAmt;
  double turnAmt;
  double strafeAmt;

  bool liftState = 0;

  bool runLiftPID = 0;

  double liftTarget = 0.0;

  //how many degrees the motor should spin to the get the lift to the top/bottom of its path
  double liftBottomTarget = 5;
  double liftTopTarget = 500;

  while (1) {

    /* CHASSIS */

    driveAmt = Controller1.Axis3.value();
    turnAmt = Controller1.Axis1.value();
    strafeAmt = Controller1.Axis4.value();

    LFDrive.spin(directionType::fwd, driveAmt + turnAmt + strafeAmt, velocityUnits::pct);
    RFDrive.spin(directionType::fwd, driveAmt - turnAmt - strafeAmt, velocityUnits::pct);
    LBDrive.spin(directionType::fwd, driveAmt + turnAmt - strafeAmt, velocityUnits::pct);
    RBDrive.spin(directionType::fwd, driveAmt - turnAmt + strafeAmt, velocityUnits::pct);


    /* LIFT */

    if(Controller1.ButtonL1.pressing()) {
      liftState = 1;
      liftTarget = liftTopTarget;
      runLiftPID = 1;
    }
    else if(Controller1.ButtonL2.pressing()) {
      liftState = 0;
      liftTarget = liftBottomTarget;

    }



    //LIFT PID CONTROLLER
    
    error = liftTarget - LiftMotor.rotation(rotationUnits::deg);

    if(runLiftPID && fabs(error) > maxError) {
     
      //Brain.Screen.print(error);

      if(fabs(error) < integralBound) {
        integral += error;
      }
      else {
        integral = 0;
      }
      //reset integral if we pass the target
      if(error * previousError < 0) {
        integral = 0;
      }

      derivative = error - previousError;

      previousError = error;

      power = kP * error + kI * integral + kD * derivative;

      //let the lift coast to its bottom resting point
      if(liftState == 0 && LiftMotor.rotation(rotationUnits::deg) < (liftBottomTarget + 10)) {
        LiftMotor.stop(brakeType::coast);
      }
      else {
        LiftMotor.spin(directionType::fwd, power, voltageUnits::volt);
      }
      

    }

    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.print(runLiftPID);


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
      else{
        LeftIntake.stop(brakeType::brake);
        RightIntake.stop(brakeType::brake);
        TopRoller.stop(brakeType::brake);
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
      else{
        LeftIntake.stop(brakeType::brake);
        RightIntake.stop(brakeType::brake);
        TopRoller.stop(brakeType::brake);
      }


    }
    

    task::sleep(20); // Sleep the task for a short amount of time to
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
