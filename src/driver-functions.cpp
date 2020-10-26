#include "driver-functions.h"

bool runLiftPID = false;

//0 = down  1 = up
bool liftState = 0;

double liftTarget = 0.0;


double error = 0;
double previousError = 0;

//The max number of degrees that the error can be
double maxError = 1;

double integral = 0;
//within how many degrees of the target the integral activates
double integralBound = 10;

double derivative = 0;


double kP = 0.0;
double kI = 0.0;
double kD = 0.0;

double power = 0.0;

int liftPID() {

  //outer condition turns loop on and off
  while(runLiftPID) {

    //runs the PID while the goal has not been reached
    while(fabs(liftTarget - error) > maxError) {

      error = liftTarget - LiftMotor.rotation(rotationUnits::deg);

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

      LiftMotor.spin(directionType::fwd, power, voltageUnits::volt);


      task::sleep(20);
    }

    if(liftState == 0) {
      runLiftPID = false;
    }
    
  }

  return 1;
}


