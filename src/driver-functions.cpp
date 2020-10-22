#include "driver-functions.h"

bool runLiftPID = false;

int LiftPID(double target) {

  double error = 0;
  double previousError = 0;

  //The max number of degrees that the error can be
  double maxError = 1;


  double integral = 0;
  //within how many degrees the integral activates
  double integralBound = 10;

  double derivative = 0;


  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;



  while(runLiftPID) {
    //condition of reaching goal here
    while(fabs(target - error) > maxError) {

      error = target - LiftMotor.rotation(rotationUnits::deg);

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

    }

  }

  return 1;
}


