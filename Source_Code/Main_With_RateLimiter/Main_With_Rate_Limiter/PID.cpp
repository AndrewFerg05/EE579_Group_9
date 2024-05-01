#include "PID.h"
#include "BT_Comms.h"
#include <cstdlib>

float error = 0;
float derivative = 0;
float output = 0;
float setup_flag = 0;



//// TUNE kp, ki AND kd
void setupPID(PIDConfig* steer, int proportionalGain, int integralGain, int derivativeGain, float integralOverflow)
{
    steer->kp = 0.75;
    steer->ki = 0;
    steer->kd = 1;
    steer->overflow = integralOverflow;
    steer->previousError = 0;
    steer->integral = 0;
}
//
//float PID(PIDConfig* steer, float setPoint, float currentPoint)
//{
//    // Initialise Previous Error
//    if (setup_flag == 0)
//    {
//        setup_flag = 1;
//        steer->prevError = normalizeAngle180(setPoint - currentPoint);
//    }
//
//    // Calculate Error
//    error = normalizeAngle180(setPoint - currentPoint);
//
//    // Calcualte Derivative Term
//    derivative = error - steer->prevError;
//
//    // Determine Output
//    output = (steer->kp * error) + (steer->ki * steer->integral) + (steer->kd * derivative);
//
//    // Store Preious Error and Integral Error
//    steer->integral += error;
//    steer->prevError = error;
//
//
//    // If Integral Exceed Overflow Reset
//    if (steer->integral > steer->overflow)
//    {
//        steer->integral = 0;
//    }
//
//    return output;
//}


float rateLimiter(int desiredAngle, int currentAngle, int currentSteering)
{
  
  int error = desiredAngle - currentAngle;
  int maxRate = 1;

  if (error > 180 or error < -180) { // this solves the 360 region issue
    error *= -1;
  }

  if (error > 5) {
    currentSteering += maxRate;
  } else if (error < -5) {
    currentSteering -= maxRate;
  }

//  BTprintfloat(abs(prevError - error));
//  BTprintfloat(abs(prevError) - abs(error));
//  BTprintfloat(abs(abs(prevError) - abs(error)));
//  BTprintfloat(0);
//  BTprintfloat(prevError);
//  BTprintfloat(error);
//  BTprintfloat(0);

  
//  if (abs(abs(prevError) - abs(error)) < 40)
//  {
//    prevError = error;
//  }
//  else
//  {
//    error = prevError;
//  }

//  currentSteering = error;

//  BTprintfloat(error);
//
//  if (currentSteering < error) {
//    currentSteering += 1;
//  } else if (currentSteering > error) {
//    currentSteering -= 1;
//  }

//  if (error < 5 and error > -5) {
//    currentSteering = 0;
//  }

  
//   if (error < -10) {
//    // i.e. turn left
//    currentSteering = currentSteering - maxRate;
//   } else if (error > 10) { // turn right
//    currentSteering = currentSteering + maxRate;
//   } 
   
   if (currentSteering < -10) {
      currentSteering = -10;
     } else if (currentSteering > 10) {
      currentSteering = 10;
     }

   BTprintfloat(currentSteering);

   return currentSteering;
}
  
 

float normalizeAngle180(float angle) {
    while (angle < -180) 
    {
        angle += 360;
    }
    while (angle > 180) 
    {
        angle -= 360;
    }
    return angle;
}

float normalizeAngle360(float angle) {
    while (angle < 0) 
    {
        angle += 360;
    }
    while (angle > 360) 
    {
        angle -= 360;
    }
    return angle;
}
