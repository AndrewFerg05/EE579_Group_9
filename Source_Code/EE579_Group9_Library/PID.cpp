#include "PID.h"
#include <cstdlib>
#include "BT_Comms.h"

float error = 0;
float derivative = 0;
float output = 0;
float setup_flag = 0;

//// TUNE kp, ki AND kd
void setupPID(PIDConfig *steer, int proportionalGain, int integralGain, int derivativeGain, float integralOverflow)
{
    steer->kp = 0.75;
    steer->ki = 0;
    steer->kd = 1;
    steer->overflow = integralOverflow;
    steer->previousError = 0;
    steer->integral = 0;
}
//
// float PID(PIDConfig* steer, float setPoint, float currentPoint)
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
int polarity = 0;
int prev_polarity=0;

float rateLimiter(int desiredAngle, int currentAngle, float currentSteering)
{

    int error = normalizeAngle180(desiredAngle - currentAngle);
    float maxRate = 0.5;

    if(error < 0 )
    {
        polarity = -1;
    }
    else
    {
        polarity = 1;
    }

    if(polarity != prev_polarity)
    {
        currentSteering = 0;
    }

    if (currentSteering < error)
    {
        currentSteering += maxRate;
    }
    else if (currentSteering > error)
    {
        currentSteering -= maxRate;
    }

    // currentSteering = error * 0.75;

    if (currentSteering < -10)
    {
        currentSteering = -10;
    }
    else if (currentSteering > 10)
    {
        currentSteering = 10;
    }

    if(currentSteering < 0 )
    {
        prev_polarity = -1;
    }
    else
    {
        prev_polarity = 1;
    }

    return currentSteering;
}

float normalizeAngle180(float angle)
{
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

float normalizeAngle360(float angle)
{
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
