#include "PID.h"

float error = 0;
float derivative = 0;
float output = 0;
float setup_flag = 0;


// TUNE kp, ki AND kd
void setupPID(PIDConfig* steer, int proportionalGain, int integralGain, int derivativeGain, float integralOverflow)
{
    steer->kp = 0.75;
    steer->ki = 0;
    steer->kd = 1;
    steer->overflow = integralOverflow;
    steer->prevError = 0;
    steer->integral = 0;
}

float PID(PIDConfig* steer, float setPoint, float currentPoint)
{
    // Initialise Previous Error
    if (setup_flag == 0)
    {
        setup_flag = 1;
        steer->prevError = normalizeAngle180(setPoint - currentPoint);
    }

    // Calculate Error
    error = normalizeAngle180(setPoint - currentPoint);

    // Calcualte Derivative Term
    derivative = error - steer->prevError;

    // Determine Output
    output = (steer->kp * error) + (steer->ki * steer->integral) + (steer->kd * derivative);

    // Store Preious Error and Integral Error
    steer->integral += error;
    steer->prevError = error;


    // If Integral Exceed Overflow Reset
    if (steer->integral > steer->overflow)
    {
        steer->integral = 0;
    }

    return output;
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
