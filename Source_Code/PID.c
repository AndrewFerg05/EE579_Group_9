#include "PID.h"

void setupPID(PIDConfig* steer, int proportionalGain, int integralGain, int derivativeGain, float integralOverflow)
{
    steer->kp = proportionalGain;
    steer->ki = integralGain;
    steer->kd = derivativeGain;
    steer->overflow = integralOverflow;
    steer->prevError= 0;
}

float PID(PIDConfig* steer, float setPoint, float currentPoint)
{
    float error;
    float integral;
    float derivative;
    float output;
    
    // Calculate error
    error = setPoint - currentPoint;

    // Calcualte integral term
    integral = integral + error;

    // Calcualte derivative term
    derivative = error - steer->prevError;

    // Determine the output signal
    output = (steer->kp*error) + (steer->ki*integral) + (steer->kd*derivative);

    // Store the error for next interation
    steer->prevError = error;

    // if the integral gets out of hand reset to 0
    if (integral > steer->overflow)
    {
        integral = 0;
    }

    // return PID output signal
    return output;
}

void setupServo(servoConfig* limits, int minDegrees, int maxDegrees, int minMicroSeconds, int maxMicroSeconds)
{
    limits->minAngle = minDegrees;
    limits->maxAngle = maxDegrees;
    limits->minMicro = minMicroSeconds;
    limits->maxMicro = maxMicroSeconds;
}

int steeringMs(servoConfig* limits, float controlSignal)
{
    int microSeconds;

    // Check PID control signal has no exceeded limits of servo
    if (controlSignal > limits->maxAngle)
    {
        controlSignal = limits->maxAngle;
    }
    if (controlSignal < limits->minAngle)
    {
        controlSignal = limits->minAngle;
    }

    // re-map limits of servo to microSeconds
    microSeconds = (controlSignal-limits->minAngle)/(limits->maxAngle-limits->minAngle) * (limits->maxMicro - limits->minMicro) + limits->minMicro;

    // return microSeconds
    return microSeconds;
}