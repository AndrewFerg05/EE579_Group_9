#include "PID.h"

float error = 0;
float derivative = 0;
float output = 0;
float setup_flag = 0;

void setupPID(PIDConfig* steer, int proportionalGain, int integralGain, int derivativeGain, float integralOverflow)
{
    steer->kp = proportionalGain;
    steer->ki = integralGain;
    steer->kd = derivativeGain;
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
        steer->prevError = setPoint - currentPoint;
    }

    // Calculate Error
    error = setPoint - currentPoint;

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



void setupServo(servoConfig* limits, int minDegLimit, int maxDegLimit, int MicroSecondsStart, int MicroSecondsEnd)
{
    limits->minAngle = minDegLimit;
    limits->maxAngle = maxDegLimit;
    limits->minMicro = MicroSecondsStart;
    limits->maxMicro = MicroSecondsEnd;
    limits->msPerDeg = (limits->maxMicro - limits->minMicro) / 180.00;
}

int deg2dc(servoConfig* limits, float controlSignal, int PWMfreq)
{
    // Check PID control signal has not exceeded limits of servo
    if (controlSignal > limits->maxAngle)
    {
        controlSignal = limits->maxAngle;
    }
    if (controlSignal < limits->minAngle)
    {
        controlSignal = limits->minAngle;
    }

    // Period in microseconds
    period = (1 / PWMfreq) * 1000000)

    // Pulse width in micro seconds
    pulseWidth = limits->minMicro + limits->msPerDeg * controlSignal

    // duty cycle expressed as a percentage
    int dutyCycle = (pulseWidth / period) * 100;

    // return microSeconds
    return dutyCycle;
}