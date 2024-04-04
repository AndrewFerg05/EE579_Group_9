#ifndef PID_H_
#define PID_H_

// Tuneable Parameters
#define P_Gain 1
#define I_Gain 0
#define D_Gain 0
#define I_Limit 10000



// Struct Definitions
typedef struct PIDConfig{
    int ki;
    int kd;
    int kp;
    float overflow;
    float prevError;
    float integral;
}PIDConfig;

typedef struct servoConfig{
    int minMicro;
    int maxMicro;
    int minAngle;
    int maxAngle;
    float msPerDeg;
}servoConfig;

// Variable Definitions
extern float setup_flag; 

// Function Definiitions
extern void setupPID(PIDConfig*, int, int, int, float);
extern float PID(PIDConfig*, float, float);
extern void setupServo(servoConfig*, int, int, int, int);
extern int steeringMs(servoConfig*, float);

#endif
