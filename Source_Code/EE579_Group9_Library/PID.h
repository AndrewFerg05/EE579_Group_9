#ifndef PID_H_
#define PID_H_

// Tuneable Parameters
#define I_Limit 10000

// These are overwritten in setup file but should be left here
#define P_Gain 0
#define I_Gain 0
#define D_Gain 0 
// set kp, ki, and kd parameters in PID.cpp
extern int prevError;

// Struct Definitions
typedef struct PIDConfig{
    float ki;
    float kd;
    float kp;
    float overflow;
    float previousError;
    float integral;
}PIDConfig;


// Variable Definitions
extern float setup_flag; 

// Function Definiitions
extern void setupPID(PIDConfig*, int, int, int, float);
//extern float PID(PIDConfig*, float, float);
extern float rateLimiter(int, int, float);
extern float normalizeAngle180(float);
extern float normalizeAngle360(float);

#endif
