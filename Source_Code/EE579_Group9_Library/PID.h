#ifndef PID_H_
#define PID_H_

// Tuneable Parameters
#define I_Limit 10000

// These are overwritten in setup file but should be left here
#define P_Gain 0
#define I_Gain 0
#define D_Gain 0 
// set kp, ki, and kd parameters in PID.cpp



// Struct Definitions
typedef struct PIDConfig{
    float ki;
    float kd;
    float kp;
    float overflow;
    float prevError;
    float integral;
}PIDConfig;

<<<<<<< HEAD:Source_Code/EE579_Group9_Library/PID.h
=======
typedef struct servoConfig{
    int minMicro;
    int maxMicro;
    int minAngle;
    int maxAngle;
    float msPerDeg;
}servoConfig;
>>>>>>> main:Source_Code/AF/PID.h

// Variable Definitions
extern float setup_flag; 

// Function Definiitions
extern void setupPID(PIDConfig*, int, int, int, float);
extern float PID(PIDConfig*, float, float);
<<<<<<< HEAD:Source_Code/EE579_Group9_Library/PID.h
extern float normalizeAngle180(float);
extern float normalizeAngle360(float);
#endif
=======
extern void setupServo(servoConfig*, int, int, int, int);
extern int deg2dc(servoConfig*, float);

#endif
>>>>>>> main:Source_Code/AF/PID.h
