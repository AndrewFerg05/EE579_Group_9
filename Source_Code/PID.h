#include "main.h"

typedef struct PIDConfig{
    int ki;
    int kd;
    int kp;
    float overflow;
    float prevError;
}PIDConfig;

// Store servo configuration 
typedef struct servoConfig{
    int minMicro;
    int maxMicro;
    int minAngle;
    int maxAngle;
}servoConfig;