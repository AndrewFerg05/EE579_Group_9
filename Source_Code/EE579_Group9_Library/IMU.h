#include "math.h"
#include "ICM_20948.h"


#ifndef IMU_H_
#define IMU_H_
#define PI 3.14159265
#define WIRE_PORT Wire 
#define AD0_VAL 1  



//Variables
extern ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;


//Functions
extern void setupIMU();
extern float vector_dot(const float a[3], const float b[3]);
extern void vector_normalize(float a[3]);
extern void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
extern void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]);
extern void updateYaw();
extern float getYaw(); 
#endif