#include "math.h"
#include "ICM_20948.h"


#ifndef IMU_H_
#define IMU_H_
#define PI 3.14159265
#define WIRE_PORT Wire 
#define AD0_VAL 1  
#include <Wire.h>
#include <ICM_20948.h>

//const float gyro_scale = DEG_TO_RAD; // Conversion factor for gyroscope data (assuming degrees to radians)
//const float accel_scale = G / 1024.0f; // Conversion factor for accelerometer data (assuming G-force to m/s²)

//float yaw = 0.0f;  // Initial yaw angle
//float yaw_rate = 0.0f;  // Gyroscope reading
//float complementary_filter_coeff = 0.95f;  // Filter coefficient (adjust between 0.9 and 0.99 for best results)
//unsigned long last_update_time = 0;  // Time of the last sensor reading


//Setup
void setupIMU();

float getYaw(){};


#endif
