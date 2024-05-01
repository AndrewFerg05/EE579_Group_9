#include "IMU.h"
#include <Wire.h>
#include "ICM_20948.h"

const float gyro_scale = DEG_TO_RAD; // Conversion factor for gyroscope data (assuming degrees to radians)
const float accel_scale = 9.81 / 1024.0f; // Conversion factor for accelerometer data (assuming G-force to m/s²)

float yaw = 0.0f;  // Initial yaw angle
float yaw_rate = 0.0f;  // Gyroscope reading
float complementary_filter_coeff = 0.95f;  // Filter coefficient (adjust between 0.9 and 0.99 for best results)
unsigned long last_update_time = 0;  // Time of the last sensor reading

ICM_20948 imu;

//Setup
void setupIMU()
{
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  imu.begin(WIRE_PORT, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) 
  {
    Serial.println(F("ICM_90248 not detected"));
    while (1);
  }
  last_update_time = millis();  // Initialize time variable
}

float getYaw(){
  unsigned long current_time = millis();
  float dt = (float)(current_time - last_update_time) / 1000.0f;  // Time elapsed in seconds
  last_update_time = current_time;  // Update time for next iteration

  icm.readGyro();  // Read gyroscope data
  yaw_rate = icm.calcGyroX() * gyro_scale;  // Extract and scale x-axis gyroscope data for yaw

  // Read accelerometer data for tilt compensation
  float accel_x = icm.calcAccelX() * accel_scale;
  float accel_y = icm.calcAccelY() * accel_scale;

  // Calculate tilt angle (assuming sensor is oriented flat)
  float tilt_angle = atan2(accel_y, accel_x);

  // Apply complementary filter with time integration
  float filtered_yaw = complementary_filter_coeff * (yaw + yaw_rate * dt) + (1.0f - complementary_filter_coeff) * tilt_angle;

  // Update yaw angle (be mindful of overflow)
  yaw = filtered_yaw;

  Serial.print("Yaw: ");
  Serial.println(yaw, RAD_TO_DEG);
}
