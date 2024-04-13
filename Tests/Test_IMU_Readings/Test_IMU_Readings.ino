
//Code to test the IMU (Adafruit ICM 20948 sensor)
//Requires the Sparkfun ICM_90248 Arduino library
//Modified version of J. Remington Mahony AHRS 3D Fusion Filter - https://github.com/jremington/ICM_20948-AHRS
//Which has been modified and repackaged into the library for this project

//Code will print out headings (000 being North) to the serial monitor
//Rotate the sensor through 360 degrees
//Ensure the output moves smoothly through the 360 degree range
//If the output does not do this the sensor needs recallibrated

//Connections
//ESP32 3V3 - Vin IMU
//ESP32 GND - GND IMU
//ESP32 GPIO22 - SCL IMU
//ESP32 GPIO21 - SDA IMU

//13/04/24 - Known Issue - the average yaw returned by getYaw() does not transition smoothly from 360 degrees back to 0 degrees


#include "AF_IMU.h"

//Variables
float actual_yaw = 0;

//Test Print Variables
#define PRINT_SPEED 200       // ms between prints
unsigned long lastPrint = 0;  // Keep track of print time

void setup() 
{
  Serial.begin(115200);
  while (!Serial);            //wait for connection
  setupIMU();

}

void loop() {
    updateYaw();               //Update quaternion matrix
    
    if (millis() - lastPrint > PRINT_SPEED) 
    {
      actual_yaw= getYaw();           //Get current yaw
      Serial.print(actual_yaw, 0);    //Print current yaw
      Serial.println();
      lastPrint = millis();           // Update lastPrint time
    }

}
