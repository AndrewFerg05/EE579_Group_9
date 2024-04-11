#include "AF_IMU.h"

//Variables
float actual_yaw = 0;

//Test Print Variables
#define PRINT_SPEED 200 // ms between prints
unsigned long lastPrint = 0; // Keep track of print time

void setup() 
{
  Serial.begin(115200);
  while (!Serial); //wait for connection
  setupIMU();

}

void loop() {
    updateYaw();
    if (millis() - lastPrint > PRINT_SPEED) 
    {
      actual_yaw= getYaw();
      Serial.print(actual_yaw, 0);
      Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }

}
