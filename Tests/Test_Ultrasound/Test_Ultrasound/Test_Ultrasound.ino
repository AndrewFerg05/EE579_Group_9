
//Code to test the Ultrasound (HC-SR04) & corresponding servo
//Requires 

// When started, the ultrasound will scan through 180 degress, taking 180 distance readings
// It then processes the data and returns the smallest target object
// To process the data:
  // 1) Run through the array and find the smallest distance value
  // 2) Iterate through adjacent readings to find the width of the target (with a tolerance of +- 5 cm)
  // 3) Iterate through the width to find the depth of the target
  // 4) If the target is smaller, wider or too deep: it is either not a can or it is a false reading
  // 5) So, set a new lower boundary and repeat steps 1 - 4
  // 6) If there is no valid target in a scan, re-scan and re-process. 
  // 7) If no can is detected after 5 scans, return an error code


//Connections
//ESP32 3V3 - Vin Ultrasound
//ESP32 GND - GND Ultrasound
//ESP32 3V3 - Vin Servo
//ESP32 GND - GND Servo
//ESP32 GPIO02 - Echo Ultrasound 
//ESP32 GPIO04 - Trigger Ultrasound 
//ESP32 GPIO16 / RX2 - Servo Control 


//13/04/24 - Known Issue - the average yaw returned by getYaw() does not transition smoothly from 360 degrees back to 0 degrees


#include "Target.h"

//Variables

//Test Print Variables
#define PRINT_SPEED 200       // ms between prints
unsigned long lastPrint = 0;  // Keep track of print time

void setup() 
{
  Serial.begin(9600);
  while(!Serial);
}

void loop() {
  Target myTarget = scanForTargets_Ultrasound();
  
  while(1){
//    Serial.println("COMPLETE");
    delay(500);
  }
}
