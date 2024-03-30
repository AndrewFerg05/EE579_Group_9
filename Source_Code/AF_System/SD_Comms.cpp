#include "SD_Comms.h"
#include "BluetoothSerial.h"

BluetoothSerial serialBT;


void setupBluetooth() {
  serialBT.begin("ESP32-BT");
  while (!serialBT.connected(1000)) {
    delay(100);
  }
  
  // Print a message once connected
  Serial.println("Bluetooth connected!");
 }


int getBluetoothNextStep() {
  
  bool waitingForReading = true;
  int reading = 0;

  serialBT.println("Enter Next Step:");
  serialBT.println("0 = Add Target");
  serialBT.println("1 = Add Navigation Point");
  serialBT.println("2 = Prepare to Drive");

  while (waitingForReading) {
    if (serialBT.available()) {
      reading = serialBT.parseInt();
      Serial.println("Received Value: " + String(reading));
      while(true) {
        // get rid of the 0
        if (serialBT.available()) {
          int temp = serialBT.parseInt(); 
          break;
        }
      }
      if (reading >= 0 and reading <= 2){
        // correct reading
        waitingForReading = false;
      } else {
          serialBT.print("Incorrect input ");
       }
     }
    
    delay(100);
   }
  return reading;
}


int getBluetoothReading(int n, char measurementType) {
  
  bool waitingForReading = true;
  int reading = 0;

  if (measurementType == 'd') {
    serialBT.print("Enter distance to target ");
    serialBT.print(n);
    serialBT.println(" in cm. Use exactly 3 integers.");
  }
  else if (measurementType == 'a') {
    serialBT.print("Enter angle to target ");
    serialBT.print(n);
    serialBT.println(" in cm. Use exactly 2 integers, plus '-' for negative values.");
  }

  while (waitingForReading) {
    if (serialBT.available()) {
      reading = serialBT.parseInt();

      while(true) {
        // get rid of the 0
        if (serialBT.available()) {
          int temp = serialBT.parseInt(); 
          break;
        }
      }

      if (measurementType == 'd') {
        if (reading <= 1000 and reading > 0){
          waitingForReading = false;
        } else {
          serialBT.println("Incorrect Reading");
        }
      }

      else if (measurementType == 'a') {
        if (reading >= -90 and reading < 90) {
          waitingForReading = false;
        } else {
          serialBT.println("Incorrect Reading");
        }
        
      }
    }
    delay(100);
  }
    
    
return reading;
}
