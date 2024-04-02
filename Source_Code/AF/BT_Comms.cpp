#include "BT_Comms.h"
#include "BluetoothSerial.h"

BluetoothSerial serialBT;

void setupBluetooth() 
{
  serialBT.begin("ESP32-BT");

  //Wait For Connection
  while (!serialBT.connected(1000)) 
  {
    Serial.println("Waiting to Pair...");
    delay(100);
  }
  // Print a message once connected
  Serial.println("Bluetooth connected!");
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

  while (waitingForReading) 
  {
    if (serialBT.available()) {
      reading = serialBT.parseInt();

      while(true) 
      {
        // get rid of the 0
        if (serialBT.available()) 
        {
          int temp = serialBT.parseInt(); 
          break;
        }
      }

      if (measurementType == 'd') {
        if (reading <= 1000 and reading > 0)
        {
          waitingForReading = false;
        } else {
          serialBT.println("Incorrect Reading");
        }
      }

      else if (measurementType == 'a') {
        if (reading >= -90 and reading < 90) 
        {
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

int getBluetoothInputType() 
{
  bool waitingForReading = true;
  int reading = 0;

  serialBT.println("Target Type:");
  serialBT.println("0 = Target");
  serialBT.println("1 = Waypoint");

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
      if (reading >= 0 and reading < 2){
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

int getBluetoothNumberTargets()
{
    bool waitingForReading = true;
    int reading = 0;

    serialBT.println("Enter the numer of targets and waypoints");

    while(waitingForReading)
    {                   
        if(serialBT.available()) 
        {
            reading = serialBT.parseInt();
            Serial.println("Received Value: " + String(reading));

            while(true)
            {
                if (serialBT.available())
                {
                    int temp = serialBT.parseInt();
                    break;
                }
            }
            
            if (reading > 1 and reading < 4)
            {
                // Valid Input
                waitingForReading = false;
            } 
                else 
            {
                serialBT.print("Incorrect input ");
            }

        }

    delay(100);
    }
    return reading;
}

int getBluetoothFlag()
{
    bool waitingForReading = true;
    int reading = 0;

    serialBT.println("Enter 1 to Start");

    while(waitingForReading)
    {                   
        if(serialBT.available()) 
        {
            reading = serialBT.parseInt();
            Serial.println("Received Value: " + String(reading));

            while(true)
            {
                if (serialBT.available())
                {
                    int temp = serialBT.parseInt();
                    break;
                }
            }
            
            if (reading == 1)
            {
                // Valid Input
                waitingForReading = false;
            } 
                else 
            {
                serialBT.print("Invalid input");
            }

        }

    delay(100);
    }
    return reading;
}