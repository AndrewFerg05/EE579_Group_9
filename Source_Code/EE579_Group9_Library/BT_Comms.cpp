#include "BT_Comms.h"
#include "BluetoothSerial.h"

BluetoothSerial serialBT;

void setupBluetooth()
{
  serialBT.begin("Bumpy");

  // Wait For Connection
  while (!serialBT.connected(1000))
  {
    Serial.println("Waiting to Pair...");
    delay(100);
  }

  // Print a message once connected
  Serial.println("Bluetooth connected!");
}

float getBluetoothReading(int n, char measurementType)
{

  bool waitingForReading = true;
  float reading = 0;

  if (measurementType == 'd')
  {
    serialBT.print("Enter distance to target ");
    serialBT.print(n);
    serialBT.println(" in metres.");
  }
  else if (measurementType == 'a')
  {
    serialBT.print("Enter angle to target ");
    serialBT.print(n);
    serialBT.println(". Use -ve for targets to the left.");
  }

  while (waitingForReading)
  {
    if (serialBT.available())
    {
      reading = serialBT.parseInt();
      Serial.println("Reading: ");
      Serial.print(reading);
      while (true)
      {
        // get rid of the 0
        if (serialBT.available())
        {
          int temp = serialBT.parseInt();
          break;
        }
      }

      if (measurementType == 'd')
      {
        if (reading <= 1000 and reading > 0)
        {
          waitingForReading = false;
        }
        else
        {
          serialBT.println("Incorrect Reading");
        }
      }

      else if (measurementType == 'a')
      {
        if (reading >= -180 and reading < 180)
        {
          waitingForReading = false;
        }
        else
        {
          serialBT.println("Incorrect Reading");
        }
      }
    }
    delay(100);
  }

  Serial.println("Reading: ");
  Serial.print(reading);
  return reading;
}

int getBluetoothInputType()
{
  bool waitingForReading = true;
  int reading = 0;

  serialBT.println("Target Type:");
  serialBT.println("0 = Can");
  serialBT.println("1 = Waypoint");

  while (waitingForReading)
  {
    if (serialBT.available())
    {
      reading = serialBT.parseInt();
      Serial.println("Received Value: " + String(reading));
      while (true)
      {
        // get rid of the 0
        if (serialBT.available())
        {
          int temp = serialBT.parseInt();
          break;
        }
      }
      if (reading >= 0 and reading < 2)
      {
        // correct reading
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

int getBluetoothNumberTargets()
{
  bool waitingForReading = true;
  int reading = 0;

  serialBT.println("Enter the numer of targets and waypoints");

  while (waitingForReading)
  {
    if (serialBT.available())
    {
      reading = serialBT.parseInt();
      Serial.println("Received Value: " + String(reading));

      while (true)
      {
        if (serialBT.available())
        {
          int temp = serialBT.parseInt();
          break;
        }
      }

      if (reading > 0 and reading < 4)
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

  while (waitingForReading)
  {
    if (serialBT.available())
    {
      reading = serialBT.parseInt();
      Serial.println("Received Value: " + String(reading));

      while (true)
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

void BTprintint(int a)
{
  serialBT.print(a);
}

void BTprintError(int a)
{
  serialBT.print("Error ");
  serialBT.println(a);
}

void BTprintfloat(float a)
{
  serialBT.println(a, 0);
}
