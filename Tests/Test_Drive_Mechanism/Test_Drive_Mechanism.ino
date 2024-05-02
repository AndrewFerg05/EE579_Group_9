//Used to test the drive mechanism of the car
//Programme ESP32 - then disconnect from computer and connect the ESP32 to the 9V battery
//Car waits while phone connects to the ESP32 via Bluetooth (BT)
//Once BT connection established BT message will be transmitted to the phone asking the user to enter a 1
//Once a 1 has been received, the car should then...
//Drive forwards at 50% speed for 1 second
//Drive forward at 100% speed for 1 second
//Drive backwards at 50% speed for 1 second
//Drive backwards at 100% speed for 1 second
//Stop

//If the car does not do this check motor conections and battery voltage levels

//Connections
// ESP32 GPIO12   - IN4 Motor Driver
// ESP32 GPIO13   - IN3 Motor Driver
// ESP32 3V3      - EEP Motor Driver
// 4.5V           - VCC Motor Driver
// GND            - GND Motor Driver
// Motor Forward  - OUT3 Motor Driver
// Motor Reverse  - OUT4 Motor Driver

// NB. Above connections are for the custom PCB
// If using the veroboard prototype
// ESP32 GPIO13 is reverse
// ESP32 GPIO12 is forward
// so swap MOTOR_FORWARD_PIN and MOTOR_REVERSE_PIN below accordingly



#include "BT_Comms.h" 
#include "CarControl.h"

#define MOTOR_FORWARD_PIN 13
#define MOTOR_REVERSE_PIN 12
#define DRIVE_PWM_FREQ 10

//Test Print Variables
#define PRINT_SPEED 200 // ms between prints
unsigned long lastPrint = 0; // Keep track of print time

enum State
{
    idle,           //wait for start
    drive,          //moving towards target
};

enum State current_state = idle, next_state = idle;
bool start_flag = 0;


void setup() 
{
  setupDrive();
  setupBluetooth();
  Serial.begin(115200);
}

void loop() 
{
  switch(current_state)
    {
      case idle:
      {
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("IDLE");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        
        start_flag = getBluetoothFlag();
        if(start_flag == 1)
        {
          next_state = drive;
          start_flag = 0;
          
        }
        break;
      }
      
      case drive:
      {
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("Drive");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
 
        delay(1000);
        forward(50);
        delay(1000);
        forward(100);
        delay(1000);
        forward(0);
        delay(1000);
        reverse(50);
        delay(1000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(1000);

        next_state = idle;
        break;
      }
    }
  current_state = next_state;
}
