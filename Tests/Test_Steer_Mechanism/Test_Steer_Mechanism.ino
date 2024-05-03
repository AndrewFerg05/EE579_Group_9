//Used to test the steer mechanism of the car
//Programme ESP32 - then disconnect from computer and connect the ESP32 to the 9V battery
//Car waits while phone connects to the ESP32 via Bluetooth (BT)
//Once BT connection established BT message will be transmitted to the phone asking the user to enter a 1
//Once a 1 has been received, the car should then...
//Drive forwards at full lock left for 1 second
//Drive backwrads at full lock left for 1 second
//Drive forwards at half lock left for 1 second
//Drive backwrads at half lock left for 1 second
//Drive forwards straight for 1 second
//Drive backwrads straight for 1 second
//Drive forwards at half lock right for 1 second
//Drive backwrads at half lock right for 1 second
//Drive forwards at full lock right for 1 second
//Drive backwrads at full lock right for 1 second
//Stop

//If the car does not do this check motor conections, servo conections and battery voltage levels

//Connections
// ESP32 GPIO12   - IN4 Motor Driver
// ESP32 GPIO13   - IN3 Motor Driver
// ESP32 3V3      - EEP Motor Driver
// 4.5V           - VCC Motor Driver
// GND            - GND Motor Driver
// Motor Forward  - OUT3 Motor Driver
// Motor Reverse  - OUT4 Motor Driver

// ESP32 GPIO17   - Servo input
// 4.5V           - Servo Vin
// GND            - Servo GND

// NB. Above connections are for the custom PCB
// If using the veroboard prototype
// ESP32 GPIO13 is reverse
// ESP32 GPIO12 is forward
// so swap MOTOR_FORWARD_PIN and MOTOR_REVERSE_PIN below accordingly
// ESP32 GPIO5   - Servo input
// so change MOTOR_STEER_PIN ccordingly

// 13/04/24 - Known Issue: Mechianical steering issue means half lock left will be similar to full lock

#include "BT_Comms.h"
#include "CarControl.h"
#define MOTOR_FORWARD_PIN 13
#define MOTOR_REVERSE_PIN 12
#define MOTOR_STEER_PIN 17
#define DRIVE_PWM_FREQ 10
#define STEER_PWM_FREQ 50


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

void test_steer(int steering)
{
  // Constrain steering angle input
//  steering = constrain(steering, -20, 20);
//
//  int dutycycle;
//  //  Straight = Duty Cycle 82
//  //  Full lock left = Duty Cycle 71
//  //  Full lock left = Duty Cycle 88
//
//  if (steering < 0)
//  {
//    // Map the range from -20 to 0 to the duty cycle range of 71 to 82 - full left steerin range
//    dutycycle = map(steering, -20, 0, 71, 82);
//  }
//  else
//  {
//    // Map the range from 0 to 20 to the duty cycle range of 82 to 88 - full right steering range
//    dutycycle = map(steering, 0, 20, 82, 88);
//  }
  ledcWrite(3, steering); // Update servo PWM duty cycle
}

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

        steer(-20);
        forward(100);
        delay(1000);
        forward(0);
        delay(2000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(2000);

        steer(-10);
        forward(100);
        delay(1000);
        forward(0);
        delay(2000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(2000);
                
        steer(0);
        forward(100);
        delay(2000);
        forward(0);
        delay(4000);
        reverse(100);
        delay(2000);
        reverse(0);
        delay(4000);
        
        steer(10);
        forward(100);
        delay(1000);
        forward(0);
        delay(2000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(2000);

        steer(20);
        forward(100);
        delay(1000);
        forward(0);
        delay(2000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(2000);
        next_state = idle;
        break;
      }
    }
  current_state = next_state;
}
