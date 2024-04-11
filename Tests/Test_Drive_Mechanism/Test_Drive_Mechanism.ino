
#include "BT_Comms.h"
#define MOTOR_FORWARD_PIN 12
#define MOTOR_REVERSE_PIN 13
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

void forward(int dutyCyclePercentage) 
{
  // Convert duty cycle percentage to PWM duty cycle value (0-255)
  int dutyCycleValue = map(dutyCyclePercentage, 0, 100, 0, 255);
  
  ledcWrite(0, dutyCycleValue); // Set PWM duty cycle
}

void reverse(int dutyCyclePercentage) 
{
  // Convert duty cycle percentage to PWM duty cycle value (0-255)
  int dutyCycleValue = map(dutyCyclePercentage, 0, 100, 0, 255);
  
  ledcWrite(1, dutyCycleValue); // Set PWM duty cycle
}

void setupDrive()
{
  ledcAttachPin(MOTOR_REVERSE_PIN, 1); // Attach PWM channel 1 to reverse motor pin
  ledcSetup(1, DRIVE_PWM_FREQ, 8); // PWM frequency: 5000 Hz, PWM resolution: 8-bit (0-255)
  ledcWrite(1, 0); // Set PWM duty cycle
  
  ledcAttachPin(MOTOR_FORWARD_PIN, 0); // Attach PWM channel 0 to forward motor pin
  ledcSetup(0, DRIVE_PWM_FREQ, 8); // PWM frequency: 5000 Hz, PWM resolution: 8-bit (0-255)
  ledcWrite(0, 0); // Set PWM duty cycle 
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
