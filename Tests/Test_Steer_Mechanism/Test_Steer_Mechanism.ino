
#include "BT_Comms.h"
#define MOTOR_FORWARD_PIN 12
#define MOTOR_REVERSE_PIN 13
#define MOTOR_STEER_PIN 5
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

void steer(int steering) 
{
    steering = constrain(steering, -20, 20);
    int dutycycle; // declare dutycycle here

    Serial.println(steering);
    if (steering < 0) 
    {
        Serial.println("Lower");
        // Map the range from -20 to 0 to the duty cycle range of 71 to 82 with resolution of 6
        dutycycle = map(steering, -20, 0, 71, 82);
    } 
    else 
    {
        Serial.println("upper");
        // Map the range from 0 to 20 to the duty cycle range of 82 to 88(maintaining the same value)
        dutycycle = map(steering, 0, 20, 82, 88);
    }
    Serial.println(dutycycle);
    ledcWrite(3, dutycycle);
}
  


void setupDrive()
{
//  Reverse
  ledcAttachPin(MOTOR_REVERSE_PIN, 1);    // Attach PWM channel 1 to reverse motor pin
  ledcSetup(1, DRIVE_PWM_FREQ, 8);        // PWM frequency: 10 Hz, PWM resolution: 8-bit (0-255)
  ledcWrite(1, 0);                        // Set PWM duty cycle
//  Forward
  ledcAttachPin(MOTOR_FORWARD_PIN, 0);    // Attach PWM channel 0 to forward motor pin
  ledcSetup(0, DRIVE_PWM_FREQ, 8);        // PWM frequency: 10 Hz, PWM resolution: 8-bit (0-255)
  ledcWrite(0, 0);                        // Set PWM duty cycle 
//  Steer
  ledcAttachPin(MOTOR_STEER_PIN, 3);      // Attach PWM channel 2 to steering servo pin
  ledcSetup(3, 50, 10);                   // Configure LEDC channel 2 with a frequency of 50Hz and a resolution of 8 bits
  ledcWrite(3, 82);                       // Default to Straight     
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
        delay(1000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(1000);

        steer(-10);
        forward(100);
        delay(1000);
        forward(0);
        delay(1000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(1000);
                
        steer(0);
        forward(100);
        delay(1000);
        forward(0);
        delay(1000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(1000);
        
        steer(10);
        forward(100);
        delay(1000);
        forward(0);
        delay(1000);
        reverse(100);
        delay(1000);
        reverse(0);
        delay(1000);

        steer(20);
        forward(100);
        delay(1000);
        forward(0);
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
