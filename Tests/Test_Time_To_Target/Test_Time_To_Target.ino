//Used to test how long it takes the car to drive to a target to set the speed of the car
//Used to set car speed, how long before target it slows down, and how long it slows down for - tested in straight line

//Tuneable Parameters
//Target.h:
//car_speed - speed of car in m/s and 
//time_offset - how long before it reachs the target does start to slow down in ms 
//This file:
//slow_duration - how long it slows down for
//can also set slow down speed in this file

//Programme ESP32 - disconnect and plug it into the 9V batter
//Car waits for BT connection
//Once connected prompts the user to enter a distance
//The prompts user to input a 1 to start
//Will drive in a straight line - and stop within scanning range of spacified distance

//If doesn't work check PCB connections and battery voltage levels


#include "BT_Comms.h"
#include "Target.h"
#include "Scheduler.h"

#define slow_duration 500         //Set how long the car slows down when it reaches target in ms

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
    programme,
    idle,           //wait for start
    drive,          //moving towards target
    slow            //slow down
};

enum State current_state = programme, next_state = programme;
bool start_flag = 0;
bool PID_flag = 1;

//Events
Time CurrentTime = {0,0};
Time IdleMode = {0, -1};
Time DriveMode = {0, -1};
Time SlowMode = {0, -1};

void IRAM_ATTR Timer0_ISR()
{
  CurrentTime = increment(CurrentTime);

  if(IsScheduled(IdleMode)) 
  {
    next_state = idle;
    IdleMode.ms = -1; 
  }

  if(IsScheduled(DriveMode)) 
  {
    next_state = drive;
    DriveMode.ms = -1; 
  }
  
  if(IsScheduled(SlowMode)) 
  {
    next_state = slow;
    SlowMode.ms = -1;
    IdleMode = schedule(slow_duration);
  }

}


//Timer Pointer
hw_timer_t *Timer0 = NULL;

//Function Definitions
void setupTimer()
{
  Timer0 = timerBegin(0, 80, true);                   //Timer0, Predivider = 80 => 1MHz, Count Up
  timerAttachInterrupt(Timer0, &Timer0_ISR, true);    //Enable interrupt and attach ISR function
  timerAlarmWrite(Timer0, 1000, true);
  timerAlarmEnable(Timer0);
}

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

//    Serial.println(steering);
    if (steering < 0) 
    {
//        Serial.println("Lower");
        // Map the range from -20 to 0 to the duty cycle range of 71 to 82 with resolution of 6
        dutycycle = map(steering, -20, 0, 71, 82);
    } 
    else 
    {
//        Serial.println("upper");
        // Map the range from 0 to 20 to the duty cycle range of 82 to 88(maintaining the same value)
        dutycycle = map(steering, 0, 20, 82, 88);
    }
//    Serial.println(dutycycle);
    ledcWrite(2, dutycycle);
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
  ledcAttachPin(MOTOR_STEER_PIN, 2);      // Attach PWM channel 2 to steering servo pin
  ledcSetup(2, 50, 10);                   // Configure LEDC channel 2 with a frequency of 50Hz and a resolution of 8 bits
  ledcWrite(2, 82);                       // Default to Straight     
}

void carControl(float control_signal)
{

  Serial.print("Control Signal: ");
  Serial.println(control_signal, 0);
  
  int round_signal = constrain(round(control_signal), -60, 60);

  Serial.print("Rounded Signal: ");
  Serial.println(round_signal);
  
  int steering_angle = map(round_signal, -60, 60, -20, 20);

  Serial.print("Steering Angle: ");
  Serial.println(steering_angle);
  steer(steering_angle);
  
  if (steering_angle < 30 && steering_angle > -30) 
  {
        // Steering angle is within -30 and 30 degrees
        forward(100); // Move forward with speed 100
  } 
  else 
  {
        // Steering angle is not within -30 and 30 degrees
        forward(80); // Move forward with speed 80
  }
  
}

//{Ditance, anglefromstraight, timeToTarget, angleToTarget, isWaypoint}
Target Test_Target = {10.0, 0, 0, 0.0, false};

void setup() 
{
  setupTimer();
  setupDrive();
  setupBluetooth();
  Serial.begin(115200);
}

void loop() 
{
  switch(current_state)
    {
      case programme:
      {
              Test_Target.distance = getBluetoothReading(0, 'd') / 100;
              Serial.println("Distance: ");
              Serial.println(Test_Target.distance);
              calculateTimeAndAngle(&Test_Target);
              next_state = idle;
        break;
      }
      
      case idle:
      {
        Serial.println("IDLE");
        steer(0);
        reverse(0);
        forward(0);
        
        start_flag = getBluetoothFlag();
        if(start_flag == 1)
        {
          next_state = drive;
          start_flag = 0;
          SlowMode = schedule(Test_Target.timeToTarget);
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
        steer(0);  
        forward(100);       
        break;
      }

            
      case slow:
      {
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("Slow");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        steer(0);
        forward(0);
        reverse(50);            //Can set to drive in reverse or just stop 
        break;
      }
    }
  current_state = next_state;
}
