//Code for testing the PID controller for the steering
//Programme ESP32 - then disconnect from computer and connect the ESP32 to the 9V battery
//Car waits until BT is connected
//Once connected IMU readings are taken for 2 seconds
//This is indicated as finished when the BT message is trasnmitted prompting the user to enter a 1 and the wheels twitch
//Once a 1 is entered, the car should drive on an angle offset defined by TARGET_ANGLE from its starting orientation
//Car should drive for 4 seconds before stopping

//Used to configure the kp, ki, kd parameters of the controller - defined in the setupPID() function in PID.cpp
//Can also configure how the PID control error singal is mapped to steering changes in carControl()

//Can check IMU calibration using Test_IMU_Reading.ino
//Can connect to serial monitor to check the control signal asnd angle corrections
//Can print values to BT serial monitor using BTprintfloat() or BTprintint()


#include "BT_Comms.h"
#include "AF_IMU.h"
#include "PID.h"
#include "AF_Scheduler.h"

#define TARGET_ANGLE 0            //Change to drive on different set points from starting direction
#define MOTOR_FORWARD_PIN 13
#define MOTOR_REVERSE_PIN 12
#define MOTOR_STEER_PIN 17
#define DRIVE_PWM_FREQ 10
#define STEER_PWM_FREQ 50
#define PID_INTERVAL 5            //Change to set duration in ms between PID updates

//Test Print Variables
#define PRINT_SPEED 200 // ms between prints
unsigned long lastPrint = 0; // Keep track of print time

enum State
{
    programme,
    idle,           //wait for start
    drive,          //moving towards target
};

enum State current_state = programme, next_state = programme;
bool start_flag = 0;
bool PID_flag = 1;

//Events
Time CurrentTime = {0,0};
Time IdleMode = {0, -1};
Time PIDUpdate = {0, -1};

void IRAM_ATTR Timer0_ISR()
{
  CurrentTime = increment(CurrentTime);

  if(IsScheduled(IdleMode)) 
  {
    next_state = idle;
    IdleMode.ms = -1;
    PIDUpdate.ms = -1;
    
  }

  if(IsScheduled(PIDUpdate))
  {
    PID_flag = 1;
    PIDUpdate = schedule(PID_INTERVAL);
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


//PID Variables
PIDConfig Steer_PID;
float control_signal;
float actual_yaw, desired_yaw;



void setup() 
{
  setupTimer();
  setupIMU();
  setupPID(&Steer_PID, P_Gain, I_Gain, D_Gain, I_Limit);
  setupDrive();
  setupBluetooth();
  Serial.begin(115200);
  IdleMode = schedule(2000);
}

void loop() 
{
      updateYaw();
  switch(current_state)
    {
      case programme:
      {
        actual_yaw= getYaw();
        desired_yaw = normalizeAngle360(actual_yaw + TARGET_ANGLE);

        
        Serial.print("Desired yaw: ");
        Serial.println(desired_yaw, 0);
        Serial.print("Actual yaw: ");
        Serial.println(actual_yaw, 0);
        Serial.println();
        Serial.println("Programme");      
        break;
      }
      
      case idle:
      {
//        if (millis() - lastPrint > PRINT_SPEED) 
//        {
//          Serial.print("IDLE");
//
//          Serial.println();
//          lastPrint = millis(); // Update lastPrint time
//        }


        Serial.print("Desired yaw: ");
        Serial.println(desired_yaw, 0);
        Serial.print("Actual yaw: ");
        Serial.println(actual_yaw, 0);
        Serial.println();

                     
        forward(0);
        reverse(0);
        delay(1000);
        steer(20);
        delay(1000);
                steer(-20);
        delay(1000); 
                steer(0);
        delay(1000);  
               
        start_flag = getBluetoothFlag();
        if(start_flag == 1)
        {
          next_state = drive;
          start_flag = 0;
          IdleMode = schedule(4000);
          PIDUpdate = schedule(PID_INTERVAL);
        }
        break;
      }
      
      case drive:
      {

      actual_yaw= getYaw();
      if( PID_flag == 1)
      {
//        BTprintfloat(actual_yaw);
        carControl(control_signal);
        control_signal = PID(&Steer_PID, desired_yaw, actual_yaw);
        PID_flag = 0;
      }
    
    if (millis() - lastPrint > PRINT_SPEED) 
    {
      Serial.print("Actual:   ");
      Serial.println(actual_yaw, 0);
      Serial.print("Desired:   ");
      Serial.println(desired_yaw, 0);
      Serial.print("Control Signal:   ");
      Serial.println(control_signal, 0);
      lastPrint = millis(); // Update lastPrint time
    }
      
        break;
      }
    }
  current_state = next_state;
}
