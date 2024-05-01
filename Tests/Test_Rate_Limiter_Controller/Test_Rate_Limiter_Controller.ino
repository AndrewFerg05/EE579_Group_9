//Code for testing the Rate Limiter controller for the steering
//Programme ESP32 - then disconnect from computer and connect the ESP32 to the 9V battery
//Car waits until BT is connected
//Once connected IMU readings are taken for 2 seconds
//This is indicated as finished when the BT message is trasnmitted prompting the user to enter a 1 and the wheels twitch
//Once a 1 is entered, the car should drive on an angle offset defined by TARGET_ANGLE from its starting orientation
//Car should drive for 4 seconds before stopping


//Can check IMU calibration using Test_IMU_Reading.ino
//Can connect to serial monitor to check the control signal asnd angle corrections
//Can print values to BT serial monitor using BTprintfloat() or BTprintint()


#include "BT_Comms.h"
#include "IMU.h"
#include "PID.h"
#include "Scheduler.h"
#include "CarControl.h"

#define TARGET_ANGLE 0            //Change to drive on different set points from starting direction
#define MOTOR_FORWARD_PIN 13
#define MOTOR_REVERSE_PIN 12
#define MOTOR_STEER_PIN 17
#define DRIVE_PWM_FREQ 10
#define STEER_PWM_FREQ 50
#define RATE_INTERVAL 1            //Change to set duration in ms between Rate updates

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
bool Rate_flag = 1;

//Events
Time CurrentTime = {0,0};
Time IdleMode = {0, -1};
Time RateUpdate = {0, -1};

void IRAM_ATTR Timer0_ISR()
{
  CurrentTime = increment(CurrentTime);

  if(IsScheduled(IdleMode)) 
  {
    next_state = idle;
    IdleMode.ms = -1;
    RateUpdate.ms = -1;
    
  }

  if(IsScheduled(RateUpdate))
  {
    Rate_flag = 1;
    RateUpdate = schedule(RATE_INTERVAL);
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


float control_signal;
float actual_yaw, desired_yaw;



void setup() 
{
  setupTimer();
  setupIMU();
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

        BTprintfloat(desired_yaw);
//        BTprintfloat(1000000001);

        
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
          IdleMode = schedule(8000);
          RateUpdate = schedule(RATE_INTERVAL);
        }
        break;
      }
      
      case drive:
      {

      actual_yaw= getYaw();
      if( Rate_flag == 1)
      {
        control_signal = rateLimiter(desired_yaw, actual_yaw, control_signal);
        forward(50);
        steer(control_signal);
        if (millis() - lastPrint > PRINT_SPEED) 
          {
//            BTprintfloat(actual_yaw);
            BTprintfloat(actual_yaw);
//            BTprintfloat(100000000000000001);
            lastPrint = millis(); // Update lastPrint time
          }

        Rate_flag = 0;
      }
    
//    if (millis() - lastPrint > PRINT_SPEED) 
//    {
//      Serial.print("Actual:   ");
//      Serial.println(actual_yaw, 0);
//      Serial.print("Desired:   ");
//      Serial.println(desired_yaw, 0);
//      Serial.print("Control Signal:   ");
//      Serial.println(control_signal, 0);
//      lastPrint = millis(); // Update lastPrint time
//    }
      
        break;
      }
    }
  current_state = next_state;
}
