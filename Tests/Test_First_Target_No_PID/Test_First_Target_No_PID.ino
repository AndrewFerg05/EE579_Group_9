//Test file for hitting one can without PID
//Programme ESP32 - Then disconnect from laptop and connect to 9V battery on car
// Waits for BT connection
//Prompts User to enter 0 for target , 1 for waypoint - Enter 0 for this test
//Prompts user to enter distance in cm
//Enter distance to can instraight line in front
//Should drive to near that can and scan
//If target detected to right should move right a bit and rescan
//Vice versa for left
//If can detected straight in front, drives straight and returns to idle
//Rescans and adjusts 10x before returning to idle or untill can is detected straight in front


#include "Scheduler.h"
#include "Target.h"
#include "BT_Comms.h"


#define slow_duration 500
#define targetHit_duration 10000
#define EndProtocol_duration 100000
#define SystemEnd_duration 120000

#define MOTOR_FORWARD_PIN 13
#define MOTOR_REVERSE_PIN 12
#define MOTOR_STEER_PIN 17
#define DRIVE_PWM_FREQ 10
#define STEER_PWM_FREQ 50


//Test Print Variables
#define PRINT_SPEED 200 // ms between prints
unsigned long lastPrint = 0; // Keep track of print time

//Variables
float actual_yaw = 0;
float straight_yaw = 0;
bool programme_ready_flag = 0;
bool start_flag = 0;
bool PID_flag = 1;
int target_select = 0;
int target_counter = 0;
Target First_Target;
Target End_Target;
Target closestTarget;

//PID Variables
float control_signal;


//State Machine Variables
enum State
{
    programme,      //waiting input variables
    idle,           //wait for start
    drive,          //moving towards target
    slow,           //slowing down when nearer target
    target,         //sensing
    stabilize_IMU,  //collect IMU readings
    hit,            //hit target
};

enum State current_state = programme, next_state = programme;

//Events
Time CurrentTime = {0,0};
Time IdleMode = {0, -1};
Time ProgrammeReady = {2,0};
Time DriveMode = {0, -1};
Time SlowMode = {0, -1};
Time TargetMode = {0, -1};
Time StartFlag = {0, -1};
Time HitMode = {0,-1};

//Timer Pointer
hw_timer_t *Timer0 = NULL;


//Interrupts
//Scheduler Interrupt - Clocked @ 1ms - handles state transitions
void IRAM_ATTR Timer0_ISR()
{
  CurrentTime = increment(CurrentTime);

  if(IsScheduled(ProgrammeReady))
  {
    ProgrammeReady.ms = -1;
    programme_ready_flag = 1;
  }

  

  if(IsScheduled(SlowMode)) 
  {
    next_state = slow;
    SlowMode.ms = -1;
    TargetMode = schedule(slow_duration);    
  }

  
  if(IsScheduled(TargetMode)) 
  {
    next_state = target;
    TargetMode.ms = -1;
  }
  
  
  if(IsScheduled(HitMode)) 
  {
    HitMode.ms = -1;
    next_state = hit;

    target_counter++;
    
    if(target_counter < 10)                 //After 10 attempts moves on
    {
      TargetMode = schedule(500);                           //Move a bit and rescan - CAN CHANGE SCHEDULE VALUE TO CHANGE HOW LONG IT MOVES FOR
    }
    else
    {
      IdleMode = schedule(500);
    }
  }
  
  
  if(IsScheduled(IdleMode)) 
  {
    next_state = idle;
    IdleMode.ms = -1;
    DriveMode.ms = -1;
    SlowMode.ms = -1;
    TargetMode.ms = -1;
  }

}

//Function Definitions
void setupTimer()
{
  Timer0 = timerBegin(0, 80, true);                   //Timer0, Predivider = 80 => 1MHz, Count Up
  timerAttachInterrupt(Timer0, &Timer0_ISR, true);    //Enable interrupt and attach ISR function
  timerAlarmWrite(Timer0, 1000, true);
  timerAlarmEnable(Timer0);
}

//Drive Functions
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
        // Map the range from -20 to 0 to the duty cycle range of 71 to 82 with resolution of 6
        dutycycle = map(steering, -20, 0, 71, 82);
    } 
    else 
    {
        // Map the range from 0 to 20 to the duty cycle range of 82 to 88(maintaining the same value)
        dutycycle = map(steering, 0, 20, 82, 88);
    }
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
  //Constrain PID error and map to steering range
  int round_signal = constrain(round(control_signal), -60, 60);
  int steering_angle = map(round_signal, -60, 60, -20, 20);
  steer(steering_angle);
  
  if (steering_angle < 30 && steering_angle > -30) 
  {
        forward(100); // Move forward with speed 100
  } 
  else 
  {
        forward(80); // Move forward with speed 80
  }

//  if (millis() - lastPrint > PRINT_SPEED) 
//  {
//    Serial.print("Control Signal: ");
//    Serial.println(control_signal, 0);
//    Serial.print("Rounded Signal: ");
//    Serial.println(round_signal);
//    Serial.print("Steering Angle: ");
//    Serial.println(steering_angle);    
//    Serial.println();
//    lastPrint = millis(); // Update lastPrint time
//  }
  
}


//Main
void setup()
{
  Serial.begin(115200);
  setupDrive();
  setupTimer();
  setupBluetooth();
}


void loop()
{
  
    switch(current_state)
    {
      case programme:
      {
        if(programme_ready_flag == 1)
        {
            //Move wheels back and forth
            forward(0);
            reverse(0);
            delay(1000);
            steer(20);
            delay(1000);
            steer(-20);
            delay(1000); 
            steer(0);
            delay(1000);  
            
            //Default parameters
            //{Ditance, anglefromstraight, timeToTarget, angleToTarget, isWaypoint}
            First_Target = (Target){10.0, 0, 0, 0.0, false};

            int readingType = getBluetoothInputType(); // 0 for Target, 1 for Waypoint
            First_Target.distance = getBluetoothReading(0, 'd') / 100;
            Serial.println("Distance: ");
            Serial.print(First_Target.distance);
            
            if (readingType == 0) 
            {
              First_Target.isWaypoint = false;
            } 
            else 
            {
              First_Target.isWaypoint = true;
            }
              
              Serial.println(First_Target.isWaypoint);

//            //Non-BlueTooth Definitions            
//            //Set Target 0
//            Targets[0].distance = 10;
//            Targets[0].angleFromStraight = 215 + straight_yaw;
//            Targets[0].isWaypoint = false;
//            //Set Target 1
//            Targets[1].distance = 10;
//            Targets[1].angleFromStraight = 215 + straight_yaw;
//            Targets[1].isWaypoint = true;
//            //Set Target 2 - If condition
//            Targets[2].distance = 10;
//            Targets[2].angleFromStraight = 215 + straight_yaw;
//            Targets[2].isWaypoint = false;

            calculateTimeAndAngle(&First_Target);
            next_state = idle;
            Serial.print("PROGRAMME READY");
            Serial.println();
        }

        
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("PROGRAMME NOT READY");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }

      case idle:
      {
        Serial.print("IDLE");
        forward(0);
        reverse(0);
        //Wait for BT start signal
        start_flag = getBluetoothFlag();
        
        if(start_flag == 1)
        {
          Serial.print("IDLE SYS READY");
          Serial.println();
          SlowMode = schedule(First_Target.timeToTarget);
          next_state = drive;
          start_flag = 0;
        }

        
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("IDLE SYS NOT READY");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }

      
      
      case drive:
      {
        reverse(0);
        forward(100);
        steer(0);
        
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          //Serial.print(actual_yaw);
          Serial.print("DRIVE STRAIGHT");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        
        break;
      }
      
      case slow:
      {
        forward(0);
        reverse(50);
        steer(0);
        
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("SLOW");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }

      case target:
      {
        reverse(0);
        forward(0);
        steer(0);
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("TARGET");
          Serial.println();
          
          Serial.print(target_counter);
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        
         closestTarget = scanForTargets_Ultrasound();
         if (closestTarget.angleToTarget != -690) 
         {
             Serial.println("TARGET SUCCESSFULlY LOCATED");
             Serial.println(target_counter);
             next_state = stabilize_IMU;                              //Allow some IMU readings to be taken - Just wait state in this test
             HitMode = schedule(500);                                 //Schedule Transition to hit mode
             closestTarget.distance = closestTarget.distance/100;    //Convert to m
             calculateTime(&closestTarget);
         } else 
         {
              Serial.println("UNABLE TO LOCATE TARGET");
              next_state = stabilize_IMU;  //Stabilize IMU while it waits
              IdleMode = schedule(500);   //End Test
         }
        break;
      }

      case stabilize_IMU:         //Simple a wait state in this Test
      {
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("Stabilize_IMU NO calcs");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }

        break;
      }

      
      case hit:
      {
        
        reverse(0);
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("HIT MANUAL");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        } 
      
      if(closestTarget.angleToTarget < -5)       //If to left of target turn left and move slowly
      {
         if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("LEFt");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        } 
        steer(-10);
        forward(80);
      }
      else if(closestTarget.angleToTarget > 5) //If to right of target turn right and move slowly
      {
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("RIGHT");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        } 
        steer(10);
        forward(80);
      }
      else                                    //If relatively straight go forward full and schedule change to drive for next target
      {
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("Straight");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        } 
        steer(0);
        forward(100);
        target_counter = 100;
        next_state = stabilize_IMU;
        IdleMode = schedule(closestTarget.timeToTarget+500);
        HitMode.ms = -1;
      }
      
      break;
      }
      

    }

    current_state = next_state;
  
}
