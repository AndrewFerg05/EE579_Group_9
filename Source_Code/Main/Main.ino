//Should drive to target area and scan once
//If can detected attempts to hit before moving onto next target

#include "IMU.h"
#include "Scheduler.h"
#include "Target.h"
#include "PID.h"
#include "CarControl.h"
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
#define RATE_LIMIT_INTERVAL 10           //Change to set duration in ms between Rate_Limit updates

//Test Print Variables
#define PRINT_SPEED 200 // ms between prints
unsigned long lastPrint = 0; // Keep track of print time

//Variables
float actual_yaw = 0;
float straight_yaw = 0;
bool programme_ready_flag = 0;
bool start_flag = 0;
bool Rate_flag = 1;
int target_select = 0;
Target Targets[3];
Target End_Target;
Target closestTarget;


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
    endsequence     //Drive away from start
};

enum State current_state = programme, next_state = programme;

//Events
Time CurrentTime = {0,0};
Time IdleMode = {0, -1};
Time ProgrammeReady = {0,-1};
Time DriveMode = {0, -1};
Time SlowMode = {0, -1};
Time TargetMode = {0, -1};
Time EndProtocol = {0, -1};
Time StartFlag = {0, -1};
Time RateUpdate = {0, -1};


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

  
  if(IsScheduled(DriveMode)) 
  {
    DriveMode.ms = -1;
    if(target_select > 2)
    {
      next_state = endsequence;
    }
    else
    {
      next_state = drive;
      SlowMode = schedule(Targets[target_select].timeToTarget);
      RateUpdate = schedule(RATE_LIMIT_INTERVAL);   
    }

  }


  if(IsScheduled(SlowMode)) 
  {
    next_state = slow;
    SlowMode.ms = -1;
    if(Targets[target_select].isWaypoint == false)
    {
      TargetMode = schedule(slow_duration);    
    }
    else
    {
      DriveMode = schedule(slow_duration);
    }
    target_select++;
  }

  
  if(IsScheduled(TargetMode)) 
  {
    RateUpdate.ms = -1;
    next_state = target;
    TargetMode.ms = -1;
    //DriveMode = schedule(targetHit_duration);
  }
  
  
  
  if(IsScheduled(EndProtocol)) 
  {
    next_state = endsequence;
    RateUpdate = schedule(RATE_LIMIT_INTERVAL); 
    EndProtocol.ms = -1;
    DriveMode.ms = -1;
    SlowMode.ms = -1;
    TargetMode.ms = -1;
  }
  
  if(IsScheduled(IdleMode)) 
  {
    next_state = idle;
    IdleMode.ms = -1;
    EndProtocol.ms = -1;
    DriveMode.ms = -1;
    SlowMode.ms = -1;
    TargetMode.ms = -1;
    RateUpdate.ms = -1;
  }

    if(IsScheduled(RateUpdate))
  {
    Rate_flag = 1;
    RateUpdate = schedule(RATE_LIMIT_INTERVAL);
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

//Main
void setup()
{
  Serial.begin(115200);
  setupDrive();
  setupIMU();
  setupTimer();
  setupBluetooth();
  ProgrammeReady = schedule(4000);
}


void loop()
{
  
    updateYaw();
    switch(current_state)
    {
      case programme:
      {
        straight_yaw = getYaw();
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
            Targets[0] = (Target){10.0, straight_yaw, 0, 0.0, false};
            Targets[1] = (Target){10.0, straight_yaw, 0, 0.0, false};
            Targets[2] = (Target){10.0, straight_yaw, 0, 0.0, true};
            End_Target = (Target){20.0, straight_yaw, 120000, straight_yaw, true};


            //BlueTooth Definition
            int numberTargets = getBluetoothNumberTargets();
            
            for(int index = 0; index < numberTargets; index++) 
            {
              int readingType = getBluetoothInputType(); // 0 for Target, 1 for Waypoint
              Targets[index].distance = getBluetoothReading(index+1, 'd') / 100;
              Serial.println("Distance: ");
              Serial.print(Targets[index].distance);
              Targets[index].angleFromStraight = normalizeAngle360(getBluetoothReading(index+1, 'a') + straight_yaw);
              Serial.println("Angle: ");
              Serial.print(Targets[index].angleFromStraight);
              
              if (readingType == 0) 
              {
                Targets[index].isWaypoint = false;
              } 
              else 
              {
                Targets[index].isWaypoint = true;
              }
              
              Serial.println(Targets[index].isWaypoint);
           }

////            //Non-BlueTooth Definitions            
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

            calculateTargets();
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

        //Wait for BT start signal
        forward(0);
        reverse(0);
        steer(0);
        start_flag = getBluetoothFlag();
        if(start_flag == 1)
        {
          EndProtocol = schedule(EndProtocol_duration);   //Drive away with 20s to go
          IdleMode = schedule(SystemEnd_duration);      //Return to idle at end
          Serial.print("IDLE SYS READY");
          Serial.println();
          SlowMode = schedule(Targets[target_select].timeToTarget);
          next_state = drive;
          RateUpdate = schedule(RATE_LIMIT_INTERVAL);
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
        actual_yaw= getYaw();
        if( Rate_flag == 1)
        {
          BTprintError(101);
          BTprintfloat(actual_yaw);
          BTprintfloat(Targets[target_select].angleToTarget);
          control_signal = rateLimiter(Targets[target_select].angleToTarget, actual_yaw, control_signal);
          forward(75);
          steer(control_signal);
          
          Rate_flag = 0;
        }
        
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          //Serial.print(actual_yaw);
          Serial.print("DRIVE");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
           
        break;
      }
      
      case slow:
      {
        forward(0);
        reverse(10);
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
           
        strikeCanCloseDistance();
        next_state = stabilize_IMU;
        DriveMode = schedule(500);

          Serial.print("Target");
          Serial.println();
        
        break;
      }

      case stabilize_IMU:
      {
        actual_yaw= getYaw();
        //Calculate params 
          if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("Stabilize_IMU");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }
      
      
      case endsequence:
      {
        actual_yaw= getYaw();
        if( Rate_flag == 1)
        {
          
          control_signal = rateLimiter(End_Target.angleToTarget, actual_yaw, control_signal);
          forward(100);
          steer(control_signal);
         
          Rate_flag = 0;
        }

        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("END PROTOCOL");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }
    }

    current_state = next_state;
  
}
