// ADVANCED MICROCONTROLLERS
// Group X
// Andrew F, Ciaran P and Sean D

#include "IMU.h"
#include "Scheduler.h"
#include "Target.h"
#include "PID.h"
#include "BT_Comms.h"
#include "CarControl.h"

#define slow_duration 500
#define targetHit_duration 10000
#define EndProtocol_duration 100000
#define SystemEnd_duration 120000

#define RATELIMITER_INTERVAL 50

//Test Print Variables
#define PRINT_SPEED 200 // ms between prints
unsigned long lastPrint = 0; // Keep track of print time

//Variables
float actual_yaw = 0;
float straight_yaw = 0;
bool programme_ready_flag = 0;
bool start_flag = 0;
bool RateLimiter_Flag = 1;
int target_select = 0;
int currentSteering = 0;
int prevError = 0;

Target Targets[3];
Target End_Target;

//PID Variables
PIDConfig Steer_PID;
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
//    hit,          //hit target
    runaway,        // put as much distance between can and car as possible
    endsequence     //Drive away from start
};

enum State current_state = programme, next_state = programme;

//Events
Time CurrentTime = {0,0};
Time IdleMode = {0, -1};
Time ProgrammeReady = {2,0};
Time DriveMode = {0, -1};
Time SlowMode = {0, -1};
Time TargetMode = {0, -1};
Time EndProtocol = {0, -1};
Time StartFlag = {0, -1};
Time RateLimiterUpdate = {0, -1};
//Time HitMode = {0,-1};

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
      RateLimiterUpdate = schedule(RATELIMITER_INTERVAL);   
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
    RateLimiterUpdate.ms = -1;
    next_state = target;
    TargetMode.ms = -1;
    //DriveMode = schedule(targetHit_duration);
  }
  
//  if(IsScheduled(HitMode)) 
//  {
//    HitMode.ms = -1;
//    next_state = hit;
//    DriveMode = schedule(closestTarget.timeToTarget + 500); //Extra to drive past can
//    PIDUpdate = schedule(PID_INTERVAL);                     //Start PID
//  }

  
  if(IsScheduled(EndProtocol)) 
  {
    next_state = endsequence;
    RateLimiterUpdate = schedule(RATELIMITER_INTERVAL); 
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
    RateLimiterUpdate.ms = -1;
  }

  if(IsScheduled(RateLimiterUpdate))
  {
    RateLimiter_Flag = 1;
    RateLimiterUpdate = schedule(RATELIMITER_INTERVAL);
  }
}


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
  setupPID(&Steer_PID, P_Gain, I_Gain, D_Gain, I_Limit);
  setupBluetooth();
}


void loop()
{
    
    updateYaw();  // Update yaw as frequently as possible to keep IMU fresh 
    
    switch(current_state)
    {
      case programme:
      {
        
        // Stabilise the IMU by running it for some time
//        for(int i = 0; i < 50; i++) {
//          updateYaw();
//          getYaw();
//        }

       
        straight_yaw = getYaw(); // set straight yaw: the reference for all other angles
        
        if(programme_ready_flag == 1)
        {
            
            // Wiggle wheels to indicate car is ready
            
            carControl(0, 0, 0, 0.5);
            carControl(20, 0, 0, 0.5); // steer, forward power, reverse power, time
            carControl(-20, 0, 0, 0.5);
            carControl(0, 0, 0, 0.5);

    
            //Default parameters of targets
            //{Ditance, anglefromstraight, timeToTarget, angleToTarget, isWaypoint}
            Targets[0] = (Target){10.0, straight_yaw, 0, 0.0, false};
            Targets[1] = (Target){10.0, straight_yaw, 0, 0.0, false};
            Targets[2] = (Target){10.0, straight_yaw, 0, 0.0, true};
            End_Target = (Target){20.0, straight_yaw, 120000, straight_yaw, true};

            
            // Get actual target details from user
            int numberTargets = getBluetoothNumberTargets();
            
            for(int index = 0; index < numberTargets; index++) 
            {
              int readingType = getBluetoothInputType(); // 0 for Target, 1 for Waypoint
              Targets[index].distance = getBluetoothReading(index+1, 'd');
              Targets[index].angleFromStraight = normalizeAngle360(getBluetoothReading(index+1, 'a') + straight_yaw);
//              prevError = Targets[index].angleFromStraight - straight_yaw; // this will help to stabilise the rate limiter. See RateLimiter.cpp    
              if (readingType == 0) { 
                Targets[index].isWaypoint = false;
              } else {
                Targets[index].isWaypoint = true;
              }
           }


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
          SlowMode = schedule(Targets[target_select].timeToTarget);
          next_state = drive;
          RateLimiterUpdate = schedule(RATELIMITER_INTERVAL);
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
        
        if( RateLimiter_Flag == 1)
        {
          currentSteering = rateLimiter(Targets[target_select].angleFromStraight, actual_yaw, currentSteering);
          steer(currentSteering);
          forward(100); 
          RateLimiter_Flag = 0;
          BTprintfloat(currentSteering);
          
        }
        
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print(actual_yaw);
          Serial.print("DRIVE");
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
            
        strikeCanCloseDistance();
       
        next_state = stabilize_IMU;
        DriveMode = schedule(500);
        
        break;
      }

      case stabilize_IMU:
      {
        actual_yaw= getYaw();  
        
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
        if( RateLimiter_Flag == 1)
        {
          currentSteering = rateLimiter(Targets[target_select].angleFromStraight, actual_yaw, currentSteering);
          steer(currentSteering);
          forward(100); 
          RateLimiter_Flag = 0;
          BTprintfloat(currentSteering);
          if (collisionTest) {
            forward(0);
            
            BTprintfloat(-1);
            while(1) {
              delay(1000);
            }
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
}
