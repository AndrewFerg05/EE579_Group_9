
#include "AF_IMU.h"
#include "AF_scheduler.h"
#include "AF_Target.h"
#include "SD_Comms.h"


#define slow_duration 500
#define targetHit_duration 5000
#define EndProtocol_duration 30000
#define SystemEnd_duration 35000

//Test Print Variables
#define PRINT_SPEED 200 // ms between prints
unsigned long lastPrint = 0; // Keep track of print time

//Variables
float actual_yaw = 0;
float straight_yaw = 0;
bool programme_ready_flag = 0;
bool start_flag = 0;
int target_select = 0;
Target Targets[3];
Target End_Target;




//State Machine Variables
enum State
{
    programme,      //waiting input variables
    idle,           //wait for start
    drive,          //moving towards target
    slow,           //slowing down when nearer target
    target,         //sensing and hitting
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


//Timer Pointer
hw_timer_t *Timer0 = NULL;


//Interrupts
  //Scheduler Interrupt - Clocked @ 1ms - handles state transitions
void IRAM_ATTR Timer0_ISR()
{
  CurrentTime = increment(CurrentTime);


  if(IsScheduled(StartFlag))
  {
    start_flag = 1;
    StartFlag.ms = -1;
    
  }




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
    }

    //SlowMode = schedule(10000);
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
    //TargetMode = schedule(10000);
  }

  
  if(IsScheduled(TargetMode)) 
  {
    next_state = target;
    TargetMode.ms = -1;
    DriveMode = schedule(targetHit_duration);
  }

  
  if(IsScheduled(EndProtocol)) 
  {
    next_state = endsequence;
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
  while (!Serial); //wait for connection
//  setupIMU();
  setupTimer();
//  serialBT.begin("ESP32-BT");
} 




void loop()
{
   
    
    updateYaw();
//    if (millis() - lastPrint > PRINT_SPEED) 
//    {
//      actual_yaw= getYaw();
//      Serial.print(actual_yaw, 0);
//      Serial.println();
//      lastPrint = millis(); // Update lastPrint time
//    }

    switch(current_state)
    {
      case programme:
      {
        if(programme_ready_flag == 1)
        {
            
           
            straight_yaw = getYaw();
            //Default parameters
            //{Ditance, anglefromstraight, timeToTarget, angleToTarget, isWaypoint}
            Targets[0] = (Target){10.0, straight_yaw, 0, 0.0, false};
            Targets[1] = (Target){10.0, straight_yaw, 0, 0.0, false};
            Targets[2] = (Target){10.0, straight_yaw, 0, 0.0, true};
            End_Target = (Target){20.0, straight_yaw, 120000, straight_yaw, true};

            //Replace with Bluetooth - Can add condition to not set 3 targets and leave last target as default
            
            
            //Set Target 0
            Targets[0].distance = 6;
            Targets[0].angleFromStraight = 30;
            Targets[0].isWaypoint = false;
            //Set Target 1
            Targets[1].distance = 7;
            Targets[1].angleFromStraight = 30;
            Targets[1].isWaypoint = true;
            //Set Target 2 - If condition
            Targets[2].distance = 8;
            Targets[2].angleFromStraight = -30;
            Targets[2].isWaypoint = false;

            calculateTargets();
            next_state = idle;

            StartFlag = schedule(2000); //Testing Only

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

        //Change Start Flag with Bluetooth
        if(start_flag == 1)
        {
          EndProtocol = schedule(EndProtocol_duration);   //Drive away with 20s to go
          IdleMode = schedule(SystemEnd_duration);      //Return to idle at end
          Serial.print("IDLE SYS READY");
          Serial.println();
          SlowMode = schedule(Targets[target_select].timeToTarget);
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

        //DRIVE
        //CONTROL ANGLE Targets[target_select].angleToTarget
        
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("DRIVE");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }
      
      case slow:
      {

        //SLOW DOWN 
        //CONTROL ANGLE Targets[target_select].angleToTarget
        
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

        Target closestTarget = (Target){0, 0, 0, 0, false};
    
        closestTarget = scanForTargets_Ultrasound();

        if (millis() - lastPrint > PRINT_SPEED) 
        {
          closestTarget = scanForTargets_Ultrasound();
          Serial.print("TARGET");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }
      
      case endsequence:
      {

        //DRIVE
        //CONTROL ANGLE End_Target.angleToTarget
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
