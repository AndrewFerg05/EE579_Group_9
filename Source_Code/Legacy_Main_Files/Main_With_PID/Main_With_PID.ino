//Should drive to target area and scan once
//If can detected attempts to hit before moving onto next target

#include "IMU.h"
#include "Scheduler.h"
#include "Target.h"
#include "PID.h"
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
#define PID_INTERVAL 5            //Change to set duration in ms between PID updates

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
Target Targets[3];
Target End_Target;
Target closestTarget;

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
    hit,            //hit target
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
Time PIDUpdate = {0, -1};
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
      PIDUpdate = schedule(PID_INTERVAL);   
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
    PIDUpdate.ms = -1;
    next_state = target;
    TargetMode.ms = -1;
    //DriveMode = schedule(targetHit_duration);
  }
  
  
  if(IsScheduled(HitMode)) 
  {
    HitMode.ms = -1;
    next_state = hit;
    DriveMode = schedule(closestTarget.timeToTarget + 500); //Extra to drive past can
    PIDUpdate = schedule(PID_INTERVAL);                     //Start PID
  }
  
  if(IsScheduled(EndProtocol)) 
  {
    next_state = endsequence;
    PIDUpdate = schedule(PID_INTERVAL); 
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
    PIDUpdate.ms = -1;
  }

    if(IsScheduled(PIDUpdate))
  {
    PID_flag = 1;
    PIDUpdate = schedule(PID_INTERVAL);
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
  setupIMU();
  setupTimer();
  setupPID(&Steer_PID, P_Gain, I_Gain, D_Gain, I_Limit);
  setupBluetooth();
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
          PIDUpdate = schedule(PID_INTERVAL);
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
        if( PID_flag == 1)
        {
          control_signal = PID(&Steer_PID, Targets[target_select].angleToTarget, actual_yaw);
          carControl(control_signal);
          
          PID_flag = 0;
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
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("TARGET");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        
        closestTarget = scanForTargets_Ultrasound();
        if (closestTarget.angleToTarget != -690) 
         {
             Serial.println("TARGET SUCCESSFULlY LOCATED");
             next_state = stabilize_IMU;      //Allow some IMU readings to be taken
             HitMode = schedule(500);         //Schedule Transition to hit mode
             closestTarget.distance = closestTarget.distance/100;    //Convert to m 
         } else 
         {
              Serial.println("UNABLE TO LOCATE TARGET");
              next_state = stabilize_IMU;  //Stabilize IMU while it waits
              DriveMode = schedule(500);  //Drive away
         }
        break;
      }

      case stabilize_IMU:
      {
        actual_yaw= getYaw();
        closestTarget.angleToTarget = normalizeAngle360(closestTarget.angleToTarget + actual_yaw);              //Calculate absolute angle to can
        calculateTime(&closestTarget);                  //Calculate params 
          if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("Stabilize_IMU");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }
      
      case hit:
      {
        reverse(0);
        actual_yaw= getYaw();
        if( PID_flag == 1)
        {
          control_signal = PID(&Steer_PID, closestTarget.angleToTarget, actual_yaw);
          carControl(control_signal);
          PID_flag = 0;
        }
        if (millis() - lastPrint > PRINT_SPEED) 
        {
          Serial.print("HIT");
          Serial.println();
          lastPrint = millis(); // Update lastPrint time
        }
        break;
      }
      
      case endsequence:
      {
        actual_yaw= getYaw();
        if( PID_flag == 1)
        {
          control_signal = PID(&Steer_PID, End_Target.angleToTarget, actual_yaw);
          carControl(control_signal);
         
          PID_flag = 0;
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
