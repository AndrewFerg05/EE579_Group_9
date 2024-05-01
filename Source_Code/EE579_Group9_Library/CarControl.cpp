#include "CarControl.h"
#include "Target.h"
#include "BT_Comms.h"

#include <Arduino.h>


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
    //Constrain steering angle input
    steering = constrain(steering, -20, 20);
    
    int dutycycle;
//  Straight = Duty Cycle 82
//  Full lock left = Duty Cycle 71
//  Full lock left = Duty Cycle 88


    if (steering < 0) 
    {
        // Map the range from -20 to 0 to the duty cycle range of 71 to 82 - full left steerin range
        dutycycle = map(steering, -20, 0, 71, 82);
    } 
    else 
    {
        // Map the range from 0 to 20 to the duty cycle range of 82 to 88 - full right steering range
        dutycycle = map(steering, 0, 20, 82, 88);
    }
    ledcWrite(3, dutycycle);  //Update servo PWM duty cycle
}


void carControl(float angle, float forwardPower, float reversePower, float duration) {
  
  if (duration > 0) {
    steer(angle);
    delay(30);
    forward(forwardPower);
    reverse(reversePower);
    delay(duration * 1000); // ms
    forward(0);
    reverse(0);
    delay(200);
  }
}



void strikeCanCloseDistance() {

  bool rescan = true;
  float driveTime;
  Target closestTarget;
  int numberScans = 0;
  
  while (rescan and numberScans <= 10) {
          numberScans++;
          
          closestTarget = scanForTargets_Ultrasound();   
          
          if (closestTarget.angleToTarget == -690) {
            // No target was found in 5 scans. Move on to the next target. 
            rescan = false;
          }
                       
          if (closestTarget.angleToTarget > -10 and closestTarget.angleToTarget < 10 and closestTarget.distance < 0.25) {
            // CAR ALIGNED AND CLOSE
            carControl(0, 0, 50, 0.5); // reverse a little (for added effect)
            carControl(0, 100, 0, 0.5); // strike the can
            rescan = false; // move on to next target
            
          } else if (closestTarget.angleToTarget > -10 and closestTarget.angleToTarget < 10 and closestTarget.distance > 0.25) {
            // CAR ALIGNED BUT FAR
            driveTime =  (closestTarget.distance - 0.2) / car_speed; // aim to stop a little bit before the can
            carControl(0, 100, 0, driveTime); // Drive straight up to the can

          } else if ((closestTarget.angleToTarget > 70 or closestTarget.angleToTarget < -70) and closestTarget.distance < 0.25) {
            // CAN TOO STEEP AN ANGLE TO HIT
            carControl(0, 0, 75, 1); // reverse straight
            
          } else if (closestTarget.angleToTarget > 70 and closestTarget.distance >= 0.25) {
            // CAN IS STEEP AND FAR. GO FORWARD RIGHT AND REVERSE LEFT TO LINE UP
            carControl(20, 75, 0, 1);
            carControl(-20, 0, 75, 1);
            
          } else if (closestTarget.angleToTarget < -70 and closestTarget.distance >= 0.25) {
            // CAN IS STEEP AND FAR. GO FORWARD LEFT AND REVERSE RIGHT TO LINE UP
            carControl(-20, 75, 0, 1);
            carControl(20, 0, 75, 1);
            
          } else if (closestTarget.distance < 0.25 and closestTarget.angleToTarget < 0) {
            // REVERSE IN THE OPPOSITE DIRECTION SLIGHTLY
            carControl(20, 0, 75, 1);
            
          } else if (closestTarget.distance < 0.25 and closestTarget.angleToTarget > 0) {
            // REVERSE IN THE OPPOSITE DIRECTION SLIGHTLY
            carControl(-20, 0, 75, 1);
            
          } else if (closestTarget.angleToTarget > 0) {
            // CAN IS FAR AND TO THE RIGHT
            driveTime = (closestTarget.angleToTarget / 90) * 0.9; // it takes 0.9 seconds to turn 90 degrees at 75 power: use this as reference
            carControl(20, 75, 0, driveTime);
            driveTime = ((closestTarget.distance - 0.2) / car_slow_speed ) - driveTime; // drive straight for the remaiunder
            carControl(0, 75, 0, driveTime);
            
          } else if (closestTarget.angleToTarget < 0) {
            // CAN IS FAR AND TO THE LEFT            
            driveTime = (-1* closestTarget.angleToTarget / 90) * 1.2; // it takes 1.2 seconds to turn 90 degrees TUNE!
            carControl(-20, 75, 0, driveTime);
            driveTime = ((closestTarget.distance - 0.2) / car_slow_speed ) - driveTime; // drive straight for the remaiunder
          }
        }
}
