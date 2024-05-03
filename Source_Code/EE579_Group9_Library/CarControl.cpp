#include "CarControl.h"
#include "Target.h"
#include "BT_Comms.h"
#include <Arduino.h>

void setupDrive()
{
  //  Reverse
  ledcAttachPin(MOTOR_REVERSE_PIN, 1); // Attach PWM channel 1 to reverse motor pin
  ledcSetup(1, DRIVE_PWM_FREQ, 8);     // PWM frequency: 10 Hz, PWM resolution: 8-bit (0-255)
  ledcWrite(1, 0);                     // Set PWM duty cycle
                                       //  Forward
  ledcAttachPin(MOTOR_FORWARD_PIN, 0); // Attach PWM channel 0 to forward motor pin
  ledcSetup(0, DRIVE_PWM_FREQ, 8);     // PWM frequency: 10 Hz, PWM resolution: 8-bit (0-255)
  ledcWrite(0, 0);                     // Set PWM duty cycle
                                       //  Steer
  ledcAttachPin(MOTOR_STEER_PIN, 3);   // Attach PWM channel 2 to steering servo pin
  ledcSetup(3, 50, 11);                // Configure LEDC channel 2 with a frequency of 50Hz and a resolution of 8 bits
  ledcWrite(3, 164);                    // Default to Straight
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
  // Constrain steering angle input
  steering = constrain(steering, -20, 20);

  int dutycycle;
  //  Straight = Duty Cycle 82
  //  Full lock left = Duty Cycle 71
  //  Full lock left = Duty Cycle 88

  if (steering < 0)
  {
    // Map the range from -20 to 0 to the duty cycle range of 71 to 82 - full left steerin range
    dutycycle = map(steering, -20, 0, 146, 160);
  }
  else
  {
    // Map the range from 0 to 20 to the duty cycle range of 82 to 88 - full right steering range
    dutycycle = map(steering, 0, 20, 160, 170);
  }
  ledcWrite(3, dutycycle); // Update servo PWM duty cycle
}

void carControl(float angle, float forwardPower, float reversePower, float duration)
{

  if (duration > 0)
  {
    steer(angle);
    delay(30);
    forward(forwardPower);
    reverse(reversePower);
    delay(duration * 1000); // ms
    forward(0);
    reverse(0);
  }
}

void strikeCanCloseDistance()
{

  bool rescan = true;
  float driveTime;
  Target closestTarget;
  int numberScans = 0;
  int steer_intern = 0;

  while (rescan and numberScans <= 15)
  {
    numberScans++;

    closestTarget = scanForTargets_Ultrasound();
    BTprintError(0);
    BTprintfloat(closestTarget.distance);
    // BTprintint(closestTarget.angleToTarget);

    if (closestTarget.angleToTarget == -690)
    {
      BTprintError(1);
      // No target was found in 5 scans. Move on to the next target.
      rescan = false;
    }
    closestTarget.angleToTarget+=7;
    BTprintint(closestTarget.angleToTarget);
    if (closestTarget.angleToTarget > -10 and closestTarget.angleToTarget < 10 and closestTarget.distance < 0.4)
    {
      BTprintError(2);
      // CAR ALIGNED AND CLOSE
      carControl(0, 0, 75, 0.4);  // reverse a little (for added effect)
      delay(500);
      steer(-20);
      delay(200);
      steer(20);
      delay(200);
      steer(0);
      delay(200);
      carControl(0, 100, 0, 1); // strike the can
      rescan = false;             // move on to next target
    }
    else if (closestTarget.angleToTarget > -10 and closestTarget.angleToTarget < 10 and closestTarget.distance < 1.5)
    {
      BTprintError(3);
      if(closestTarget.angleToTarget<0)
      {
        steer_intern = -10;
      }
      else if (closestTarget.angleToTarget>0)
      {
        steer_intern = 10;
      }
      else
      {
        steer_intern = 0;
      }
      // CAR ALIGNED - DRIVE A LITTTLE AND STEER GENTLY
      carControl(steer_intern, 75, 0, 0.5);
    }

    else if (closestTarget.angleToTarget > -10 and closestTarget.angleToTarget < 10 and closestTarget.distance >= 1.5)
    {
      BTprintError(3);
      // CAR ALIGNED BUT FAR - DRIVE FORWARD
      driveTime = (closestTarget.distance - 0.1) / car_speed; 
      carControl(0, 75, 0, driveTime);
    }
      
    else if (closestTarget.angleToTarget > -40 and closestTarget.angleToTarget < 40 and closestTarget.distance < 0.5)
    {
      BTprintError(4);
      // CAN TOO STEEP AN ANGLE TO HIT
      if(closestTarget.angleToTarget<0)
      {
        steer_intern = -20;
      }
      else if (closestTarget.angleToTarget>0)
      {
        steer_intern = 20;
      }
      else
      {
        steer_intern = 0;
      }

      carControl(-steer_intern, 0, 75, 0.3); // reverse away from can
    }
    else if (closestTarget.angleToTarget > -40 and closestTarget.angleToTarget < 40 and closestTarget.distance >= 0.5)
    {
      BTprintError(5);
      if(closestTarget.angleToTarget<0)
      {
        steer_intern = -20;
      }
      else if (closestTarget.angleToTarget>0)
      {
        steer_intern = 20;
      }
      else
      {
        steer_intern = 0;
      }
      //CAR LESS ALIGNED - STEER MORE
      carControl(steer_intern, 75, 0, 0.6);
    }

    else if (closestTarget.angleToTarget > -70 and closestTarget.angleToTarget < 70 and closestTarget.distance < 1)
    {
      BTprintError(7);
      // CAN TOO STEEP AN ANGLE TO HIT
      carControl(-closestTarget.angleToTarget, 0, 75, 0.5); // reverse away from can
    }

    else if (closestTarget.angleToTarget > -70 and closestTarget.angleToTarget < 70 and closestTarget.distance >= 1)
    {
      BTprintError(8);
      if(closestTarget.angleToTarget<0)
      {
        steer_intern = -20;
      }
      else if (closestTarget.angleToTarget>0)
      {
        steer_intern = 20;
      }
      else
      {
        steer_intern = 0;
      }
      carControl(steer_intern, 75, 0, 0.75);
    }

    else if ((closestTarget.angleToTarget > -90 and closestTarget.angleToTarget < 90) and closestTarget.distance < 1)
    {
      BTprintError(9);
      // CAN TOO STEEP AN ANGLE TO HIT
      carControl(-closestTarget.angleToTarget, 0, 75, 0.5); // reverse away from can
      carControl(0, 75, 0, 0.75); // reverse away from can
    }

    else if ((closestTarget.angleToTarget > -90 or closestTarget.angleToTarget < 90) and closestTarget.distance >= 1)
    {
      if(closestTarget.angleToTarget<0)
      {
        steer_intern = -20;
      }
      else if (closestTarget.angleToTarget>0)
      {
        steer_intern = 20;
      }
      else
      {
        steer_intern = 0;
      }
      carControl(steer_intern, 75, 0, 1);
    }

    delay(500); // delay to allow for momentum to stop
  }
}
