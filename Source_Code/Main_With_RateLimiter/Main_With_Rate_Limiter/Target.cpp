#include "Target.h"
#include <Arduino.h>


void calculateTimeAndAngle(Target* target)
{
    target->timeToTarget = (int)(((target->distance / car_speed) * 1000) - time_offset);   // Caculate how long it will take to reach the target
    target->angleToTarget = normalizeTargetAngle(target->angleFromStraight);   // Calculate the angle relative to the car's current position    
}


void calculateTime(Target* target) {
  target->timeToTarget = (int)(((target->distance / car_speed) * 1000) - time_offset);   // Caculate how long it will take to reach the target
}

void calculateTargets()
{
    for (int i = 0; i < 3; i++) 
    {
        calculateTimeAndAngle(&Targets[i]);
    }
}

void setupUltrasound() {
  pinMode(servoPin, OUTPUT); 
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

double sendUltrasoundPing()
{
    // Clear  output
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);

    // Send 20 us ping
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(29);
    digitalWrite(triggerPin, LOW);

    long startTime = micros(); // start the timer
    
    // Wait for pulse to return
    while (digitalRead(echoPin) == LOW) {
      long tempReading = micros();
      if (tempReading - startTime > 50000) { // if the pulse doesn't return within the time frame, return 5s
        return 50000;
      }
    };
    
    startTime = micros(); // update the start time now that the pulse has started

    while (digitalRead(echoPin) == HIGH) {
      long tempReading = micros();
      if (tempReading - startTime > 50000) {
        return 50000;
      }
    }
    double pulseWidth = micros() - startTime;
 
    return pulseWidth;
}



// Turn servo to angle through PWM signal
void turnServo(int angle) {
  int servoPulseWidth = map(angle, 0, 180, 500, 2400); // Map angle to pulse width
  digitalWrite(servoPin, HIGH); // Set pin high
  delayMicroseconds(servoPulseWidth); // Wait for pulse width duration
  digitalWrite(servoPin, LOW); // Set pin low
  delay(30); // Delay for stability 
}


Target scanForTargets_Ultrasound() 
{
    // SETUP
    setupUltrasound();

    // DEFAULT VALUES
    Target closestTarget; 
    float distanceReadings[91]; // 0 = most right, 90 = most left

    
    // BEGIN SCAN (max 5)
    for (int scanCount = 0; scanCount < 5; scanCount++) 

    {
        // TAKE 90 READINGS, one every second degree
        for (int i = 0; i <= 90; i++)
        {
            turnServo(i*2);
            distanceReadings[i] = 0.01723 * sendUltrasoundPing();
        }

        // PROCESS SCAN TO FIND CLOSEST VALID OBJECT
        double lowerBoundary = -1;
        bool processingComplete = false;
        
        while (!processingComplete)
        {
           
            // Smooth the readings
            for (int i = 1; i < 90; i++) {
              if ((distanceReadings[i-1] - distanceReadings[i] < -10 or distanceReadings[i-1] - distanceReadings[i] > 10) and (distanceReadings[i+1] - distanceReadings[i] < -10 or distanceReadings[i+1] - distanceReadings[i] > 10)) {
                distanceReadings[i] = distanceReadings[i-1];
              }
            }

            // Find the smallest distance in the array (which is greater than the lower boundary)
            int targetAngle = -1; // smallest angle
            
            for (int i = 0; i <= 90; i++)
            {
                if (targetAngle == -1) { // if the target angle has not been set, set it
                  if (distanceReadings[i] > lowerBoundary) {
                    targetAngle = i; 
                  }
                  
                } else { // if target angle has already been set, check if the new distance is smaller
                  if (distanceReadings[i] < distanceReadings[targetAngle] && distanceReadings[i] > lowerBoundary)
                  {
                      targetAngle = i;
                  }
                }
            }

            // Print the readings 
            for (int i = 0; i <= 90; i++) {
              Serial.println(distanceReadings[i]);
            }

            if (targetAngle == -1)
            {
                // i.e. there was no valid object in the array, and a rescan must take place
                processingComplete = true;
            }

            // Find the maximum distance  within a tolerance of 10 cm
            int leftmostAngle = targetAngle;
            for (int i = targetAngle + 1; i <= 90; i++)
            {
                double difference = distanceReadings[i] - distanceReadings[i - 1];
                if (difference < 0)
                {
                    difference = difference * -1;
                }
                if (difference >= 10)
                {
                    leftmostAngle = i - 1;
                    break;
                }
                if (i == 90){
                  leftmostAngle = 90;
                }
            }

            // Find the lowest distance within a tolerance of 5cm
            int rightmostAngle = targetAngle;
            for (int i = targetAngle - 1; i >= 0; i--)
            {
                int difference = distanceReadings[i + 1] - distanceReadings[i];
                if (difference < 0)
                {
                    difference = difference * -1;
                }

                if (difference >= 10)
                {
                    rightmostAngle = i + 1;
                    break;
                }
                if (i == 0) {
                  rightmostAngle = 0;
                }
            }

            double furthestDistance_inLimits = distanceReadings[targetAngle];
            double closestDistance_inLimits = distanceReadings[targetAngle];
            
            for (int i = rightmostAngle; i <= leftmostAngle; i++)
            {
                if (distanceReadings[i] > furthestDistance_inLimits)
                {
                    furthestDistance_inLimits = distanceReadings[i];
                }
                if (distanceReadings[i] < closestDistance_inLimits)
                {
                    closestDistance_inLimits = distanceReadings[i];
                }
            }

          // calculate the objects angle range
          int angleRange = leftmostAngle - rightmostAngle + 1;

          // calclulate the mean distance
          double sum = 0.0;
          for (int i = rightmostAngle; i <= leftmostAngle; i++)
          {
            sum += distanceReadings[i];
          }
          double meanDistance = sum / angleRange;
            
            
          // calcualte the objects line of best fit
          double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x_squared= 0;

          for (int i = rightmostAngle; i <= leftmostAngle; ++i) {
                sum_x += i;
                sum_y += distanceReadings[i];
                sum_xy += i * distanceReadings[i];
                sum_x_squared += i * i;
           }
           
          double objectGradient = (angleRange * sum_xy - sum_x * sum_y) / (angleRange * sum_x_squared - sum_x * sum_x);

        
            
          // Decide if there is a valid can
            if (angleRange <= 5)
            {
                Serial.println("False Reading: Angle range was too small");
                lowerBoundary = distanceReadings[targetAngle];
            }
            else if (objectGradient > 0.15 or objectGradient < -0.15) {
              Serial.println("False Reading: Angle range was too small");
              lowerBoundary = distanceReadings[targetAngle];
            } else {
              // CAN DETECTED
              // Turn servo to face can
              turnServo(targetAngle * 2);    

               // Updated variables
              closestTarget.distance = meanDistance / 100; // CM -> M

              targetAngle = round((leftmostAngle + rightmostAngle) / 2);
              
              closestTarget.angleToTarget = -1*(targetAngle*2 - 90); // map range from 0 -> 180 (in steps of two) to 90 -> -90
              processingComplete = true;
              return closestTarget;
            }
        }

    };

    // if the code reaches here, 5 scans failed to find a can
    closestTarget.angleToTarget = -690;
    return closestTarget;
}


float normalizeTargetAngle(float angle) {
    while (angle < 0) 
    {
        angle += 360;
    }
    while (angle > 360) 
    {
        angle -= 360;
    }
    return angle;
}


bool collisonTest() {
  turnServo(90);
  float distanceToObject = 0.01723 * sendUltrasoundPing();
  if (distanceToObject < 30) {
    return true;
  } else {
    return false;
  }
}
