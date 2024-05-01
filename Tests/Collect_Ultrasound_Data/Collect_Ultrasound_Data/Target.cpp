#include "Target.h"
#include <Arduino.h>

#define triggerPin 4 
#define echoPin 2 
#define servoPin 16 
#define STEADY_SPEED 1.00

void calculateTimeAndAngle(Target* target)
{
    target->timeToTarget = (int)(((target->distance / car_speed) * 1000) - time_offset);    // Tune car speed
    target->angleToTarget = normalizeTargetAngle(target->angleFromStraight - angle_offset);                      // Replace with your actual conversion
}

void calculateTargets()
{
    for (int i = 0; i < 3; i++) 
    {
        calculateTimeAndAngle(&Targets[i]);
    }
}

double sendUltrasoundPing()
{
    // Clear  output
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);

    // Send 5 us ping
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(triggerPin, LOW);

    long startTime = micros(); // start the timer
    
    // Wait for pulse to return
    while (digitalRead(echoPin) == LOW) {
      long tempReading = micros();
      if (tempReading - startTime > 50000) {
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
    double travelTime = micros() - startTime;
 
//    return the returning pulse's width
    return travelTime;
}

void setupUltrasound() {
  pinMode(servoPin, OUTPUT); 
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// Turn servo to angle through PWM signal
void turnServo(int angle) {
  int pulseWidth = map(angle, 0, 180, 500, 2400); // Map angle to pulse width
  digitalWrite(servoPin, HIGH); // Set pin high
  delayMicroseconds(pulseWidth); // Wait for pulse width duration
  digitalWrite(servoPin, LOW); // Set pin low
  delay(30); // Delay for stability 
}


Target scanForTargets_Ultrasound() 
{
    // SETUP
    setupUltrasound();

    // DEFAULT VALUES
    Target closestTarget; // default values
    float distanceReadings[91]; // 0 = most right, 89 = most left

    
    // BEGIN SCAN (max 5)
    for (int scanCount = 0; scanCount < 5; scanCount++) 

    {
        // TAKE 90 READINGS, one every second degree
        for (int i = 0; i <= 90; i++)
        {
            turnServo(i*2);
            distanceReadings[i] = 0.01723 * sendUltrasoundPing();
            
            Serial.println(distanceReadings[i]);
        }

        // PROCESS SCAN TO FIND CLOSEST VALID OBJECT
        double lowerBoundary = -1;
        bool processingComplete = false;
        int numberOfMoves = 0;
        
        while (!processingComplete and numberOfMoves <= 10)
        {
           numberOfMoves++;
            // Smooth the readings
            for (int i = 1; i < 90; i++) {
              if ((distanceReadings[i-1] - distanceReadings[i] < -10 or distanceReadings[i-1] - distanceReadings[i] > 10) and (distanceReadings[i+1] - distanceReadings[i] < -10 or distanceReadings[i+1] - distanceReadings[i] > 10)) {
                distanceReadings[i] = distanceReadings[i-1];
              }
            }

            // Find the smallest distance in the array (which is greater than the lower boundary)
            int targetAngle = -1;
            
            for (int i = 0; i <= 90; i++)
            {
                if (targetAngle == -1) {
                  if (distanceReadings[i] > lowerBoundary) {
                    targetAngle = i;
                  }
                  
                } else {
                  if (distanceReadings[i] < distanceReadings[targetAngle] && distanceReadings[i] > lowerBoundary)
                  {
                      targetAngle = i;
                  }
                }
            }

            // Print the readings 
//            for (int i = 0; i < 90; i++) {
//              Serial.println(distanceReadings[i]);
//            }

            if (targetAngle == -1)
            {
                // i.e. there was no valid object in the array, and a rescan must take place
                processingComplete = true;
            }

            // Find the maximum distance  within a tolerance of 5cm
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

            // calcaulate the object's depth and range
            double objectDepth = furthestDistance_inLimits - closestDistance_inLimits;
            int angleRange = leftmostAngle - rightmostAngle + 1;
            
            // calculate the objects variance
            double sum = 0.0;
            for (int i = rightmostAngle; i <= leftmostAngle; i++)
            {
                sum += distanceReadings[i];
            }
            double mean = sum / angleRange;
//            
//            double variance = 0.0;
//            for (int i = rightmostAngle; i <= leftmostAngle; ++i) {
//                variance += (distanceReadings[i] - mean) * (distanceReadings[i] - mean);
//            }
//            variance /= angleRange;
            
            // calcualte the objects line of best fit
            double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x_squared= 0;

            for (int i = rightmostAngle; i <= leftmostAngle; ++i) {
                sum_x += i;
                sum_y += distanceReadings[i];
                sum_xy += i * distanceReadings[i];
                sum_x_squared += i * i;
            }
        

            double m = (angleRange * sum_xy - sum_x * sum_y) / (angleRange * sum_x_squared - sum_x * sum_x);
            
//            double width = sin((angleRange/2)*3.1415/180)*mean*2;
//            
//            Serial.println("");
//            Serial.print("Target: ");
//            Serial.print(rightmostAngle);
//            Serial.print(" to ");
//            Serial.println(leftmostAngle);
//            Serial.print("Mean Distance: ");
//            Serial.println(mean);
//            Serial.print("Target Depth: ");
//            Serial.println(objectDepth);
//            Serial.print("Angle Range: ");
//            Serial.println(angleRange);
////            Serial.print("Variace: ");
////            Serial.println(variance);
//            Serial.print("Gradient: ");
//            Serial.println(m);
////            Serial.print("Width: ");
////            Serial.println(width);


            // The best indidcators of can are 
              // 1) Gradient
              // 2) Angle Range
              // 3) Depth

            Serial.println("");
            
            if (m > 0.3 or m < -0.3) {
//              Serial.println("Gradient indicates this is not a can");

              Serial.print("Target at angle: ");
              Serial.print(rightmostAngle);
              Serial.print(" to ");
              Serial.print(leftmostAngle);
              Serial.print(" was rejected because it's gradient was ");
              Serial.println(m);
              
              lowerBoundary = distanceReadings[targetAngle];
            } else if (angleRange <= 5) {
              Serial.println("Small angle range indicates this is not a can.");

              Serial.print("Target at angle: ");
              Serial.print(rightmostAngle);
              Serial.print(" to ");
              Serial.print(leftmostAngle);
              Serial.print(" was rejected because it's angle range was ");
              Serial.println(angleRange);

              lowerBoundary = distanceReadings[targetAngle];
              
            } else
            {
                // CAN DETECTED

                Serial.print("Target at angle: ");
                Serial.print(rightmostAngle);
                Serial.print(" to ");
                Serial.print(leftmostAngle);
                Serial.print(" was accepted with gradient, depth and angle:");
                Serial.println(m);
                Serial.println(objectDepth);
                Serial.println(angleRange);

              
                // Turn servo to face can
                targetAngle = round((leftmostAngle + rightmostAngle) / 2);   
             
                turnServo(targetAngle * 2); 
 
               // update variables
               closestTarget.distance = mean;
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



void strikeCanCloseDistance() {

  bool rescan = true;
  float driveTime;
  Target closestTarget;
  
  while (rescan) {
          
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
            driveTime =  (closestTarget.distance - 0.2) / STEADY_SPEED; // aim to stop a little bit before tha can
            carControl(0, 75, 0, driveTime); // Drive straight up to the can

          } else if ((closestTarget.angleToTarget > 70 or closestTarget.angleToTarget < -70) and closestTarget.distance < 0.25) {
            // CAN TOO STEEP AN ANGLE TO HIT
            carControl(0, 0, 75, 1); // reverse straight
            
          } else if (closestTarget.angleToTarget > 70 and closestTarget.distance >= 0.25) {
            // CAN IS STEEP AND FAR. GO FORWARD RIGHT AND REVERSE LEFT TO LINE UP
            carControl(20, 100, 0, 0.5);
            carControl(-20, 0, 100, 0.5);
            
          } else if (closestTarget.angleToTarget < -70 and closestTarget.distance >= 0.25) {
            // CAN IS STEEP AND FAR. GO FORWARD LEFT AND REVERSE RIGHT TO LINE UP
            carControl(-20, 100, 0, 0.5);
            carControl(20, 0, 100, 0.5);
            
          } else if (closestTarget.distance < 0.25 and closestTarget.angleToTarget < 0) {
            // REVERSE IN THE OPPOSITE DIRECTION SLIGHTLY
            carControl(20, 0, 100, 0.3);
            
          } else if (closestTarget.distance < 0.25 and closestTarget.angleToTarget > 0) {
            // REVERSE IN THE OPPOSITE DIRECTION SLIGHTLY
            carControl(-20, 0, 100, 0.3);
            
          } else if (closestTarget.angleToTarget > 0) {
            // CAN IF FAR AND TO THE RIGHT
            driveTime = (closestTarget.angleToTarget / 90) * 1.2; // it takes 1.2 seconds to turn 90 degrees TUNE!
            carControl(20, 75, 0, driveTime);
            driveTime = ((closestTarget.distance - 0.2) / STEADY_SPEED ) - driveTime; // drive straight for the remaiunder
            carControl(0, 75, 0, driveTime);
            
          } else if (closestTarget.angleToTarget < 0) {
            // CAN IF FAR AND TO THE LEFT            
            driveTime = (-1* closestTarget.angleToTarget / 90) * 1.2; // it takes 1.2 seconds to turn 90 degrees TUNE!
            carControl(-20, 75, 0, driveTime);
            driveTime = ((closestTarget.distance - 0.2) / STEADY_SPEED ) - driveTime; // drive straight for the remaiunder
          }
        }
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
    ledcWrite(3, dutycycle);
}
  
