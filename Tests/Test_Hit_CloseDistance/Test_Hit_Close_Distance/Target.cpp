#include "Target.h"
#include <Arduino.h>

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
    // Set-up pins
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Clear  output
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);

    // Send 5ms ping
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(triggerPin, LOW);

    // Wait for pulse to return
    while (digitalRead(echoPin) == LOW);
    long startTime = micros();

    while (digitalRead(echoPin) == HIGH);
    double travelTime = micros() - startTime;

    // return the returning pulse's width
    return travelTime;
}

void setupUltrasoundServo() {
  pinMode(servoPin, OUTPUT); 
}

// Turn ultrasound to angle via servo 
void generatePWM(int angle) {
  int pulseWidth = map(angle, 0, 180, 1000, 2000); // Map angle to pulse width
  digitalWrite(servoPin, HIGH); // Set pin high
  delayMicroseconds(pulseWidth); // Wait for pulse width duration
  digitalWrite(servoPin, LOW); // Set pin low
  sleep(0.005); // Delay for stability 
}

Target scanForTargets_Ultrasound() 
{
    // SETUP
    setupUltrasoundServo();

    // DEFAULT VALUES
    Target closestTarget; // default values
    float distanceReadings[180]; // 0 = most right, 180 = most left

    for (int scanCount = 0; scanCount < 5; scanCount++) // max 5 scans
    {
        double lowerBoundary = -1; // Used so that if there is a false reading, the scan will go to the next lowest distance
        // SCAN  180 DEGREES
        for (int i = 0; i < 180; i++)
        {
            generatePWM(i);
            distanceReadings[i] = 0.01723 * sendUltrasoundPing();
            Serial.println(distanceReadings[i]);
        }

        // PROCESS SCAN
        while (1)
        {
            int leftmostAngle = 0;
            int rightmostAngle = 0;
            int targetAngle = 0;

            // Find the smallest distance in the array (which is greater than the lower boundary)
            for (int i = 1; i <= 180; i++)
            {
                if (distanceReadings[i] < distanceReadings[targetAngle] && distanceReadings[i] > lowerBoundary)
                {
                    targetAngle = i;
                }
            }


            if (targetAngle == 0)
            {
                // i.e. the closest angle was never updated cos no can was detected, and another scan must take place
                break;
            }

            // Find the maximum distance  within a tolerance of 5cm
            leftmostAngle = targetAngle;
            for (int i = targetAngle + 1; i <= 180; i++)
            {
                double difference = distanceReadings[i] - distanceReadings[i - 1];
                if (difference < 0)
                {
                    difference = difference * -1;
                }
                if (difference >= 5)
                {
                    leftmostAngle = i - 1;
                    break;
                }
            }

            // Find the lowest distance within a tolerance of 5cm
            rightmostAngle = targetAngle;
            for (int i = targetAngle - 1; i >= 0; i--)
            {
                int difference = distanceReadings[i + 1] - distanceReadings[i];
                if (difference < 0)
                {
                    difference = difference * -1;
                }

                if (difference >= 5)
                {
                    rightmostAngle = i + 1;
                    break;
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

            if (leftmostAngle - rightmostAngle <= 5)
            {
                
                Serial.println("False Reading: Angle range was too small");
                lowerBoundary = furthestDistance_inLimits;
            }
            else if (furthestDistance_inLimits - closestDistance_inLimits >= 15)
            {
                Serial.println("False Reading: distance range was too big: ");
                lowerBoundary = furthestDistance_inLimits;
            }
            else if (leftmostAngle - rightmostAngle >= 90)
            {
                Serial.println("False Reading: angle range was too big: ");
                lowerBoundary = furthestDistance_inLimits;
            }
            else
            {
                // CAN DETECTED!
                // Average min and max to find the centre point of the can,
                closestTarget.angleToTarget = round((leftmostAngle + rightmostAngle) / 2);
                closestTarget.distance = distanceReadings[static_cast<int>(closestTarget.angleToTarget)];
                // turn ultrasound to face can
                for (int i = 179; i >= static_cast<int>(closestTarget.angleToTarget); i--) {
                  generatePWM(i);
                }
                // then move angle to the correct range, from 0 to 180 to -90 to 90
                closestTarget.angleToTarget = closestTarget.angleToTarget - 90;
                Serial.println("FINAL ANGLE: ");
                Serial.println(static_cast<int>(closestTarget.angleToTarget));
                Serial.println("FINAL DISTANCE: ");
                Serial.println(static_cast<int>(closestTarget.distance));
                
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