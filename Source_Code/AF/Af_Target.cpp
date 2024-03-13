#include "AF_Target.h"



void calculateTimeAndAngle(Target* target)
{
    target->timeToTarget = (int)(((target->distance / car_speed) * 1000) - time_offset);    // Tune car speed
    target->angleToTarget = target->angleFromStraight - angle_offset;                      // Replace with your actual conversion
}

void calculateTargets()
{
    for (int i = 0; i < 3; i++) 
    {
        calculateTimeAndAngle(&Targets[i]);
    }
}





long sendUltrasoundPing(int trigger, int echo)
{
  // Set up pins
  pinMode(trigger, OUTPUT);  
  pinMode(echo, INPUT); 

  // Clear the output
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  // Send a 5ms ping
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(triggerPin, LOW);

  // Wait for ping to return 
  return pulseIn(echoPin, HIGH); // NOTE: rewrite this code to not use library
}




void ultrasoundScan() {
  // NOTE: Figure how to setup servos in the new code configuration

  int closestAngle = -1; // The angle, (0 is straight, -ve = left, +ve = right) to the can
  float distanceReadings[180]; // 0 = most right, 180 = most left

  int scanCount = 0; // A second (and third, fourth etc) takes place if the first scan fails
  // up to ten scans

  while (closestAngle == -1 and scanCount < 10){
    // SCAN FOR 180 DEGREES
    for (int i = 0; i <= 180; i--){
        sensorMount.write(sensorMountPin, i);
        delay(5);
        distanceReadings[i] = 0.01723 * sendUltrasoundPing(triggerPin, echoPin);
    }
    
    // PROCESS SCAN
    while(1) {
      double lowerBoundary = -1; // this is used so if there is a false reading, the 2nd lowest, then 3rd lowest etc
      int leftmostAngle = 0; 
      int rightmostAngle = 0;
      int targetAngle = 0;

      // Find the smallest distance in the array, which is greater than the lower boundary
      for (int i = 1; i <= 180; i++) {
        if (distanceReadings[i] < distanceReadings[targetAngle] && distanceReadings[i] > lowerBoundary) {
            targetAngle = i;
        }
      }

      if (targetAngle == 0) {
        // i.e. the closest angle was never updated cos no can was detected, and another scan must take place
        break;
      }

      // Find the maximum distance  within a tolerance of 5cm
      leftmostAngle = targetAngle;
      for (int i = targetAngle+1; i <= 180; i++) {
        double difference = distanceReadings[i] - distanceReadings[i-1];
        if (difference < 0) {
          difference = difference*-1;
        }
        if (difference >= 5) {
          leftmostAngle = i-1;
          break;
        } 
      }

      // Find the lowest distance within a tolerance of 5cm
      rightmostAngle = targetAngle;
      for (int i = targetAngle-1; i >= 0; i--) {
        int difference = sensorValues[i+1] - sensorValues[i];
        if (difference < 0){
          difference = difference*-1;
        }
        
        if (difference >= 2) {
          rightmostAngle = i+1;
          break;
        } 
      }

      double furthestDistance_inLimits = distanceReadings[targetValue];
      double closestDistance_inLimits = distanceReadings[targetValue];
      for (int i = rightmostAngle; i <= leftmostAngle; i++ ) {
        if (distanceReadings[i] > furthestDistance_inLimits){
          furthestDistance_inLimits = distanceReadings[i];
        }
        if (distanceReadings[i] < closestDistance_inLimits) {
          closestDistance_inLimits = distanceReadings[i]; 
        }
      }
      
      if (leftmostAngle - rightmostAngle <= 15) {
        Serial.println("FALSE READING: Angle range was too small");
        lowerBoundary = furthestDistance_inLimits;
      } else if (furthestDistance_inLimits - closestDistance_inLimits > 5){
        Serial.println("");
        Serial.print("FALSE READING: distance range was too big: ");
        Serial.println(furthestDistance_inLimits - closestDistance_inLimits);
        lowerBoundary = furthestDistance_inLimits;
      }  
        else if (leftmostAngle - rightmostAngle >= 75 ){
        Serial.println("");
        Serial.print("FALSE READING: angle range was too big: ");
        Serial.println(leftmostAngle - rightmostAngle);
        lowerBoundary = furthestDistance_inLimits;
      } else {
        // CAN DETECTED!
        // Average min and max to find the centre point of the can
        closestAngle = round((leftmostAngle + rightmostAngle) / 2;)
        
        Serial.println("");
        Serial.print("Closest object is now ");
        Serial.print(distanceReadings[closestAngle]);
        Serial.print("  at an angle: ");
        Serial.println(closestAngle);
    }
    
    }
    scanCount += 1;
  };


  if (closestAngle != -1) {
    for (int i = 180; i >= closestAngle; i--) {
    sensorMount.write(sensorMountPin, i);
    delay(5);
  }
  } else {
    Serial.print("SCAN LIMITED REACHED: No can detected");
  }


}