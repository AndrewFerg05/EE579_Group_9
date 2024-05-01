
//Code to test the Ultrasound (HC-SR04) & corresponding servo
//Requires 

// When started, the ultrasound will scan through 180 degress, taking 180 distance readings
// It then processes the data and returns the smallest target object
// To process the data:
  // 1) Run through the array and find the smallest distance value
  // 2) Iterate through adjacent readings to find the width of the target (with a tolerance of +- 5 cm)
  // 3) Iterate through the width to find the depth of the target
  // 4) If the target is smaller, wider or too deep: it is either not a can or it is a false reading
  // 5) So, set a new lower boundary and repeat steps 1 - 4
  // 6) If there is no valid target in a scan, re-scan and re-process. 
  // 7) If no can is detected after 5 scans, return an error code


//Connections
//ESP32 3V3 - Vin Ultrasound
//ESP32 GND - GND Ultrasound
//ESP32 3V3 - Vin Servo
//ESP32 GND - GND Servo
//ESP32 GPIO02 - Echo Ultrasound 
//ESP32 GPIO04 - Trigger Ultrasound 
//ESP32 GPIO16 / RX2 - Servo Control 


//13/04/24 - Known Issue - the average yaw returned by getYaw() does not transition smoothly from 360 degrees back to 0 degrees


#include "Target.h"
#include "BT_Comms.h"

#define MOTOR_FORWARD_PIN 13
#define MOTOR_REVERSE_PIN 12
#define MOTOR_STEER_PIN 17
#define DRIVE_PWM_FREQ 10
#define STEER_PWM_FREQ 50

//Variables

//Test Print Variables
#define PRINT_SPEED 200       // ms between prints
unsigned long lastPrint = 0;  // Keep track of print time

void setup() 
{
  Serial.begin(9600);
  while(!Serial);
  setupBluetooth();

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

void loop() {
  
  int getBluetooth = getBluetoothFlag();
  turnServo(90);
  getBluetooth = getBluetoothFlag();
  Target closestTarget = scanForTargets_Ultrasound();
  BTprintfloat(closestTarget.angleToTarget);
  BTprintfloat(closestTarget.distance);
  
//  float steering = getBTreading('s');
//  Serial.println(steering);
//  float power = getBTreading('p');
//  float t = getBTreading('t');
//  int getBluetooth = getBluetoothFlag();
   

//  carControl(steering, power, 0, t / 1000);
  
}
