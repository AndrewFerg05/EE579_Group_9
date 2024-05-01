#ifndef CARCONTROL_H_
#define CARCONTROL_H_

#define accelerating_time 0.2    // Time added for acceleration at small distances
#define MOTOR_FORWARD_PIN 13
#define MOTOR_REVERSE_PIN 12
#define MOTOR_STEER_PIN 17
#define DRIVE_PWM_FREQ 10
#define STEER_PWM_FREQ 50


extern void setupDrive();

extern void strikeCanCloseDistance(); // A decision tree that moves car to can based on ultrasound scans
  
extern void carControl(float, float, float, float); // drive car in one function: steering, forward power, reverse power, duration

extern void forward(int); // drive car forward

extern void reverse(int); // drive car backwards

extern void steer(int); // steer (min / max of approx. 20 deg)


#endif
