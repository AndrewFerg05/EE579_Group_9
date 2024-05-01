#ifndef TARGET_H_
#define TARGET_H_

//Tuneable Parameters
#define car_speed 1.2      // Car's velocity after acceleration
#define car_slow_speed 0.5 // Car's avg. velocity over short distances
#define time_offset 100    // Time subtracted so it stops before target
#define angle_offset 0
// Ultrasound Pins
#define servoPin 13     
#define triggerPin 12
#define echoPin 27

//Variable Definitions
typedef struct {
    float distance;             //  m 
    float angleFromStraight;    // deg
    int timeToTarget;           //  ms
    float angleToTarget;        // deg
    bool isWaypoint;
} Target;

extern Target Targets[3];
extern Target End_Target;

// Navigation Functions
extern void calculateTimeAndAngle(Target*);
extern void calculateTime(Target*);
extern void calculateTargets(void);

// Ultrasound functions
extern void setupUltrasound(); 
extern double sendUltrasoundPing(); 
extern Target scanForTargets_Ultrasound(); // scans 180 deg, then returns closest target
extern void turnServo(); // controls servo direction
extern float normalizeTargetAngle(float); // Ensure angle is between 0 and 360
extern bool collisionTest(); // Check if the car is about to hit an object

#endif
