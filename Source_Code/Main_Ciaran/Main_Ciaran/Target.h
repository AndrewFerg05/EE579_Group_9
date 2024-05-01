#ifndef TARGET_H_
#define TARGET_H_

//Tuneable Parameters
#define car_speed 1.2       //m/s
#define time_offset 1000    //Time subtracted so it stops before target
#define angle_offset 0 

// Ultrasound Parameters
#define servoPin 13
#define triggerPin 12
#define echoPin 27

//Variable Definitions
typedef struct {
    float distance;             //  m 
    float angleFromStraight;
    int timeToTarget;           //  ms
    float angleToTarget;
    bool isWaypoint;
} Target;

extern Target Targets[3];
extern Target End_Target;

//Function Definitions
extern void calculateTimeAndAngle(Target*);
extern void calculateTargets(void);

// Ultrasound functions
extern void setupUltrasound(); 
extern double sendUltrasoundPing(); 
extern Target scanForTargets_Ultrasound(); // scans 180 deg, then returns closest target
extern void turnServo(); // controls servo direction
float normalizeTargetAngle(float);
void strikeCanCloseDistance();
void carControl(float, float, float, float);
void steer(int);
void forward(int);
void reverse(int);
#endif
