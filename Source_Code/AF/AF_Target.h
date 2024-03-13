#ifndef AF_TARGET_H_
#define AF_TARGET_H_

//Tuneable Parameters
#define car_speed 2     //  m/s
#define time_offset 1000
#define angle_offset 0 


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


// Sean's Function Definitions
extern void setupServos();
extern void setupUltrasound();
extern double sendUltrasoundPing(triggerPin, echoPin);
extern Target scanForTargets_Ultrasound();


#endif