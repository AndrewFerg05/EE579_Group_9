#ifndef SCHEDULER_H_
#define SCHEDULER_H_

//Function to check if events are scheduled for the current cycle
#define IsScheduled(X) ((CurrentTime.s == X.s) && (CurrentTime.ms == X.ms))


//Variable Definitions
typedef struct
{
    int s;
    int ms;
} Time;

extern Time CurrentTime;


//Function Definitions
extern void setupScheduler();
extern Time increment(Time);
extern Time schedule(int durations);

#endif