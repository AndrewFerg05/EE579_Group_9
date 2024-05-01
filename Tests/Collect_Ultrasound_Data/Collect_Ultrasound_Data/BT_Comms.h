#ifndef BT_COMMS_H_
#define BT_COMMS_H_


//Function Definitions
extern void setupBluetooth();
extern float getBluetoothReading(int, char);
extern int getBluetoothInputType(); 
extern int getBluetoothNumberTargets();
extern int getBluetoothFlag();
extern void BTprintint(int);
extern void BTprintfloat(float);
extern void BTprintError(int);

extern float getBTreading(char);

#endif
