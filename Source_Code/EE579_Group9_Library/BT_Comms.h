#ifndef BT_COMMS_H_
#define BT_COMMS_H_

// Function Definitions

extern void setupBluetooth();

extern float getBluetoothReading(int, char); // Ask the user for a distance / angle reading

extern int getBluetoothInputType(); // Ask the user if it is a waypoint / or can

extern int getBluetoothNumberTargets(); // Ask the user for the number of targets

extern int getBluetoothFlag(); // Stall the code until the user says go

// Print Information to phone

extern void BTprintint(int);

extern void BTprintfloat(float);

extern void BTprintError(int);

#endif
