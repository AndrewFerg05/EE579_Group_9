#include "SD_Comms.h"
#include "BluetoothSerial.h"

BluetoothSerial serialBT;


void setupBluetooth() {
  serialBT.begin("ESP32-BT");
}


char getBluetoothReading(int N) {

 char cmd;

  if (serialBT.available()){
    cmd = serialBT.read();
    return cmd;
  } else {
    return '1';
  }



  // if (cmd == '1') {
  //   digitalWrite(12, HIGH);
  //   runScan();
  // } else if (cmd == '0') {
  //   digitalWrite(12, LOW);
  // }


}


