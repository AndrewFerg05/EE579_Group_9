Calibration steps for IMU

1. Run ICM_20948_get_cal_data.ino on ESP32 - Follow instructions printed to serial monitor
   (Hold still for gyro offsets, and then move VERY SLOWLY AND STEADILY around all axes of freedom)
3. Copy and paste Gyro offset into IMU.cpp
4. Highlight and copy the IMU readings into an excel file
5. In Excel split into columns: Data->Text To Columns -> Delimited - > Comma -> Finish
6. Save 3 left-hand columns as acc3_raw.csv and save right-hand 3 columns as mag3_raw.csv
   (actual file names not important first 3 columns are accelerometer, and second 3 are magnetometer and need to be saved in seperate files)
7. Open calibrate3.py in a python editor
8. Change file name to match accelerometer (acc3_raw.csv) csv file and run
9. Copy both matrices it says to copy in print out into accelerometer offsets in IMU.cpp
10. Change file name to match magnetometer (mag3_raw.csv) csv file and run
11. Copy both matrices it says to copy in print out into magnetometer offsets in IMU.cpp
12. Run Test_IMU_Readings.ino to test output
