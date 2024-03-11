README for AF_system ino baseline version and AF library

Sparkfun ICM_90248 Arduino library is required.

Copy AF folder into libraries directory for Arduino

Contains:
  Sheduler Files - RTOS functions
  Target Files - How to represent targets/waypoints and how to calculate times and angles
  IMU Repackaging - manages all function calls from Sparkfun ICM_90248 Arduino library



AF_System.ino is the main file
Note: All scheduled timings are not what will be in the final system and were for testing 

Will need to comment out IMU calls, setupIMU() updateYaw() and getYaw() in AF_system.ino when running without the IMU connected
