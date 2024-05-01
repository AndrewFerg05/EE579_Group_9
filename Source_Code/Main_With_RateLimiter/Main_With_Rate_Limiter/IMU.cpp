#include "IMU.h"


//Variables
    //Gyro
float Gscale = (PI / 180.0) * 0.00763; 
float G_offset[3] = {-92.6, 123.6, 196.1}; 


    //Accelerometer
float A_B[3] =  { 312.92, 1196.59, 689.69};
float A_Ainv[3][3] = {
{ 0.05479 , 0.00359 , -0.00116 },
{ 0.00359 , 0.06953 , 0.00076 },
{ -0.00116 , 0.00076 , 0.06205 }};

    //Magnetometer
float M_B[3] =  {-78.71, 290.38, -115.03};

float M_Ainv[3][3] = {
{ 3.2026 , -0.04717 , -0.34181 },
{ -0.04717 , 4.05522 , 0.40158 },
{ -0.34181 , 0.40158 , 2.81148 }};


float declination = -14.84;

  //Quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

  //Raw values
float Gxyz[3], Axyz[3], Mxyz[3];

  //float yaw, pitch, roll;
float yaw, pitch, roll;

  //Quaternion Update Timer Values
unsigned long now = 0, last = 0;
float deltat = 0; 

  //IMU Instance
ICM_20948_I2C imu;

  //Proportional Values
float Kp = 50.0;
float Ki = 0.0;






//Functions
    //Setup
void setupIMU()
{
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  imu.begin(WIRE_PORT, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) 
  {
    Serial.println(F("ICM_90248 not detected"));
    while (1);
  }
}




    //Vector Maths
float vector_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}


    //Quaternion Update and Mahony Filter
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


//update quaternion with integrated contribution
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

    //Get IMU raw values and scaled by calibration values
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) 
{
  byte i;
  float temp[3];

  Gxyz[0] = Gscale * (imu.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (imu.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (imu.agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

  //Update the quaternion if I2C Data is Available
void updateYaw()
{
  if ( imu.dataReady() ) {

    imu.getAGMT();

    
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0

    Mxyz[1] = -Mxyz[1]; //reflect Y and Z
    Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;
    //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);
  }
}

//float average_Yaw[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
//int i = 0;
//int flag = 0;
//
//  //Calculate Yaw
//float getYaw()
//{
//      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
//      yaw   *= 180.0 / PI;
//      yaw = -(yaw + declination);
//      if (yaw < 0) yaw += 360.0;
//      if (yaw >= 360.0) yaw -= 360.0;
//
//      // return yaw;
//
//      
//      if (i==4) flag = 1;
//      if (i==5) i = 0;
//      average_Yaw[i] = yaw;
//      i++;
//      if (flag == 0) return yaw;
//
////      return yaw;
//      return (average_Yaw[0] + average_Yaw[1] + average_Yaw[2] + average_Yaw[3] + average_Yaw[4])/5;
//      
//}


float average_Yaw[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float prev_yaw = 0;
float average = 0;
float wrong_count = 0;
int i = 0;
int flag = 0;

  //Calculate Yaw
//float getYaw()
//{
//      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
//      yaw   *= 180.0 / PI;
//      yaw = -(yaw + declination);
//      if (yaw < 0) yaw += 360.0;
//      if (yaw >= 360.0) yaw -= 360.0;
//
//      // return yaw;
//
//      
//      if (i==4) flag = 1;
//      if (i==5) i = 0;
//      average_Yaw[i] = yaw;
//      i++;
//      if (flag == 0) return yaw;
//
//      return yaw;
////      return (average_Yaw[0] + average_Yaw[1] + average_Yaw[2] + average_Yaw[3] + average_Yaw[4])/5;
//      
//}




float getYaw()
{
    
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      yaw   *= 180.0 / PI;
      yaw = -(yaw + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;
      // return yaw;
     
      if (i==4)
      {
        prev_yaw = (average_Yaw[0] + average_Yaw[1] + average_Yaw[2] + average_Yaw[3] + average_Yaw[4])/5;
        flag = 1;
       
      }
 
      if (i==5) i = 0;
 
      average_Yaw[i] = yaw;
      i++;
      if (flag == 0) return yaw;
 
      average = (average_Yaw[0] + average_Yaw[1] + average_Yaw[2] + average_Yaw[3] + average_Yaw[4])/5;
 
      if(((average - prev_yaw) > 30)||((average - prev_yaw) < -30))
      {
        wrong_count++;
 
        if(wrong_count > 7)
        {
          wrong_count = 0;
          prev_yaw = average;
        }
 
        return prev_yaw;
      }
      else
      {
        wrong_count = 0;
        prev_yaw = average;
        return average;
      }     
}
