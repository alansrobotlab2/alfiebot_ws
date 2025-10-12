#include "config.h" 
#include "driverboard.h"

extern DriverBoard b;

void getIMUData(){
  //Obtain IMU data
  imuDataGet( &b.stAngles, &b.stGyroRawData, &b.stAccelRawData, &b.stMagnRawData);
  b.board_temp = QMI8658_readTemp();

  b.IMU_Roll  = b.stAngles.roll;
  b.IMU_Pitch = b.stAngles.pitch;
  b.IMU_Yaw   = b.stAngles.yaw;

  // Convert from microTesla (µT) to Tesla (T): 1 µT = 1e-6 T
  b.stMagnRawData.s16X = (float)b.stMagnRawData.s16X * 1e-6f;
  b.stMagnRawData.s16Y = (float)b.stMagnRawData.s16Y * 1e-6f;
  b.stMagnRawData.s16Z = (float)b.stMagnRawData.s16Z * 1e-6f;
}