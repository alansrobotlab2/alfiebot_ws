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

  // Magnetic field data is already converted to Tesla in imuDataGet()
  // No additional conversion needed here
}