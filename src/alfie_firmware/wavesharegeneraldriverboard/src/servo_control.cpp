#include "servo_control.h"
#include "config.h"
#include "driverboard.h"
#include "utils.h"

extern DriverBoard b;



void updateServoStatus()
{

  b.st.syncReadPacketTx(b.IDS, NUMSERVOS, SBS_CURRENTLOCATION, 13);

  int RxResult = 0;

  for (int i = 0; i < NUMSERVOS; i++)
  {
    RxResult = b.st.syncReadPacketRx((i + 1), b.buf.bytes + SBS_CURRENTLOCATION);
    if (RxResult > 0)
    {
      memcpy(b.mBuf[i].bytes + SBS_CURRENTLOCATION, b.buf.bytes + SBS_CURRENTLOCATION, 13);
    }
    else
    {
      flushSerialServoLine();
      return;
    }
  }
}

  /*
    So this is going to be two part operation.
    iterate through the servo list one time and build a list of servo ids
    that have a torque switch set to 0.

    then do a sync write to update the torque switch to zero on those servos
    to disable them immediately.

    then iterate through the servo list again and build a list of servo ids
    that have a torque switch set to 1 and populate the msg buffer 
    with acceleration and position info.

    then do a sync write to update the position and acceleration on those servos
    acknowledging that any change to those values automatically sets the torque switch to 1
  */

void updateServoIdle()
{
  int index = 0;

  // update torque switch for disabled servos
  for (int i = 0; i < NUMSERVOS; i++)
  {
    if (b.mBuf[i].memory.torqueSwitch == 0)
    {
      b.torquecommandbuf[index] = 0;
      b.servoCMDIDS[index] = (i + 1);
      index++;
    }
  }
  b.st.syncWrite(b.servoCMDIDS, index, SBS_TORQUEENABLE, b.torquecommandbuf, 1);
}


void updateServoActive()
{
  int index = 0;

  // selectively update position based on the torque switch == 1
  for (int i = 0; i < NUMSERVOS; i++)
  {
    if (b.mBuf[i].memory.torqueSwitch == 1)
    {
      b.servocommandbuf[(2 * index) + 0] = (b.mBuf[i].memory.targetLocation) & 0xFF;
      b.servocommandbuf[(2 * index) + 1] = (b.mBuf[i].memory.targetLocation >> 8) & 0xFF;
      b.servoCMDIDS[index] = (i + 1);
      index++;
    }
  }

  if (index > 0)
  {
    b.st.syncWrite(b.servoCMDIDS, index, SBS_TARGETLOCATION, b.servocommandbuf, 2);
  }
}

void updateServoActive_BAD()
{
  int index = 0;

  // selectively update position based on the torque switch == 1
  for (int i = 0; i < NUMSERVOS; i++)
  {
    if (b.mBuf[i].memory.torqueSwitch == 1)
    {
      b.servocommandbuf[(3 * index) + 0] = b.mBuf[i].bytes[SBS_ACCELERATION];
      b.servocommandbuf[(3 * index) + 1] = b.mBuf[i].bytes[SBS_TARGETLOCATION + 0];
      b.servocommandbuf[(3 * index) + 2] = b.mBuf[i].bytes[SBS_TARGETLOCATION + 1];
      b.servoCMDIDS[index] = (i + 1);
      index++;
    }
  }

  if (index > 0)
  {
    b.st.syncWrite(b.servoCMDIDS, index, 41, b.servocommandbuf, 3);
  }
}

void disableAllServoTorques()
{
  for (int i = 0; i < NUMSERVOS; i++)
  {
    b.mBuf[i].memory.torqueSwitch = 0;
  }
}


/*
  the value from the servo is a 12 bit signed value in the range of -2048 to 2047
  so we convert to a formal 16 bit signed value
*/
int16_t positioncorrectionfrom12bitservo(int16_t servovalue)
{

  if(servovalue & 0x800)
  {
    return -(servovalue - 0x800);
  }
  else
  {
    return servovalue;
  }
}

/*
  the value from to the servo is a 12 bit signed value in the range of -2048 to 2047
  so we convert from a formal 16 bit signed value to a 12 bit signed value
*/
int16_t positioncorrectionto12bitservo(int16_t controllervalue)
{
  if (controllervalue < 0)
  {
    return (0x800 + abs(controllervalue));
  }
  else
  {
    return controllervalue;
  }
}


/*
  The only thing I could figure out in order to interrupt
  the sync comms and get everything reset.
  Otherwise the packets get out of sync and the 8ms loop goes to 120ms
  and all the data is wrong.
  It works.
*/
void flushSerialServoLine()
{
  for (int e = 0; e < NUMSERVOS; e++)
  {
    b.st.Ping(e + 1);
  }
}
