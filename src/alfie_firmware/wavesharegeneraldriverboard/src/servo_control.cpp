#include "servo_control.h"
#include "config.h"
#include "driverboard.h"
#include "utils.h"

extern DriverBoard b;



void updateServoStatus()
{
  uint8_t numServos = b.getNumServos();

  b.st.syncReadPacketTx(b.IDS, numServos, SBS_CURRENTLOCATION, 13);

  int RxResult = 0;

  for (int i = 0; i < numServos; i++)
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
  uint8_t numServos = b.getNumServos();

  // update torque switch for disabled servos
  for (int i = 0; i < numServos; i++)
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


// one 16-digit number split into two 8-digit numbers
// DataL is low, DataH is high
void Host2SCS(u8 *DataL, u8* DataH, u16 Data)
{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);

}

uint16_t Host2SCSPosition(int16_t position)
{
  uint16_t servo_position = 0;

		if(position < 0){
			servo_position = -position;
			servo_position |= (1<<15);
		}
    else {
      servo_position = position & 0x7FFF;
    }

  return servo_position;
}



void updateServoActive()
{
  int index = 0;
  uint8_t numServos = b.getNumServos();

  // selectively update position based on the torque switch == 1
  for (int i = 0; i < numServos; i++)
  {
    if (b.mBuf[i].memory.torqueSwitch == 1)
    {
      b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 0] = (b.mBuf[i].memory.acceleration) & 0xFF;
      Host2SCS(&b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 1], &b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 2], Host2SCSPosition(b.mBuf[i].memory.targetLocation));
      Host2SCS(&b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 3], &b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 4], 0);
      Host2SCS(&b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 5], &b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 6], b.mBuf[i].memory.runningSpeed);
      Host2SCS(&b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 7], &b.servocommandbuf[(SERVOCOMMANDPACKETSIZE * index) + 8], b.mBuf[i].memory.torqueLimit);

      b.servoCMDIDS[index] = (i + 1);
      index++;
    }
  }

  if (index > 0)
  {
    b.st.syncWrite(b.servoCMDIDS, index, SBS_TARGETLOCATION, b.servocommandbuf, SERVOCOMMANDPACKETSIZE);
  }
}

void updateServoActive_SIMPLE()
{
  int index = 0;
  uint8_t numServos = b.getNumServos();

  // selectively update position based on the torque switch == 1
  for (int i = 0; i < numServos; i++)
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
  uint8_t numServos = b.getNumServos();

  // selectively update position based on the torque switch == 1
  for (int i = 0; i < numServos; i++)
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
  uint8_t numServos = b.getNumServos();
  for (int i = 0; i < numServos; i++)
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
  uint8_t numServos = b.getNumServos();
  for (int e = 0; e < numServos; e++)
  {
    b.st.Ping(e + 1);
  }
}
