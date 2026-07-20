/*
 * SCSerial.h
 * hardware interface layer for waveshare serial bus servo
 * date: 2023.6.28
 */


#include "SCSerial.h"

SCSerial::SCSerial()
{
	IOTimeOut = 100;
	pSerial = NULL;
}

SCSerial::SCSerial(u8 End):SCS(End)
{
	IOTimeOut = 100;
	pSerial = NULL;
}

SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
	IOTimeOut = 100;
	pSerial = NULL;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
	int Size = 0;
	int ComData;
	unsigned long t_begin = millis();
	unsigned long t_user;
	while(1){
		ComData = pSerial->read();
		if(ComData!=-1){
			if(nDat){
				nDat[Size] = ComData;
			}
			Size++;
			t_begin = millis();
		}
		if(Size>=nLen){
			break;
		}
		t_user = millis() - t_begin;
		if(t_user>IOTimeOut){
			break;
		}
	}
	return Size;
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}

int SCSerial::writeSCS(unsigned char bDat)
{
	return pSerial->write(&bDat, 1);
}

void SCSerial::rFlushSCS()
{
	while(pSerial->read()!=-1);
}

void SCSerial::wFlushSCS()
{
	// Block until the UART has physically shifted out the whole packet. On the
	// RP2040 earlephilhower core writeSCS() only queues into the TX ring buffer
	// and returns, so without this we start reading the half-duplex bus before
	// the request has left the wire and the auto-direction adapter drops the
	// servo replies. (The stock Feetech library flushes here; it was a no-op.)
	pSerial->flush();
}