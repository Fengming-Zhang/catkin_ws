#ifndef PULLPRESSSENSOR_H
#define  PULLPRESSSENSOR_H
#include "SerialCom.h"
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "DeviceInterface.h"
#define BYTE unsigned char

class PullPressureSensor
{
public :
  PullPressureSensor(const char * _name, byte id, CSerialCom* com);
  ~PullPressureSensor(){}

  void init();
  void debugprint(byte* data);
  byte getId(); 



	CSerialCom* com;
  byte id;
	float GetForce();
	
private:
	byte msg[8];
	byte read[9];
	int check;

  byte CRC[2];
  void CRC166(unsigned char *puchMsg,unsigned short usDataLen);
};

static BYTE request_message[8] = {0x03, 0x03, 0x00, 0x00, 0x00 ,0x02, 0xC5, 0xE9};
#endif
