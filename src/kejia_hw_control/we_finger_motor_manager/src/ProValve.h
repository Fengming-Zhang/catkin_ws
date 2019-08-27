#ifndef PROVALVE_H
#define PROVALVE
#include "SerialCom.h"
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <map>
#include "DeviceInterface.h"

#define BYTE unsigned char
#define LIST_LEN 100

class ProVavle
{
public:
	ProVavle(const char * _name, byte id, CSerialCom* com);
	~ProVavle(){}
  void init();
  void debugprint(byte* data);

  byte getId(); //only used in single sensor network!!

  void setPressure(float f1);

  void setPressure_ITV(float f1);

  CSerialCom* com;
  byte id;    
  float pressure;

private:
 
  byte CRC[2];
  void CRC166(unsigned char *puchMsg,unsigned short usDataLen);

};

static BYTE init_message[7] = { 0x01, 0x03, 0x00, 0x00, 0x08, 0x44, 0x0c };
static BYTE init_message_ITV[7] = { 0x02, 0x03, 0x00, 0x00, 0x08, 0x44, 0x0c };

#endif
