#ifndef RS485_H
#define RS485_H
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

class RS485
{
public:
  RS485(const char * _name, byte id, CSerialCom* com);
  ~RS485(){}
  void init();
  void debugprint(byte* data);

  byte getId(); //only used in single sensor network!!
  bool setId(byte newid);
  void getPressure(); //Kpa

  CSerialCom* com;
  byte id;    //new id
  byte last_id; //last setted id
  float pressure;

private:
 
  byte CRC[2];
  void CRC166(unsigned char *puchMsg,unsigned short usDataLen);

};




#endif
