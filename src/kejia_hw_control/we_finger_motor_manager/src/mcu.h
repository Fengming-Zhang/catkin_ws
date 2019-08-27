#ifndef MCU_H
#define MCU_H
#include "SerialCom.h"
#include "ros/ros.h"

#define PUMP_ON 1
#define PUMP_OFF 0
class mcu
{
public:
    mcu(const char* name, CSerialCom* com);
    ~mcu();
    int set_pump();
    int close_pump();

 private:
   CSerialCom* com;
   int pump_status;
};
#endif // MCU_H
