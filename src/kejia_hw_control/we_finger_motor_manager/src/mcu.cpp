#include "mcu.h"

mcu::mcu(const char *name, CSerialCom *com)
{
    this->com=com;
    pump_status=PUMP_OFF;
}

int mcu::set_pump()
{
    int len;
    char* cmd="pump on";
    len=com->PushData(cmd,7);
    pump_status=PUMP_ON;
    return len;
}

int mcu::close_pump()
{
    int len;
    char* cmd="pump off";
    len=com->PushData(cmd,8);
    pump_status=PUMP_OFF;
    return len;
}
