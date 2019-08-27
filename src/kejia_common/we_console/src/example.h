#ifndef EXAMPLE_H
#define EXAMPLE_H
#include<we_console/WEConsole.h>

class TestWEConsole :public WEConsole
{
public:

  TestWEConsole();
  TestWEConsole(const char * name);
  ~TestWEConsole();

  const char * _name;
  void oncmd(const char* cmd);
};


#endif // EXAMPLE_H
