#ifndef WECONSOLE_H
#define WECONSOLE_H
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <histedit.h>

typedef boost::function<void (const char*)> onCmdType;

class WEConsole
{
public:
  WEConsole(const char* consoleName, onCmdType fun);
  ~WEConsole();

protected:
  void stopConsole();

private:
  EditLine *el;
  History *myhist;
  //char *prompt(EditLine *e);

  boost::thread *console_thread;
  volatile bool run_console_thread;
  void console_thread_fun(onCmdType fun);

};



#endif // WECONSOLE_H
