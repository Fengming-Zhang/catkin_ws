#include"example.h"
#include<iostream>
#include<boost/thread.hpp>
#include<ros/ros.h>

#include <unistd.h>


using namespace std;
TestWEConsole::TestWEConsole():WEConsole("Test", boost::bind(&TestWEConsole::oncmd, this, _1))
{
  _name = "Test";
  //Do whatever you want
}

TestWEConsole::TestWEConsole(const char * name):WEConsole(name, boost::bind(&TestWEConsole::oncmd, this, _1))
{
  _name = name;
  //Do whatever you want
}




TestWEConsole::~TestWEConsole()
{
  //Do what ever you want
}

void TestWEConsole::oncmd(const char *cmd)
{

  cout<<_name<<">--"<<cmd<<endl;
  // Your cmd processing funtion
}


int main(int argc, char** argv)
{
  ros::init(argc,argv, "testConsole");
  if(argc == 2)
  {
    TestWEConsole a(argv[1]);
  }
  else
    TestWEConsole a("test");

}
