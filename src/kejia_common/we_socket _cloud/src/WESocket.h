#ifndef WESOCKET_H
#define WESOCKET_H
#include <string.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <we_console/WEConsole.h>
#include <map>
#include "std_msgs/String.h"
#define MAX_BUFFER_LEN 256
#define MAX_MSG_LEN 250
#define HEART_BEAT_MSG 7756231

enum MsgType{ID_PROBLEM_MSG,ID_ANS_MSG};

struct ProMsg
{
    MsgType type;
   char data[MAX_MSG_LEN];
};

struct AnsMsg
{
    MsgType type;
    char data[MAX_MSG_LEN];
};



class WESocket: public WEConsole
{
public:
  WESocket();
  ~WESocket();

  bool useUDP;
  bool bServer;
  bool autoIP;
  std::string IP;
  std::string serverIP;
  int port;

  int sendTimes;

  struct sockaddr_in server_addr;
  struct sockaddr_in client_addr;

  int serverSocket;
  int clientSocket;

  ros::Publisher tts_pub;

  ros::Publisher ansPub;
  ros::Subscriber ansSub;
  void ansSubCallback(const std_msgs::String::ConstPtr& msg);

  ros::Publisher reqPub;
  ros::Subscriber reqSub;
  void reqSubCallback(const std_msgs::String::ConstPtr& msg);
  


   void onCmd(const char* cmd);

  boost::thread* receiveThread;
  boost::recursive_mutex send_lock;

  char recvBuffer[MAX_BUFFER_LEN];
  char buff[MAX_BUFFER_LEN];

  //for server
  bool startServer();

  //for client
  bool connectServer();

  void receiveData();
  bool sendData(void* data, int len);

  bool connected;

  const char* getMsgTypeName(MsgType type);
  const char* getMsgStr(int data);

  bool isNumber(const char* str);
  int selectSocket(int &hSocket, int msec = 100, int sec = 0, bool bRead= true );

  std::map<int, std::string> userCMD;
  void readUserCMD(const char* path);
  void printUserCMD();
  void removeSpace(std::string& str);
};

#endif // WESOCKET_H
