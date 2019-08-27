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
#include <we_msgs/Pose.h>
#include <map>
#include <cstdlib>
#include <unistd.h>
#include "std_msgs/String.h"
#define MAX_BUFFER_LEN 256
#define MAX_MSG_LEN 250
#define HEART_BEAT_MSG 7756231

enum MsgType{ID_STATE_MSG, ID_POSE_MSG, ID_HEART_BEAT_MSG,ID_ENV_MSG,ID_TASK_MSG,ID_ACTSEQ_MSG};


struct EnvMsg
{
    MsgType type;
    char data[MAX_MSG_LEN];
};

struct TaskMsg
{
    MsgType type;
    char data[MAX_MSG_LEN];
    char IP[20];
};

struct ActseqMsg
{
    MsgType type;
    char data[MAX_MSG_LEN];
};

struct StateMsg
{
    MsgType type;
    int data;
};

struct HeartBeatMsg
{
    MsgType type;
    int data;
};

struct PoseMsg
{
    MsgType type;
    float x;
    float y;
    float r;
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
  int client[129];

  ros::Publisher worldPub;
  ros::Subscriber worldSub;
  void envSubCallback(const std_msgs::String::ConstPtr& msg);

  ros::Publisher taskPub;
  ros::Subscriber taskSub;
  void taskSubCallback(const std_msgs::String::ConstPtr& msg);

  ros::Publisher actseqPub;
  ros::Subscriber actseqSub;
  void actseqSubCallback(const std_msgs::String::ConstPtr& msg);


  ros::Publisher statePub;
  ros::Subscriber stateSub;
  void stateSubCallback(const std_msgs::Int32ConstPtr& data);

  ros::Publisher posePub;
  ros::Subscriber poseSub;
  void poseSubCallback(const we_msgs::PoseConstPtr& data);

  void onCmd(const char* cmd);

  boost::thread* receiveThread;
  boost::recursive_mutex send_lock;

  char recvBuffer[MAX_BUFFER_LEN];
  char buff[MAX_BUFFER_LEN];

  //for server
  void startServer();
  bool isstartServer;
  //for client
  void connectServer();
  bool isconnectServer;

  void receiveData(int socket);
  void receiveData();
  bool sendData(void* data, int len, int socket);

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
