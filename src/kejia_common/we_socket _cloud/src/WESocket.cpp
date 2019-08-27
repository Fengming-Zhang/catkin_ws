#include "WESocket.h"
#include <fstream>
#define PEEK_CMD_SL(cmd, s, n, str) \
        (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%[^!]", str) == 1)

int sendCount =  0;
int receiveCount = 0;
using namespace std;
WESocket::WESocket():
  WEConsole("socket", boost::bind(&WESocket::onCmd, this, _1))
{
    ros::NodeHandle pri_nh("~");
    pri_nh.param("useUDP", useUDP, false);
    pri_nh.param("bServer", bServer, false);
    pri_nh.param("autoIP", autoIP, false);

    if(!autoIP)
        pri_nh.param("IP", IP, string(""));

    if(!bServer)
        pri_nh.param("serverIP", serverIP, string(""));

    pri_nh.param("port", port, 33533);

    //string path;
    //pri_nh.param("user_cmd_file", path, string(""));
    //readUserCMD(path.c_str());

    pri_nh.param("sendTimes", sendTimes, 1);
    sendTimes = sendTimes < 1 ? 1 : sendTimes;

    serverSocket = -1;
    clientSocket = -1;
    receiveThread = NULL;
    connected = false;


    
    
    tts_pub = pri_nh.advertise<std_msgs::String>("tts", 1000);

    reqPub = pri_nh.advertise<std_msgs::String>("/requestforserver", 10);
    reqSub = pri_nh.subscribe<std_msgs::String>("/clientrequest", 10, &WESocket::reqSubCallback, this);

    ansPub = pri_nh.advertise<std_msgs::String>("/answerforclient", 10);
    ansSub = pri_nh.subscribe<std_msgs::String>("/serveranswer", 10, &WESocket::ansSubCallback, this);






    if(bServer && startServer())
      receiveThread = new boost::thread(boost::bind(&WESocket::receiveData, this ));

    if(!bServer && connectServer())
      receiveThread = new boost::thread(boost::bind(&WESocket::receiveData, this ));


}


WESocket::~WESocket()
{
  printf("Send Num: %d, Receive Num: %d\n", sendCount, receiveCount);

  if(receiveThread)
  {
      receiveThread->join();
      delete receiveThread;
      receiveThread = NULL;
  }

  if(serverSocket > 0)
  {
    close(serverSocket);
    serverSocket = -1;
  }

  if(clientSocket > 0)
  {
    close(clientSocket);
    clientSocket = -1;
  }
}


bool WESocket::startServer()
{
    bzero((void*)&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    if(autoIP)
        server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    else if(inet_aton(IP.c_str(), &server_addr.sin_addr) == 0)
    {
        ROS_ERROR("IP Address Error! [%s]", IP.c_str());
        return false;
    }
    server_addr.sin_port = htons(port);


    serverSocket = socket(PF_INET, SOCK_STREAM, 0);
    if(serverSocket < 0)
    {
        ROS_ERROR("Create Socket Failed!");
        return false;
    }
    int opt =1;
    setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));

    if(bind(serverSocket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
    {
        ROS_ERROR("Server Bind Port [%s:%d] Error!", inet_ntoa(server_addr.sin_addr), port);
        return false;
    }
    ROS_INFO("Server Listening on [%s:%d]", inet_ntoa(server_addr.sin_addr), port);

    if(listen(serverSocket, 128))
    {
        ROS_ERROR("Server Listen Failed!");
        return false;
    }

    ROS_INFO("Waiting...");
    while(ros::ok())
    {
        int re = selectSocket(serverSocket, 0, 5);

        if(re < 0)
        {
            ROS_ERROR("Server Select Error!");
            return false;
        }
        else if(re == 0)
        {
            ROS_INFO("Still Waiting...");
            continue;
        }

        socklen_t len = sizeof(client_addr);
        clientSocket = accept(serverSocket, (struct sockaddr*)&client_addr, &len);
        if(clientSocket < 0)
        {
            ROS_ERROR("Server Accept Error!");
            return false;
        }

        connected = true;
        ROS_INFO("Connected! Client IP:[%s:%d]", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        return true;
    }
    return false;
}

bool WESocket::connectServer()
{
    bzero((void*)&client_addr, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    if(autoIP)
        client_addr.sin_addr.s_addr = htons(INADDR_ANY);
    else if(inet_aton(IP.c_str(), &client_addr.sin_addr) == 0)
    {
        ROS_ERROR("IP Address Error![%s]", IP.c_str());
        return false;
    }
    client_addr.sin_port = htons(0);

    clientSocket = socket(AF_INET,SOCK_STREAM,0);
    if( clientSocket < 0)
    {
        ROS_ERROR("Create Socket Failed!");
        return false;
    }

    if(bind(clientSocket, (struct sockaddr*)&client_addr, sizeof(client_addr)))
    {
        ROS_ERROR("Bind Port Error!");
        return false;
    }
    else
    {
        ROS_INFO("Local IP: [%s:%d]", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
    }

    bzero((void*)&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if(inet_aton(serverIP.c_str(), &server_addr.sin_addr) == 0)
    {
        ROS_ERROR("Server IP Address Error [%s]!", serverIP.c_str());
        return false;
    }

    socklen_t len = sizeof(server_addr);
    int count = 0;
    while(ros::ok())
    {
      if(connect(clientSocket, (struct sockaddr*)&server_addr, len) < 0)
      {
        count++;
        if((count % 50) == 1)
            ROS_INFO("Connecting to Server [%s:%d]...", serverIP.c_str(), port);
        usleep(100000);
      }
      else
      {
        connected = true;
        ROS_INFO("Connected! Server IP:[%s:%d].", serverIP.c_str(), port);

        return true;
      }
    }
    printf("stop\n");
    return false;
}


void WESocket::receiveData()
{
  ROS_INFO("Receive Thread Start...");

  while(ros::ok())
  {
    if(!connected)
    {
        ROS_INFO("Restarting connection...");

        if(serverSocket > 0)
        {
          close(serverSocket);
          serverSocket = -1;
        }

        if(clientSocket > 0)
        {
          close(clientSocket);
          clientSocket = -1;
        }

        bool re;
        if(bServer)
          re = startServer();
        else
          re = connectServer();
        if(!re)
            continue;
    }


    unsigned int len =0;
    unsigned int revDateCount = 0;
    int re = selectSocket(clientSocket, 100);
    if(re < 0)
    {
      ROS_ERROR("Receive Thread Select Error!");
      connected = false;
      continue;
    }
    else if(re == 0)
      continue;
    else
    {
      len  = recv(clientSocket, (void*)recvBuffer, MAX_BUFFER_LEN, 0);
      if(len > 0)
      {
        memcpy((void*)(buff + revDateCount), (void*)recvBuffer, len);
        revDateCount += len;
      }
      else
      {
        ROS_ERROR("Receive Data Error!");
        connected = false;
        goto __reconnect__;
      }


      while(revDateCount < sizeof(MsgType))
      {
        re = selectSocket(clientSocket, 100);
        if(re < 0)
        {
          ROS_ERROR("Receive Thread Select Error!");
          connected = false;
          continue;
        }
        else if(re > 0)
        {
          len  = recv(clientSocket, (void*)recvBuffer, MAX_BUFFER_LEN, 0);
          if(len > 0)
          {
            memcpy((void*)(buff + revDateCount), (void*)recvBuffer, len);
            revDateCount += len;
          }
          else
          {
            ROS_ERROR("Receive Data Error--2!");
            connected = false;
            goto __reconnect__;
          }
        }

      }

      MsgType type;
      memcpy((void*)&type, (void*)buff, sizeof(MsgType));

      unsigned int totalLen = 0;
      if(type == ID_ANS_MSG)
        totalLen = sizeof(AnsMsg);
      else if(type == ID_PROBLEM_MSG)
        totalLen = sizeof(ProMsg);
      



      while(revDateCount < totalLen)
      {
        re = selectSocket(clientSocket, 100);
        if(re < 0)
        {
          ROS_ERROR("Receive Thread Select Error!");
          connected = false;
          continue;
        }
        else if(re > 0)
        {
          len  = recv(clientSocket, (void*)recvBuffer, MAX_BUFFER_LEN, 0);
          if(len > 0)
          {
            memcpy((void*)(buff + revDateCount), (void*)recvBuffer, len);
            revDateCount += len;
          }
          else
          {
            ROS_ERROR("Receive Data Error--3!");
            connected = false;
            goto __reconnect__;
          }
        }
      }



      if(type == ID_ANS_MSG)
      {
        AnsMsg* msg = (AnsMsg*)buff;
        std_msgs::String cmd;
        cmd.data = msg->data;
        ansPub.publish(cmd);
        ROS_INFO("Receive ans Msg [%s]", msg->data);
        receiveCount++;
      }
      else if(type == ID_PROBLEM_MSG)
      {
        ProMsg* msg = (ProMsg*)buff;
        std_msgs::String cmd;
        cmd.data = msg->data;
        reqPub.publish(cmd);
        ROS_INFO("Receive pro Msg [%s]", msg->data);
        receiveCount++;

      }


    }
    __reconnect__:;
  }
  ROS_INFO("Receive Thread Stop.");
}


bool WESocket::sendData(void *data, int len)
{
    if(!connected)
        return false;
  send_lock.lock();
  int sendLen = send(clientSocket, data, len, 0);
  send_lock.unlock();


  MsgType type;

  memcpy((void*)&type, data, sizeof(MsgType));

  if(type == ID_ANS_MSG)
  {
      AnsMsg msg;
      memcpy((void*)&msg, data, len);
      if(sendLen != len)
      {
          ROS_ERROR("Send  Ans Error![msg: (%s) send len: %d]",msg.data, sendLen);
          connected = false;
          return false;
      }
      else
        ROS_INFO("Send ans Msg [%s]", msg.data);
  }
  else if(type == ID_PROBLEM_MSG)
  {
      ProMsg msg;
      memcpy((void*)&msg, data, len);
      if(sendLen != len)
      {
          ROS_ERROR("Send problem Msg Error![msg: (%s) send len: %d]",msg.data, sendLen);
          connected = false;
          return false;
      }
      else
        ROS_INFO("Send problem Msg [%s]", msg.data);
  }

  sendCount++;
  return sendLen == len;
}


void WESocket::reqSubCallback(const std_msgs::String::ConstPtr& msge)
{
    cout << "call back " <<endl;
    string str(msge->data);

    ProMsg msg;
    msg.type = ID_PROBLEM_MSG;

    for(int i = 0; i < str.length(); i++)
        msg.data[i]=str.at(i);
    msg.data[str.length()+1]='!';
    for(int i = 0; i < sendTimes; i++)
    {
      sendData((void*)&msg, sizeof(msg));
      if(sendTimes > 1)
          usleep(100000);
    }


}


void WESocket::ansSubCallback(const std_msgs::String::ConstPtr& msge)
{
  string str(msge->data);
  AnsMsg msg;
  msg.type = ID_ANS_MSG;
  
  for(int i = 0; i < str.length(); i++)
      msg.data[i]=str.at(i);
  msg.data[str.length()+1]='!';
  for(int i = 0; i < sendTimes; i++)
  {
    sendData((void*)&msg, sizeof(msg));
    if(sendTimes > 1)
        usleep(100000);
  }
}



void WESocket::onCmd(const char *cmd)
{
  string c(cmd);
  removeSpace(c);

  cmd = c.c_str();
  char s[MAX_MSG_LEN];


  if(PEEK_CMD_SL(cmd,"sp",2,s))
  {
    ProMsg msg;
    msg.type = ID_PROBLEM_MSG;
    for(int i= 0 ; i < MAX_MSG_LEN; i++)
        msg.data[i] = s[i];

    for(int i = 0; i < sendTimes; i++)
    {
        sendData((void*)&msg, sizeof(msg));
        if(sendTimes > 1)
            usleep(100000);
    }
  }
  else if(PEEK_CMD_SL(cmd, "rp", 2, s))
  {
    std_msgs::String cmd;
    cmd.data=s;
    reqPub.publish(cmd);
    ROS_INFO("Receive request Msg [%s]", s);

  }
  
  else if (PEEK_CMD_SL(cmd, "sa", 2, s))
  {
    AnsMsg msg;
    msg.type = ID_ANS_MSG;
    for(int i = 0; i < MAX_MSG_LEN; i++)
      msg.data[i]=s[i];

    for(int i = 0; i < sendTimes; i++)
    {
      sendData((void*)&msg, sizeof(msg));
      if(sendTimes > 1)
          usleep(100000);
    }
  }
   else if(PEEK_CMD_SL(cmd, "ra", 2, s))
  {
    std_msgs::String cmd;
    cmd.data=s;
    ansPub.publish(cmd);
    ROS_INFO("Receive task Msg [%s]", s);

  }
  
  else
  {
    printf("Unknown CMD [%s]\n", cmd);

  }



}


bool WESocket::isNumber(const char *str)
{
  if (str == NULL)
     return false;

  int len = strlen(str);
  if ( len == 0)
     return false;

  for (int i = 0; i < len; i++)
  {
    if(i == 0 && (str[i] == '-' || str[i] == '+' || isdigit(str[i])))
      continue;
    if (!isdigit(str[i]))
      return false;
  }
  return true;
}

const char* WESocket::getMsgTypeName(MsgType type)
{
    switch(type)
    {

    case ID_PROBLEM_MSG: return "problem Msg";
    case ID_ANS_MSG: return "ans Msg";
    default: return "Unknow Msg Type";
    }
}

const char* WESocket::getMsgStr(int data)
{
    return "haha";
}

int WESocket::selectSocket(int &hSocket, int msec, int sec, bool bRead)
{
    fd_set fdset;
    struct timeval tv;
    FD_ZERO(&fdset);
    FD_SET(hSocket, &fdset);
    msec = msec > 1000 ? 1000 : msec;
    tv.tv_sec  = sec;
    tv.tv_usec = msec * 1000;

    int iRet = 0;
    if ( bRead ) {
        iRet = select(hSocket + 1, &fdset, NULL , NULL, &tv);
    }else{
        iRet = select(hSocket + 1, NULL , &fdset, NULL, &tv);
    }
    return iRet;
}

void WESocket::removeSpace(string &str)
{
  size_t start = str.find_first_not_of(' ');
  size_t end = str.find_last_not_of(' ');
  if(start != str.npos && end != str.npos)
    str = str.substr(start, end - start + 1);
}



































