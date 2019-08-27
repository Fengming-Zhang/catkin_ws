#include "WESocket.h"
#include <fstream>
#include <we_msgs/xform.h>

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

    string path;
    pri_nh.param("user_cmd_file", path, string(""));
    readUserCMD(path.c_str());

    pri_nh.param("sendTimes", sendTimes, 1);
    sendTimes = sendTimes < 1 ? 1 : sendTimes;

    serverSocket = -1;
    clientSocket = -1;
    receiveThread = NULL;
    connected = false;


    


    worldPub = pri_nh.advertise<std_msgs::String>("/cloudworld_s", 10);
    worldSub = pri_nh.subscribe<std_msgs::String>("/cloudworld_c", 10, &WESocket::envSubCallback, this);

    taskPub = pri_nh.advertise<std_msgs::String>("/tasksemi_s", 10);
    taskSub = pri_nh.subscribe<std_msgs::String>("/tasksemi_c", 10, &WESocket::taskSubCallback, this);

    actseqPub = pri_nh.advertise<std_msgs::String>("/actionsequence_c", 10);
    actseqSub = pri_nh.subscribe<std_msgs::String>("/actionsequence_s", 10, &WESocket::actseqSubCallback, this);

    statePub = pri_nh.advertise<std_msgs::Int32>("socket_state", 10);
    stateSub = pri_nh.subscribe<std_msgs::Int32>("socket_state_cmd", 10, &WESocket::stateSubCallback, this);

    posePub = pri_nh.advertise<we_msgs::Pose>("socket_pose", 10);
    poseSub = pri_nh.subscribe<we_msgs::Pose>("socket_pose_cmd", 10, &WESocket::poseSubCallback, this);

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
        printUserCMD();
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
        printUserCMD();
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
      if(type == ID_STATE_MSG)
        totalLen = sizeof(StateMsg);
      else if(type == ID_POSE_MSG)
        totalLen = sizeof(PoseMsg);
      else if(type == ID_ENV_MSG)
        totalLen = sizeof(EnvMsg);
      else if(type == ID_TASK_MSG)
        totalLen = sizeof(TaskMsg);
      else if(type == ID_ACTSEQ_MSG)
        totalLen = sizeof(ActseqMsg);



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


      if(type == ID_STATE_MSG)
      {
        StateMsg* msg = (StateMsg*)buff;
        std_msgs::Int32 cmd;
        cmd.data = msg->data;
        statePub.publish(cmd);
        ROS_INFO("Receive State Msg [%d]", msg->data);
        receiveCount++;

      }
      else if(type == ID_POSE_MSG)
      {
        PoseMsg* msg = (PoseMsg*)buff;
        we_msgs::Pose cmd;
        cmd.x = msg->x;
        cmd.y = msg->y;
        cmd.r = msg->r;
        posePub.publish(cmd);
        ROS_INFO("Receive Pose Msg [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->r);
        receiveCount++;
      }
      else if(type == ID_ENV_MSG)
      {
        EnvMsg* msg = (EnvMsg*)buff;
        std_msgs::String cmd;
        cmd.data = msg->data;
        worldPub.publish(cmd);
        ROS_INFO("Receive env Msg [%s]", msg->data);
        receiveCount++;

      }


      else if(type == ID_TASK_MSG)
      {
        TaskMsg* msg = (TaskMsg*)buff;
        std_msgs::String cmd;
         cmd.data=msg->data;
        taskPub.publish(cmd);
        ROS_INFO("Receive task Msg [%s]", msg->data);
        receiveCount++;

      }


      else if(type == ID_ACTSEQ_MSG)
      {
        ActseqMsg* msg = (ActseqMsg*)buff;
        std_msgs::String cmd;
         cmd.data=msg->data;
        actseqPub.publish(cmd);
        ROS_INFO("Receive Actseq Msg [%s]", msg->data);
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

  if(type == ID_STATE_MSG)
  {
      StateMsg msg;
      memcpy((void*)&msg, data, len);
      if(sendLen != len)
      {
        ROS_ERROR("Send State Msg Error![msg: (%d)%s send len: %d]",msg.data,getMsgStr(msg.data), sendLen);
        connected = false;
        return false;
      }
      else
        ROS_INFO("Send State Msg [%d]", msg.data);
  }
  else if(type == ID_POSE_MSG)
  {
      PoseMsg msg;
      memcpy((void*)&msg, data, len);
      if(sendLen != len)
      {
          ROS_ERROR("Send Pose Msg Error![msg: (%.2f, %.2f, %.2f) send len: %d]",msg.x, msg.y, msg.r, sendLen);
          connected = false;
          return false;
      }
      else
        ROS_INFO("Send Pose Msg [%.2f, %.2f, %.2f]",msg.x, msg.y, msg.r);
  }
  else if(type == ID_ENV_MSG)
  {
      EnvMsg msg;
      memcpy((void*)&msg, data, len);
      if(sendLen != len)
      {
          ROS_ERROR("Send env Msg Error![msg: (%s) send len: %d]",msg.data, sendLen);
          connected = false;
          return false;
      }
      else
        ROS_INFO("Send env Msg [%s]", msg.data);
  }
  else if(type == ID_TASK_MSG)
  {
      TaskMsg msg;
      memcpy((void*)&msg, data, len);
      if(sendLen != len)
      {
          ROS_ERROR("Send task Msg Error![msg: (%s) send len: %d]",msg.data, sendLen);
          connected = false;
          return false;
      }
      else
        ROS_INFO("Send task Msg [%s]", msg.data);
  }
  else if(type == ID_ACTSEQ_MSG)
  {
      ActseqMsg msg;
      memcpy((void*)&msg, data, len);
      if(sendLen != len)
      {
          ROS_ERROR("Send actseq Msg Error![msg: (%s) send len: %d]",msg.data, sendLen);
          connected = false;
          return false;
      }
      else
        ROS_INFO("Send actseq Msg [%s]", msg.data);
  }
  sendCount++;
  return sendLen == len;
}


void WESocket::envSubCallback(const std_msgs::String::ConstPtr& msge)
{
    cout << "call back " <<endl;
  string str(msge->data);

  EnvMsg msg;
  msg.type = ID_ENV_MSG;
  //for(int i = 0; i < MAX_MSG_LEN; i++)
     // msg.data[i]='';

  for(int i = 0; i < str.length(); i++)
      msg.data[i]=str.at(i);
  for(int i = 0; i < sendTimes; i++)
  {
    sendData((void*)&msg, sizeof(msg));
    if(sendTimes > 1)
        usleep(100000);
  }
}

void WESocket::taskSubCallback(const std_msgs::String::ConstPtr& msge)
{
  string str(msge->data);
  TaskMsg msg;
  memset(msg.data,'\0',sizeof(msg.data));
  msg.type = ID_TASK_MSG;
 // for(int i = 0; i < MAX_MSG_LEN; i++)
     // msg.data[i]='';

   for(int i = 0; i < str.length(); i++)
      msg.data[i]=str.at(i);
  for(int i = 0; i < sendTimes; i++)
  {
    sendData((void*)&msg, sizeof(msg));
    if(sendTimes > 1)
        usleep(100000);
  }
  
}

void WESocket::actseqSubCallback(const std_msgs::String::ConstPtr& msge)
{
  string str(msge->data);
  ActseqMsg msg;
  msg.type = ID_ACTSEQ_MSG;
  //for(int i = 0; i < MAX_MSG_LEN; i++)
     // msg.data[i]='';

  for(int i = 0; i < str.length(); i++)
      msg.data[i]=str.at(i);
  for(int i = 0; i < sendTimes; i++)
  {
    sendData((void*)&msg, sizeof(msg));
    if(sendTimes > 1)
        usleep(100000);
  }
}

void WESocket::stateSubCallback(const std_msgs::Int32ConstPtr &data)
{
  StateMsg msg;
  msg.type = ID_STATE_MSG;
  msg.data = data->data;
  for(int i = 0; i < sendTimes; i++)
  {
    sendData((void*)&msg, sizeof(msg));
    if(sendTimes > 1)
        usleep(100000);
  }
}

void WESocket::poseSubCallback(const we_msgs::PoseConstPtr &data)
{
  PoseMsg msg;
  msg.type = ID_POSE_MSG;
  msg.x = data->x;
  msg.y = data->y;
  msg.r = data->r;
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
  float f1,f2,f3;
  int d;
  char s[MAX_MSG_LEN];
  //for(int i= 0 ; i < MAX_MSG_LEN; i++)
     // s[i]='';
  if(PEEK_CMD_SL(cmd,"se",2,s))
  {
    EnvMsg msg;
    msg.type = ID_ENV_MSG;
    for(int i= 0 ; i < MAX_MSG_LEN; i++)
        msg.data[i] = s[i];
    for(int i = 0; i < sendTimes; i++)
    {
        sendData((void*)&msg, sizeof(msg));
        if(sendTimes > 1)
            usleep(100000);
    }
  }
  else if(PEEK_CMD_D(cmd, "s", 1, d))
  {
    StateMsg msg;
    msg.type = ID_STATE_MSG;
    msg.data = d;
    for(int i = 0; i < sendTimes; i++)
    {
      sendData((void*)&msg, sizeof(msg));
      if(sendTimes > 1)
          usleep(100000);
    }

  }
  else if(PEEK_CMD_D(cmd, "r", 1, d))
  {
    std_msgs::Int32 cmd;
    cmd.data = d;
    statePub.publish(cmd);
    ROS_INFO("Receive State Msg [%d]", d);
  }
  else if (PEEK_CMD_FFF(cmd, "sp", 2, f1, f2, f3))
  {
    PoseMsg msg;
    msg.type = ID_POSE_MSG;
    msg.x = f1;
    msg.y = f2;
    msg.r = f3;
    for(int i = 0; i < sendTimes; i++)
    {
      sendData((void*)&msg, sizeof(msg));
      if(sendTimes > 1)
          usleep(100000);
    }
  }
  else if(PEEK_CMD_FFF(cmd, "rp", 2, f1, f2, f3))
  {
    we_msgs::Pose cmd;
    cmd.x = f1;
    cmd.y = f2;
    cmd.r = f3;
    posePub.publish(cmd);
    ROS_INFO("Receive Pose Msg [%.2f, %.2f, %.2f]", f1, f2, f3);

  }
  else if(PEEK_CMD_SL(cmd, "re", 2, s))
  {
    std_msgs::String cmd;
    cmd.data=s;
    worldPub.publish(cmd);
    ROS_INFO("Receive env Msg [%s]", s);

  }
  else if (PEEK_CMD_SL(cmd, "st", 2, s))
  {
    TaskMsg msg;
    msg.type = ID_TASK_MSG;
    for(int i = 0; i < MAX_MSG_LEN; i++)
      msg.data[i]=s[i];
    for(int i = 0; i < sendTimes; i++)
    {
      sendData((void*)&msg, sizeof(msg));
      if(sendTimes > 1)
          usleep(100000);
    }
  }
   else if(PEEK_CMD_SL(cmd, "rt", 2, s))
  {
    std_msgs::String cmd;
   cmd.data=s;
    taskPub.publish(cmd);
    ROS_INFO("Receive task Msg [%s]", s);

  }
  else if (PEEK_CMD_SL(cmd, "sq", 2, s))
  {
    ActseqMsg msg;
    msg.type = ID_ACTSEQ_MSG;
   for(int i = 0; i < MAX_MSG_LEN; i++)
      msg.data[i]=s[i];
    for(int i = 0; i < sendTimes; i++)
    {
      sendData((void*)&msg, sizeof(msg));
      if(sendTimes > 1)
          usleep(100000);
    }
  }
   else if(PEEK_CMD_SL(cmd, "rq", 2, s))
  {
    std_msgs::String cmd;
    cmd.data=s;
    actseqPub.publish(cmd);
    ROS_INFO("Receive actseq Msg [%s]", s);

  }
  else
  {
    printf("Unknown CMD [%s]\n", cmd);
    printUserCMD();
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
    case ID_POSE_MSG: return "Pose Msg";
    case ID_STATE_MSG: return "State Msg";
    case ID_HEART_BEAT_MSG: return "Heart Beat Msg";
    case ID_ENV_MSG: return "cloudworld Msg";
    case ID_TASK_MSG: return "tasksemi Msg";
    case ID_ACTSEQ_MSG: return "actionsequence Msg"; 
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

void WESocket::readUserCMD(const char *path)
{
  ifstream file(path);
  if(!file.is_open())
  {
    ROS_ERROR("Cann't Open UserCMD File: %s", path);
    return;
  }
  string line, name, val;
  while(getline(file,line))
  {
    size_t pose = line.find('=');
    if(pose != line.npos && pose != line.length() -1)
    {
      name = line.substr(0, line.find('='));
      val = line.substr(line.find('=') + 1);
      removeSpace(name);
      removeSpace(val);

      if(isNumber(val.c_str()))
      {
        int v = atoi(val.c_str());
        userCMD[v] = name;
      }
      else
      {
        printf("%s \n", val.c_str());
      }
    }
  }
  file.close();
}

void WESocket::printUserCMD()
{
  printf("----------------- USER CMD -----------------\n");
  for(map<int, string>::iterator it = userCMD.begin(); it != userCMD.end(); it++)
    printf("\t\t%3d : %s\n", it->first, it->second.c_str());
  printf("--------------------------------------------\n");
}

void WESocket::removeSpace(string &str)
{
  size_t start = str.find_first_not_of(' ');
  size_t end = str.find_last_not_of(' ');
  if(start != str.npos && end != str.npos)
    str = str.substr(start, end - start + 1);
}



































