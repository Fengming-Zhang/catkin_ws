/*
 * motorManagerNode.cpp
 *
 *  Created on: May 4, 2013
 *      Author: Min Cheng
 */

#include <iostream>
#include <string>
#include <string.h>
#include <pthread.h>
#include "xform.h"
#include <we_msgs/MotorAngles.h>
#include <we_msgs/MotorTorques.h>
#include <we_msgs/WeMcuData.h>
//#include <test_hand/WeMcuData.h>
#include <we_msgs/HandConfig.h>

#include "MotorManagerNode.h"
#include <we_finger_motor_manager/MotorManagerParameterConfig.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;

void MotorManagerNode::updateParam()
{
    private_nh.getParamCached("maxLinearSpeed", maxLinearSpeed);
    private_nh.getParamCached("maxTurnSpeed", maxTurnSpeed);

}


MotorManagerNode::MotorManagerNode() :
    WEConsole("base", boost::bind(&MotorManagerNode::processCmd, this, _1)),
    private_nh("~")
{

  armPlanning = false;
  using_arm_avoid = false;

  monitorLoopRate = 10;

  can = NULL;

  stopButtonCom = NULL;
   wl = wr = ev = sz = sy = el = wy = wz = paw = NULL;

//  for (int i =0; i<4; ++i)
//  {
//    handMotors->ForeFingerMotors[i]=NULL;
//    handMotors->MiddleFingerMotors[i]=NULL;
//    handMotors->RingFingerMotors[i]=NULL;
//    handMotors->LittleFingerMotors[i]=NULL;
//  }
//  for (int i=0;i<5;++i)
//  {
//    handMotors->ThumbMotors[i]=NULL;
//  }

   runBaseThread = runArmThread = runStopButtonThread = runMonitorThread= runHandThread = false;

  stopButton = NULL;

  isStopped = false;
  for(int i = 0; i < 5; i++)
      lastMotorIndex[i] = 0;
  lastArmPath.clear();


    private_nh.param("baseLoopRate", baseLoopRate, 10);
  private_nh.param("armLoopRate", armLoopRate, 10);
  private_nh.param("stopButtonLoopRate", stopButtonLoopRate, 10);
  private_nh.param("handLoopRate", handLoopRate, 100);

  private_nh.param("runBase", runBase, true);
  private_nh.param("runArm", runArm, true);
  private_nh.param("runStopButton", runStopButton, true);
  private_nh.param("runMonitor",runMonitor, true);
  private_nh.param("runHand",runHand, false);

  
  if(runBase)
  {
    cmdSub = nh.subscribe<std_msgs::String>("motor_manager_cmd", 100, &MotorManagerNode::onCmd, this);
    cmdVelSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MotorManagerNode::onCmdVel, this);
    odomPub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    rawWheelDataPub = nh.advertise<geometry_msgs::PointStamped>("raw_wheel_data",100);
  }
  if(runArm)
  {
    motorAnglePub = nh.advertise<we_msgs::MotorAngles>("motor_angle", 100);
  }
  if(runMonitor)
  {
    motorTorquesPub = nh.advertise<we_msgs::MotorTorques>("motor_torque", 100);
    runMonitorThread = true;
    monitorThread = new boost::thread(boost::bind(&MotorManagerNode::monitorLoop, this));
  }
  if(runStopButton)
  {
    stopButtonPub = nh.advertise<std_msgs::String>("stop_button", 10);
  }
  if(runHand)
  {
    HandDataPub = nh.advertise<we_msgs::HandConfig>("handState", 100);
  }
  

  tf_broadcaster = new tf::TransformBroadcaster;

  drsv = new dynamic_reconfigure::Server<we_finger_motor_manager::MotorManagerParameterConfig>;
  drsv->setCallback(boost::bind(&MotorManagerNode::reconfigureCB, this, _1, _2));





  initMotors();
  MotorPtrVector & motors = motorMap["all"];
  for(MotorPtrVector::iterator it = motors.begin(); it !=  motors.end(); it++)
    (*it)->init();


  if(can && can->isOpened())
    can->startRecive();

//  if(ev)
//  {
//      ev->clearPose();
//      ev->setupPositionMove(0);
//  }
}

string doubleToString(double num)
{
  char str[256];
  sprintf(str, "%lf", num);
  string result = str;
  return result;
}

MotorManagerNode::~MotorManagerNode()
{


  if(can)
    can->stopRecive();


  runBaseThread = false;
  runArmThread = false;
  runStopButtonThread = false;

  if (baseThread != NULL)
  {
    baseThread->join();
    delete baseThread;
  }

  if (armThread != NULL)
  {
    armThread->join();
    delete armThread;
  }


  if (stopButtonThread != NULL)
  {
    stopButtonThread->join();
    delete stopButtonThread;
  }


  if(wl)
      delete wl;
  if(wr)
      delete wr;
  if(ev)
      delete ev;
  if(sz)
      delete sz;
  if(sy)
      delete sy;
  if(el)
      delete el;
  if(wy)
      delete wy;
  if(wz)
      delete wz;
  if(paw)
      delete paw;


  if(can != NULL)
    delete can;

  if(stopButton != NULL)
    delete stopButton;

  if(stopButtonCom != NULL)
    delete stopButtonCom;

  delete tf_broadcaster;

}

void MotorManagerNode::initMotors()
{


  if(runBase || runArm)
  {
    can = new CanOpen(VCI_USBCAN1, 0, 0);
    if(!can->openDevice())
    {
      ROS_ERROR("Open can failed, can network motors will not be started!");
      runBase = false;
      runArm = false;
    }
  }

  if (runBase)
  {
    wl = new CanMotor("wl", 72, can);
    wr = new CanMotor("wr", 71, can);
    can->setPosePDOBaseID(wl->getPosePDOBaseID());
    //----add begin-----------
    can->setTorquePDOBaseID(wl->getTorquePDOBaseID());
    //----add end-------------
    wl->setSyncProducer(true);
    registMotor((Motor*)wl);
    registMotor((Motor*)wr);
    registMotor("all",  (Motor*)wl);
    registMotor("all",  (Motor*)wr);

    runBaseThread = true;
    baseThread = new boost::thread(boost::bind(&MotorManagerNode::baseLoop, this));
  }
  else
  {
    runBaseThread = false;
    baseThread = NULL;
    ROS_WARN("Base thread didn't start up !");
  }

  if(runArm)
  {
    #include <cstdlib>
    //const char *robot = std::getenv("ROBOT");

        ev = new CanMotor("ev", 11, can);


    sz = new CanMotor("sz", 2, can);
    sy = new CanMotor("sy", 1, can);
    el = new CanMotor("el", 14, can);
    wy = new CanMotor("wy", 15, can);
    wz = new CanMotor("wz", 16, can);
    paw = new CanMotor("paw", 17, can);
    if(!runBase)
    {
      can->setPosePDOBaseID(sz->getPosePDOBaseID());
      //---add begin--------
      can->setTorquePDOBaseID(sz->getTorquePDOBaseID());
      //---add end----------
      sz->setSyncProducer(true);
    }

    
    registMotor((Motor*)ev);
    registMotor((Motor*)sz);
    registMotor((Motor*)sy);
    registMotor((Motor*)el);
    registMotor((Motor*)wy);
    registMotor((Motor*)wz);
    registMotor((Motor*)paw);

    registMotor("all", (Motor*)ev);
    registMotor("all", (Motor*)sz);
    registMotor("all", (Motor*)sy);
    registMotor("all", (Motor*)el);
    registMotor("all", (Motor*)wy);
    registMotor("all", (Motor*)wz);
    registMotor("all", (Motor*)paw);

    registMotor("arm", (Motor*)sz);
    registMotor("arm", (Motor*)sy);
    registMotor("arm", (Motor*)el);
    registMotor("arm", (Motor*)wy);
    registMotor("arm", (Motor*)wz);
    registMotor("arm", (Motor*)paw);

    runArmThread = true;
    armThread = new boost::thread(boost::bind(&MotorManagerNode::armLoop, this));
  }
  else
  {
    runArmThread = false;
    armThread = NULL;
    ROS_WARN("Arm thread didn't start up !");
  }


  if(runStopButton)
  {
    private_nh.param("StopButtonComName", StopButtonComName, string("/dev/ttyUSB0"));
    stopButtonCom = new CSerialCom(StopButtonComName.c_str(), 9600, 8, 'N', 1);
    if(!stopButtonCom->openDevice())
    {
      ROS_ERROR("Open stop button usbcom failed!");
      runStopButton = false;
      runStopButtonThread = false;
      stopButtonThread = NULL;
      ROS_WARN("Stop button thread didn't start up !");
    }
    else
    {
      stopButton = new WEStopButton("StopButton", stopButtonCom, boost::bind(&MotorManagerNode::onStopButton, this, _1));
      runStopButtonThread = true;
      stopButtonThread = new boost::thread(boost::bind(&MotorManagerNode::stopButtonLoop, this));
    }

  }
  else
  {
      runStopButtonThread = false;
      stopButtonThread = NULL;
      ROS_WARN("Stop button thread didn't start up !");
  }


}

void MotorManagerNode::monitorLoop()
{
    ROS_INFO("Monitor Thread Start...");
    sleep(5);
    ros::Rate loopRate(monitorLoopRate);
    while (runMonitorThread)
    {
        we_msgs::MotorTorques torques;
        torques.stamp = ros::Time::now();
        if (runBaseThread)
        {
            torques.wheell = wl->getTorque();
            torques.wheelr= wr->getTorque();
        }
        if (runArmThread)
        {
            torques.shoulderz = sz->getTorque();
            torques.shouldery= sy->getTorque();
            torques.elbow = el->getTorque();
            torques.wristy= wy->getTorque();
            torques.wristz= wz->getTorque();
            torques.paw = paw->getTorque();
            torques.elevator = ev->getTorque();
        }
        motorTorquesPub.publish(torques);
        loopRate.sleep();
    }
}
void MotorManagerNode::baseLoop()
{
  ROS_INFO("Base Thread Start...");
  sleep(2);
  ros::Rate loopRate(baseLoopRate);
  while (runBaseThread)
  {
    updateParam();
    bool ignore_feedback = false;
    baseFeed.stamp = ros::Time::now();
    wlPose = wl->getPose();
//    ROS_INFO("%f---------------------", wl->getTorque());
    wrPose = wr->getPose();
    if (wl_speed != 0 && fabs(wlPose - wlLastPose) < 0.5 && isStopped == false)
            ignore_feedback = true;
    if (wr_speed != 0 && fabs(wrPose - wrLastPose) < 0.5 && isStopped == false)
            ignore_feedback = true;
    //printf("%d\n", wl_speed);
//    ROS_INFO("wlPose: %f wrPose: %f", wlPose, wrPose);
    //if (ignore_feedback == false)
    if(true)
    {
        //ROS_INFO("Not ignore feedback");
        baseFeed.wlPose = wlPose;// - wlLastPose;
        baseFeed.wrPose = wrPose;// - wrLastPose;
        baseFeed.leftWheelDist =  (wlPose - wlLastPose) * wheelPerimeter / 360.0;
        baseFeed.rightWheelDist = (wrPose - wrLastPose) * wheelPerimeter / 360.0;
        wlLastPose = wlPose;
        wrLastPose = wrPose;
        publishOdomAndTF(baseFeed);
    }
    else
    {
        ROS_ERROR("Ignore feedback");
        wlPose = baseFeed.leftWheelDist * 360.0 / wheelPerimeter + wlLastPose;
        wrPose = baseFeed.rightWheelDist * 360.0 / wheelPerimeter + wrLastPose;
        wlLastPose = wlPose;
        wrLastPose = wrPose;
        publishOdomAndTF(baseFeed);
    }
    loopRate.sleep();

    if (loopRate.cycleTime() > ros::Duration(1.0 / baseLoopRate))
      ROS_WARN(
          "Base loop missed its desired rate of %dHz... the loop actually took %.5f seconds", baseLoopRate, loopRate.cycleTime().toSec());

  }
}

void MotorManagerNode::armLoop()
{
  ROS_INFO("Arm Thread Start...");
  ros::Rate loopRate(armLoopRate);
  sleep(2);
  updateParam();
  while (runArmThread)
  {
    we_msgs::MotorAngles angles;

    angles.stamp = ros::Time::now();
    angles.shoulderz = sz->getPose();
    angles.shouldery= sy->getPose();
    angles.elbow = el->getPose();
    angles.wristy= wy->getPose();
    angles.wristz= wz->getPose();
    angles.paw = paw->getPose();

    armNavStep();



    motorAnglePub.publish(angles);

    loopRate.sleep();
    /*
    if (loopRate.cycleTime() > ros::Duration(1.0 / armLoopRate))
      ROS_WARN(
          "Arm loop missed its desired rate of %dHz... the loop actually took %.4f seconds", armLoopRate, loopRate.cycleTime().toSec());
          */
  }
}



void MotorManagerNode::stopButtonLoop()
{
  ROS_INFO("Stop Button Thread Start...");
  ros::Rate loopRate(stopButtonLoopRate);
  sleep(2);
  while (runStopButtonThread)
  {
    stopButton->mainLoop();
    loopRate.sleep();
  }
}


void MotorManagerNode::onCmd(const std_msgs::StringConstPtr & cmd)
{
  processCmd(cmd->data.c_str());

}
void MotorManagerNode::onCmdVel(const geometry_msgs::TwistConstPtr & twist)
{
  float rSpeed, lSpeed;
  float linear = between(twist->linear.x, -maxLinearSpeed, maxLinearSpeed);
  float turn = between(twist->angular.z,  -maxTurnSpeed, maxTurnSpeed);
  calSpeed(linear, turn, rSpeed, lSpeed);
  if (fabs(lSpeed) > maxLinearSpeed )
  {
      lSpeed = lSpeed / fabs(lSpeed) * maxLinearSpeed;
      rSpeed = rSpeed / fabs(lSpeed) * maxLinearSpeed;
  }
  if (fabs(rSpeed) > maxLinearSpeed)
  {
      lSpeed = lSpeed / fabs(rSpeed) * maxLinearSpeed;
      rSpeed = rSpeed / fabs(rSpeed) * maxLinearSpeed;
  }
  wheel(lSpeed, rSpeed);
}

void MotorManagerNode::onStopButton(bool stop)
{
  isStopped = stop;
  MotorPtrVector motors = motorMap["all"];
  for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
  {
    if(stop)
      (*it)->motorStop();
    else
      (*it)->motorContinue();
  }

  std_msgs::String str;
  if(stop)
    str.data = string("stop button on");
  else
      str.data = string("stop button off");
  stopButtonPub.publish(str);
}

void MotorManagerNode::processCmd(const char *tempCmd)
{
  if (strlen(tempCmd) < 1)
    return;
  const char *cmd;
  string temp(tempCmd);
  unsigned int start = temp.find_first_not_of(' ');
  unsigned int end = temp.find_last_not_of(' ');
  if(start != string::npos && end != string::npos)
  {
    temp = temp.substr(start, end - start + 1);
    cmd = temp.c_str();
  }
  else
      return;

  float f1, f2;
  int d1, d2, d3, d4, d5;

  if (PEEK_CMD_FF(cmd, "m", 1, f1, f2))
  {
    wheel(f1, f2);
  }

  else if (PEEK_CMD_D(cmd,"stop", 4, d1))
  {
    onStopButton(d1);
  }
  else if(PEEK_CMD_N(cmd, "stop", 4))
  {
    wheel(0, 0);
  }
  else if (PEEK_CMD_DDDDD(cmd, "arm", 3, d1, d2, d3, d4, d5))
  {
    if (using_arm_avoid)
    {
        int szPose = sz->getPose();
        int syPose = sy->getPose();
        int elPose = el->getPose();
        int wyPose = wy->getPose();
        int wzPose = wz->getPose();
        if (elPose > 80 && elPose < 95)
        {
            insertArmPoseTarget(szPose, syPose, 80, wyPose, wzPose, 0.5);
        }
        if (d3 > 80 && d3 < 95)
        {
            insertArmPoseTarget(d1, d2, 80, d4, d5, 3);
        }
        insertArmPoseTarget(d1, d2, d3, d4, d5, 0.5);
    }
    else
    {
        arm(d1, d2, d3, d4,d5);
    }
  }
  else if (PEEK_CMD_DDDDDF(cmd, "plan arm", 8, d1, d2, d3, d4, d5, f1))
  {
    insertArmPoseTarget(d1, d2, d3, d4, d5, f1);
  }

  else if (PEEK_CMD_D(cmd, "add sz", 6, d1))
  {
    int szpose = sz->getPose();
    d1 += szpose;
    sz->setupPositionMove(d1);
    sz->startPositionMove();
  }
  else if (PEEK_CMD_F(cmd, "sz", 2, f1))
  {
    if (using_arm_avoid)
    {
        int szPose = sz->getPose();
        int syPose = sy->getPose();
        int elPose = el->getPose();
        int wyPose = wy->getPose();
        int wzPose = wz->getPose();
        if (elPose > 80 && elPose < 95)
        {
            insertArmPoseTarget(szPose, syPose, 80, wyPose, wzPose, 0.5);
            insertArmPoseTarget(f1, syPose, 80, wyPose, wzPose, (f1 - szPose) / 80);
            insertArmPoseTarget(f1, syPose, elPose, wyPose, wzPose, 0.5);
        }
    }
    else if(sz)
    {
      sz->setupPositionMove(f1);
      sz->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "sy", 2, d1))
  {
    if(sy)
    {
      sy->setupPositionMove(d1);
      sy->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "el", 2, d1))
  {
    if(el)
    {
      el->setupPositionMove(d1);
      el->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "wy", 2, d1))
  {
    if(wy)
    {
      wy->setupPositionMove(d1);
      wy->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "wz", 2, d1))
  {
    if(wz)
    {
      wz->setupPositionMove(d1);
      wz->startPositionMove();
    }
  }
  else if (PEEK_CMD_N(cmd, "break", 5))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name))
    {
      MotorPtrVector motors;
      if(name == string("all"))
      {
        motors = motorMap["arm"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorbreak();

        motors = motorMap["cam"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorbreak();
      }
      else
      {
        motors = motorMap[name];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorbreak();
      }

    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n\n";
      return;
    }

  }
  else if (PEEK_CMD_N(cmd, "clear", 5))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name))
    {
      MotorPtrVector motors;
      if(name == string("all"))
      {
        motors = motorMap["arm"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorClear();

        motors = motorMap["cam"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorClear();
      }
      else
      {
        motors = motorMap[name];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorClear();
      }
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "init", 4))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name))
    {
      MotorPtrVector motors;
      if(name == string("all"))
      {
        motors = motorMap["arm"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorInit();

        motors = motorMap["cam"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorInit();
      }
      else
      {
        motors = motorMap[name];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorInit();
      }
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "power", 5))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name))
    {
      MotorPtrVector motors;
      if(name == string("all"))
      {
        motors = motorMap["arm"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->setPower();

        motors = motorMap["cam"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->setPower();
      }
      else
      {
        motors = motorMap[name];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->setPower();
      }
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "restart", 7))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name) && name != string("all") && name != string("arm"))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->restart();
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_DDDDD(cmd, "speed arm", 9, d1, d2, d3, d4, d5))
  {
    if(runArmThread)
    {
      sz->setSpeed(d1);
      sy->setSpeed(d2);
      el->setSpeed(d3);
      wy->setSpeed(d4);
      wz->setSpeed(d5);
    }
  }
  else if (PEEK_CMD_N(cmd, "speed acc", 9))
  {
    string name(cmd+9);
    int start = name.find_first_of(' ');
    int end = name.find_last_of(' ');
    string stringVal = name.substr(end + 1);
    name = name.substr(start + 1, end - start - 1);

    std::stringstream ss(stringVal);
    int val = -100;
    ss >> val;

    if(inMap(name))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
        (*it)->setSpeedAcc(val);
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "speed dec", 9))
  {
    string name(cmd+9);
    int start = name.find_first_of(' ');
    int end = name.find_last_of(' ');
    string stringVal = name.substr(end + 1);
    name = name.substr(start + 1, end - start - 1);

    std::stringstream ss(stringVal);
    int val = -100;
    ss >> val;

    if(inMap(name))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
        (*it)->setSpeedDec(val);
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if(PEEK_CMD_N(cmd, "set profile", 11))
  {
    char name[100]={0};
    sscanf(cmd + 12,"%s %d %d %d", name, &d1, &d2, &d3);
    string stringName((const char*)name);
    if(inMap(stringName))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
      {
        (*it)->setSpeed(d1);
        (*it)->setSpeedAcc(d2);
        (*it)->setSpeedDec(d3);
      }
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if(PEEK_CMD_N(cmd, "reset", 5))
  {
    string name(cmd);
    int start = name.find_first_of(' ');
    name = name.substr(start + 1);

    if(inMap(name))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
        (*it)->resetProfile();
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "speed", 5))
  {
    string name(cmd);
    int start = name.find_first_of(' ');
    int end = name.find_last_of(' ');
    string stringVal = name.substr(end + 1);
    name = name.substr(start + 1, end - start - 1);

    std::stringstream ss(stringVal);
    int val = -100;
    ss >> val;

    if(inMap(name))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
        (*it)->setSpeed(val);
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "armbk", 5))
  {
    if(runArmThread)
    {
        if (using_arm_avoid)
        {
            int szPose = sz->getPose();
            int syPose = sy->getPose();
            int elPose = el->getPose();
            int wyPose = wy->getPose();
            int wzPose = wz->getPose();
            if (elPose > 80 && elPose < 95)
            {
                insertArmPoseTarget(szPose, syPose, 80, wyPose, wzPose, 0.5);
            }
            insertArmPoseTarget(sz->getPose() > 0 ? 90 : -90 , 90, 80, 0, 0, 3);
            insertArmPoseTarget(sz->getPose() > 0 ? 90 : -90 , 90, 90, 0, 0, 0.5);
        }
        else
        {
            arm(sz->getPose() > 0 ? 90 : -90, 90, armbk_angle, 0, 0);
        }

      }
  }
  else if(PEEK_CMD_N(cmd, "help", 4))
  {
    cout << "armbk\n" << "init arm\n" << "break arm\n" << "clear arm\n" << "ev {0}\n" << "set ev {0}\n" << endl;
  }

  else
  {
    cout << "receive unknown command [" << cmd << "]\n";
    return;
  }
}

void MotorManagerNode::publishOdomAndTF(const BaseFeedback & data)
{
  double theta, deltaX, deltaY, turnSpeed, xSpeed, ySpeed, lWheelSpeed, rWheelSpeed;

  calOdom(data.rightWheelDist, data.leftWheelDist, theta, deltaX, deltaY);
  double t = 1.0 / (data.stamp.toSec() - lastOdomTime.toSec());
  lastOdomTime = data.stamp;

  turnSpeed = theta * t;
  xSpeed = deltaX * t;
  ySpeed = deltaY * t;
  lWheelSpeed = data.leftWheelDist * t;
  rWheelSpeed = data.rightWheelDist * t;

  double poseDeltaX = cos(angle) * deltaX - sin(angle) * deltaY;
  double poseDeltaY = sin(angle) * deltaX + cos(angle) * deltaY;
  poseX += poseDeltaX;
  poseY += poseDeltaY;
  theta = theta;
  //angle = anormalize(angle + theta);
  angle = anormalize(angle + theta);
//  printf("%+11.6f\n",angle/3.14159263*180);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = data.stamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = poseX;
  odom_trans.transform.translation.y = poseY;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  tf_broadcaster->sendTransform(odom_trans);


  nav_msgs::Odometry odom;
  odom.header.stamp = data.stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = poseX;
  odom.pose.pose.position.y = poseY;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.twist.twist.linear.x = xSpeed;
  odom.twist.twist.linear.y = ySpeed;
  odom.twist.twist.angular.x = lWheelSpeed;
  odom.twist.twist.angular.y = rWheelSpeed;
  odom.twist.twist.angular.z = turnSpeed;

  odomPub.publish(odom);

  geometry_msgs::PointStamped rawWheelData;
  rawWheelData.header.stamp = data.stamp;
  rawWheelData.point.x = data.wlPose;
  rawWheelData.point.y = data.wrPose;
  rawWheelDataPub.publish(rawWheelData);

}

void MotorManagerNode::calOdom(const double &rdist, const double &ldist, double &theta, double &deltaX, double &deltaY)
{
  theta = (rdist - ldist) / wheelDist;
  double radius;
  if (fabs(theta) > 0.00000000001)
  {
    radius = (rdist + ldist) / (2 * theta);
    deltaX = sin(theta) * radius;
    deltaY = (1 - cos(theta)) * radius;
  }
  else
  {
    deltaX = (rdist + ldist) / 2;
    deltaY = 0;
  }
}

void MotorManagerNode::calSpeed(const float &xSpeed, const float &turnSpeed, float &rightSpeed, float &leftSpeed)
{
  rightSpeed = xSpeed + (turnSpeed * wheelDist) / 2;
  leftSpeed = xSpeed - (turnSpeed * wheelDist) / 2;
}

int MotorManagerNode::generateWheelSpeedVal(double val)
{
  int sign = 1;
  if (val < 0)
  {
    sign = -1;
    val = -val;
  }
  return (int)(sign * (val / wheelPerimeter) * 10 * 360);
}



void MotorManagerNode::registMotor(Motor *motor)
{
  motorMap[motor->name].push_back(motor);
}

void MotorManagerNode::registMotor(string name, Motor *motor)
{
   motorMap[name].push_back(motor);
}

bool MotorManagerNode::inMap(string &name)
{
  return (motorMap.find(name) != motorMap.end());
}

void MotorManagerNode::wheel(double lSpeed, double rSpeed)
{
  if(!runBaseThread)
    return;
  int lvel, rvel;
//  lvel = generateWheelSpeedVal(lSpeed);
//  rvel = generateWheelSpeedVal(rSpeed);
//  wl_speed = lvel;
//  wr_speed = rvel;
  wl_speed = lSpeed;
  wr_speed = rSpeed;
  ROS_INFO("wl_speed: %i, wr_speed: %i",wl_speed, wr_speed);
  wl->velocityMove(wl_speed);
  wr->velocityMove(wr_speed);
}

void MotorManagerNode::arm(double szPose, double syPose, double elPose, double wyPose, double wzPose)
{
  if(runArmThread)
  {
    sz->setupPositionMove(szPose);
    sy->setupPositionMove(syPose);
    el->setupPositionMove(elPose);
    wy->setupPositionMove(wyPose);
    wz->setupPositionMove(wzPose);

    sz->startPositionMove();
    sy->startPositionMove();
    el->startPositionMove();
    wy->startPositionMove();
    wz->startPositionMove();
  }

}


void MotorManagerNode::reconfigureCB(we_finger_motor_manager::MotorManagerParameterConfig &cfg, uint32_t level)
{
  if(level == 0)
  {
//    if(pan)
//        pan->setOffset(cfg.panOffset);
//    if(tilt)
//        tilt->setOffset(cfg.tiltOffset);
  }
  else if(level == 1)
  {
    wheelDist = cfg.wheelDist;
    wheelPerimeter = cfg.wheelPerimeter;
  }
}