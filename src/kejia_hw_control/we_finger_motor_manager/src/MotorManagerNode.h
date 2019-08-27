/*
 * motorManagerNode.h
 *
 *  Created on: May 4, 2013
 *      Author: kiter
 */

#ifndef MOTORMANAGERNODE_H_
#define MOTORMANAGERNODE_H_

#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "we_msgs/FingerPosition.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <boost/thread/pthread/recursive_mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>
#include <we_console/WEConsole.h>

#include "CanMotor.h"
#include "SerialComMotor.h"
#include "WEStopButton.h"
#include "pantiltBroadcaster.h"
#include "ConfigManager.h"


namespace we_finger_motor_manager
{
  class MotorManagerParameterConfig;
}
struct TorqueFeedback
{
    ros::Time stamp;
    int wheellTorque;
    int wheelrTorque;
    int shoulderzTorque;
    int shoulderyTorque;
    int elbowTorque;
    int wristyTorque;
    int wristzTorque;
    int pawTorque;
    int elevatorTorque;
};

struct BaseFeedback
{
  ros::Time stamp;
  double rightWheelDist;
  double leftWheelDist;
  double wlPose;
  double wrPose;

};

struct ArmFeedback
{
  ros::Time stamp;
  int shoulderzPose;
  int shoulderyPose;
  int elbowPose;
  int wristyPose;
  int wristzPose;
  int pawPose;
};

struct JointConfig
{
  float Position;
  float Velocity;
//  float Acceleration;
  float Torque;
};

struct FingerConfig
{
  JointConfig J0;
  JointConfig J1;
  JointConfig J2;
  JointConfig J3;
};

struct ThumbConfig
{
  JointConfig J0;
  JointConfig J1;
  JointConfig J2;
  JointConfig J3;
  JointConfig J4;
};

struct HandConfig
{
  FingerConfig ForeFinger;
  FingerConfig MiddleFinger;
  FingerConfig RingFinger;
  FingerConfig LittleFinger;
  ThumbConfig Thumb;
};

struct HandFeedback
{
  ros::Time stamp;
  HandConfig hand;
};

struct ArmTarget
{
  ros::Time stamp;
  int arm[5];
};

struct CanHandMotors
{
  CanMotor* ForeFingerMotors[4];
  CanMotor* MiddleFingerMotors[4];
  CanMotor* RingFingerMotors[4];
  CanMotor* LittleFingerMotors[4];
  CanMotor* ThumbMotors[5];
};



class MotorManagerNode : public WEConsole
{
public:
  MotorManagerNode();
  ~MotorManagerNode();

  void onCmd(const std_msgs::StringConstPtr & cmd);
  void onCmdVel(const geometry_msgs::TwistConstPtr & twist);
  void onFingerPosition(const we_msgs::FingerPositionConstPtr & msg);
private:
  
  int armbk_angle;
  void initMotors();
  void updateParam();
  void onStopButton(bool stop);
  void processCmd(const char* tempCmd);
  void publishOdomAndTF(const BaseFeedback & data);
  void publishMotorAngles(const ArmFeedback & data);
  void calOdom(const double &rdist, const double &ldist, double &theta, double &deltaX, double &deltaY);
  void calSpeed(const float &xSpeed, const float &turnSpeed, float &rightlSpeed, float &leftlSpeed);
  int generateWheelSpeedVal(double val);
  void wheel(double lSpeed, double rSpeed);
  void arm(double sz, double sy, double el, double wy, double wz);
  int dexteroushand(int finger , int position);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  int wr_speed, wl_speed;
  ros::Subscriber cmdSub;
  ros::Subscriber cmdVelSub;
  ros::Subscriber fingerPositionSub;

  ros::Subscriber HandSub;
  ros::Publisher HandDataPub;

  ros::Publisher odomPub;
  ros::Publisher motorAnglePub;
  ros::Publisher motorTorquesPub;
  ros::Publisher rawWheelDataPub;
  ros::Publisher stopButtonPub;

  tf::TransformBroadcaster* tf_broadcaster;
  dynamic_reconfigure::Server<we_finger_motor_manager::MotorManagerParameterConfig> *drsv;
  void reconfigureCB(we_finger_motor_manager::MotorManagerParameterConfig & cfg, uint32_t level);

  BaseFeedback baseFeed;
  ArmFeedback armFeed;
  HandFeedback handFeed;



  void baseLoop();
  void armLoop();
  void panTiltLoop();
  void stopButtonLoop();
  void monitorLoop();
  void handLoop();

  bool runBase;
  bool runArm;
  bool runStopButton;
  bool runMonitor;
  bool runHand;
  bool runDexterousHand;

  int baseLoopRate;
  int armLoopRate;
  int stopButtonLoopRate;
  int monitorLoopRate;
  int handLoopRate;



  boost::thread* baseThread;
  boost::thread* armThread;

  boost::thread* stopButtonThread;
  boost::thread* monitorThread;
  boost::thread* handThread;

  volatile bool runBaseThread;
  volatile bool runArmThread;
  volatile bool runStopButtonThread;
  volatile bool runMonitorThread;
  volatile bool runHandThread;


  double poseX;
  double poseY;
  double angle;

  double wlPose;
  double wrPose;
  double wlLastPose;
  double wrLastPose;
  ros::Time lastOdomTime;



  double maxLinearSpeed;
  double maxTurnSpeed;
  double wheelDist;
  double wheelPerimeter;





  typedef std::vector<Motor*>  MotorPtrVector;
  std::map<std::string, MotorPtrVector > motorMap;
  void registMotor(Motor* motor);
  void registMotor(std::string name , Motor*);
  bool inMap(std::string& name);

  MotorProfile p;

  CanOpen* can;
  CanMotor* wl;
  CanMotor* wr;
  CanMotor* ev;
  CanMotor* sz;
  CanMotor* sy;
  CanMotor* el;
  CanMotor* wy;
  CanMotor* wz;
  CanMotor* paw;
  CanMotor* Motor1;
  CanMotor* Motor2;

  CanHandMotors* handMotors;

  HandConfig lastHandConfig;


  std::vector<ArmTarget> lastArmPath;
  int lastMotorIndex[5];

  bool using_arm_avoid;
  bool armPlanning;
  void insertArmPoseTarget(int sz, int sy, int el, int wy, int wz, float duration);
  void armNavStep();
  int generateArmSpeedVal(float speed);
  void motor_goto(int motor_num);
  void arm_goto();


  CSerialCom* stopButtonCom;
  std::string StopButtonComName;
  WEStopButton *stopButton;
  bool isStopped;

};

#endif /* MOTORMANAGERNODE_H_ */
