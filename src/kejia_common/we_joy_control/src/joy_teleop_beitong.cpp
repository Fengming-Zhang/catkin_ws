#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>


class WeJoyTeleop
{
private:

  int AXIS_FORWARD_BACKWARD;
  int ROCKER_FORWARD_BACKWARD;
  int AXIS_LEFT_RIGHT;
  int ROCKER_LEFT_RIGHT;
  int KEY_TURN_LEFT;
  int KEY_TURN_RIGHT;
  int KEY_SPEED_UP;

  int KEY_PANTILT_ON;
  int KEY_PAN_LEFT;
  int KEY_PAN_RIGHT;
  int KEY_TILT_UP;
  int KEY_TILT_DOWN;
  int AXIS_EV_UP_DOWN;   // axes

  int KEY_STOP_PRESS;
  int KEY_STOP_RELEASE;
  int KEY_PAW_OPEN;
  int KEY_PAW_CLOSE;
  int maxangle;
  double walk_vel_;
  double run_vel_;
  double yaw_rate_;
  double yaw_rate_run_;
  double walk_vel_y_;
  double run_vel_y_;
  double pantilt_increment_;
  double ev_increment_;
  int evorpl;

  bool speedup_;
  bool stop_;

  

  geometry_msgs::Twist cmd_vel;
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Publisher cmdPub;
  int count;
  bool testhand;

public:
  WeJoyTeleop()
  {
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    cmdPub = n.advertise<std_msgs::String>("motor_manager_cmd", 2);
    n.param("KEY_FORWARD_BACKWARD", AXIS_FORWARD_BACKWARD, 5);
    n.param("ROCKER_FORWARD_BACKWARD", ROCKER_FORWARD_BACKWARD, 5);
    n.param("KEY_LEFT_RIGHT", AXIS_LEFT_RIGHT, 4);
    n.param("ROCKER_LEFT_RIGHT", ROCKER_LEFT_RIGHT, 2);
    n.param("KEY_TURN_LEFT", KEY_TURN_LEFT, 3);
    n.param("KEY_TURN_RIGHT", KEY_TURN_RIGHT, 1);
    n.param("KEY_SPEED_UP", KEY_SPEED_UP, 6);
    n.param("KEY_PANTILT_ON", KEY_PANTILT_ON, 4);
    n.param("KEY_PAN_LEFT", KEY_PAN_LEFT, 0);
    n.param("KEY_PAN_RIGHT", KEY_PAN_RIGHT, 2);
    n.param("KEY_TILT_UP", KEY_TILT_UP, 3);
    n.param("KEY_TILT_DOWN", KEY_TILT_DOWN, 1);
    n.param("KEY_EV_UP", AXIS_EV_UP_DOWN, 5);
    n.param("testhand", testhand, false);

    n.param("KEY_STOP_PRESS", KEY_STOP_PRESS, 7);
    n.param("KEY_STOP_RELEASE", KEY_STOP_RELEASE, 5);
    n.param("KEY_PAW_OPEN", KEY_PAW_OPEN, 8);
    n.param("KEY_PAW_CLOSE", KEY_PAW_CLOSE, 9);

    n.param("walk_vel", walk_vel_, 0.3);
    n.param("run_vel", run_vel_, 0.5);
    n.param("walk_vel_y", walk_vel_y_, 0.3);
    n.param("run_vel_y", run_vel_y_, 0.5);
    n.param("yaw_rate", yaw_rate_, 0.3);
    n.param("yaw_rate_run", yaw_rate_run_, 0.8);
    n.param("pantilt_increment", pantilt_increment_, 5.0);
    n.param("ev_increment_", ev_increment_, 0.03);
    n.param("maxangle", maxangle, 12);
    n.param("evorpl",evorpl,1);
    

    speedup_ = false;
    stop_ = false;
    count = 0;
    ROS_INFO("%d %d", AXIS_FORWARD_BACKWARD, AXIS_LEFT_RIGHT);
  }

  int judgeKey(const double & vel)
  {
    if (vel < 0.0001 && vel > -0.0001)
        return 0;
    else if (vel >= 0.0001)
        return 1;
    else // vel < -0.001
        return -1;
  }



  std::string getstring ( const int n )
  {
      std::stringstream newstr;
      newstr<<n;
      return newstr.str();
  }

  void joySubCallback(const sensor_msgs::Joy::ConstPtr &msg)
  {
 /*
    if (msg->buttons[1] == 1 && msg->buttons[3] == 1)
    {
        std_msgs::String str;
        str.data = "arm 0 90 90 -90 0";
        cmdPub.publish(str);
    }
    if (msg->buttons[4] == 1 && msg->buttons[0] == 1)
    {
        std_msgs::String str;
        str.data = "armbk";
        cmdPub.publish(str);
    }
    if (msg->buttons[8] == 1 && msg->buttons[9] == 1)
    {
        std_msgs::String str;
        str.data = "arm 0 90 60 -60 0";
        cmdPub.publish(str);
    }
*/
    if (msg->buttons[0] == 1 || msg->buttons[1] == 1 \
        || msg->buttons[2] == 1 || msg->buttons[3] == 1\
        || msg->buttons[4] == 1 || msg->buttons[5] == 1)
    {
      std_msgs::String str;
      if (msg->buttons[5] == 1)
      {
       // str.data = "stop 0";
          if(evorpl == 1)
            str.data = "add ev 0.02";
          else
            str.data = "add pl 0.01";
      }
      if (msg->buttons[4] == 1)
      {
       // str.data = "stop 0";
          if(evorpl == 1)
            str.data = "add ev -0.02";
          else
            str.data = "add pl -0.01";
      }
      if (msg->buttons[3] == 1)
      {
       // str.data = "stop 0";
          str.data = "hand on";
      }

      if (msg->buttons[0] == 1)
      {
        //str.data = "stop 1";
          str.data = "hand off";
      }
      if (msg->buttons[2] == 1)
      {
        // str.data = "paw 70";
          if (count<maxangle)
            count++;
         if(!testhand)
         {
             switch (count)
             {
             case 1:
                 str.data = "sz 10";
                 break;
             case 2:
                 str.data = "sz 20";
                 break;
             case 3:
                 str.data = "sz 30";
                 break;
             case 4:
                 str.data = "sz 50";
                 break;
             case 5:
                 str.data = "sz 50";
                 break;
             case 6:
                 str.data = "sz 60";
                 break;
             case 7:
                 str.data = "sz 70";
                 break;
             case 8:
                 str.data = "sz 80";
                 break;
             case 9:
                 str.data = "sz 90";
                 break;
             case 10:
                 str.data = "sz 100";
                 break;
             case 11:
                 str.data = "sz 110";
                 break;
             case 12:
                 str.data = "sz 120";
                 break;
             case 13:
                 str.data = "sz 130";
                 break;
             case 14:
                 str.data = "sz 140";
                 break;
             case 15:
                 str.data = "sz 150";
                 break;
             case 16:
                 str.data = "sz 160";
                 break;
             case 17:
                 str.data = "sz 170";
                 break;
             case 18:
                 str.data = "sz 180";
                 break;
             case 19:
                 str.data = "sz 190";
                 break;
             case 20:
                 str.data = "sz 200";
                 break;
             case 21:
                 str.data = "sz 210";
                 break;
             case 22:
                 str.data = "sz 220";
                 break;
             case 23:
                 str.data = "sz 230";
                 break;
             case 24:
                 str.data = "sz 240";
                 break;
             case 25:
                 str.data = "sz 250";
                 break;
             default:
                 break;
             }
         }
         else
         {
             switch (count)
             {
             case 1:
                 str.data = "wz 10";
                 break;
             case 2:
                 str.data = "wz 20";
                 break;
             case 3:
                 str.data = "wz 30";
                 break;
             case 4:
                 str.data = "wz 50";
                 break;
             case 5:
                 str.data = "wz 50";
                 break;
             case 6:
                 str.data = "wz 60";
                 break;
             case 7:
                 str.data = "wz 70";
                 break;
             case 8:
                 str.data = "wz 80";
                 break;
             case 9:
                 str.data = "wz 90";
                 break;
             case 10:
                 str.data = "wz 100";
                 break;
             case 11:
                 str.data = "wz 110";
                 break;
             case 12:
                 str.data = "wz 120";
                 break;
             case 13:
                 str.data = "wz 130";
                 break;
             case 14:
                 str.data = "wz 140";
                 break;
             case 15:
                 str.data = "wz 150";
                 break;
             case 16:
                 str.data = "wz 160";
                 break;
             case 17:
                 str.data = "wz 170";
                 break;
             case 18:
                 str.data = "wz 180";
                 break;
             case 19:
                 str.data = "wz 190";
                 break;
             case 20:
                 str.data = "wz 200";
                 break;
             default:
                 break;
             }
         }
      }

      if (msg->buttons[1] == 1)
      {
          count = 0;
          if(testhand)
              str.data = "sz 0";
          else
              str.data = "wz 0";
        // str.data = "paw 0";
      }
      cmdPub.publish(std_msgs::String(str));
      return;
    }
    // L1
    /*
    else if (msg->buttons[KEY_PANTILT_ON] == 1)
    {
        std_msgs::String str;
        char incre[8], evIncre[8];
        sprintf(incre, "%.1lf", pantilt_increment_);
        sprintf(evIncre, "%.2lf", ev_increment_);

        if (msg->buttons[KEY_PAN_LEFT] == 1)
            str.data = "add pan " + std::string(incre);
        if (msg->buttons[KEY_PAN_RIGHT] == 1)
            str.data = "add pan -" + std::string(incre);
        if(msg->buttons[KEY_TILT_UP] == 1)
            str.data = "add tilt " + std::string(incre);
        if (msg->buttons[KEY_TILT_DOWN] == 1)
            str.data = "add tilt -" + std::string(incre);
        if (judgeKey(msg->axes[AXIS_EV_UP_DOWN]) == 1)
            str.data = "add ev " + std::string(evIncre);
        if (judgeKey(msg->axes[AXIS_EV_UP_DOWN]) == -1)
            str.data = "add ev -" + std::string(evIncre);
        if (!str.data.empty())
        {
//            ROS_INFO("Publish cmd: [%s]", str.data.c_str());
            cmdPub.publish(str);
        }
        return;
    }


    memset((void*)&cmd_vel, 0, sizeof(geometry_msgs::Twist));
    if (msg->buttons[KEY_SPEED_UP] == 1)
      speedup_ = true;
    else
      speedup_ = false;

    if (msg->buttons[KEY_TURN_LEFT] == 1 || judgeKey(msg->axes[ROCKER_LEFT_RIGHT]) == 1)
    {
      if (speedup_)
        cmd_vel.angular.z += yaw_rate_run_;
      else
        cmd_vel.angular.z += yaw_rate_;
    }

    if (msg->buttons[KEY_TURN_RIGHT] == 1 || judgeKey(msg->axes[ROCKER_LEFT_RIGHT]) == -1)
    {
      if (speedup_)
        cmd_vel.angular.z -= yaw_rate_run_;
      else
        cmd_vel.angular.z -= yaw_rate_;
    }

    int temp = judgeKey(msg->axes[AXIS_FORWARD_BACKWARD]) | judgeKey(msg->axes[ROCKER_FORWARD_BACKWARD]);
    if (temp == 1)
    {
      if (speedup_)
        cmd_vel.linear.x += run_vel_;
      else
        cmd_vel.linear.x += walk_vel_;
    }
    else if (temp == -1)
    {
      if (speedup_)
        cmd_vel.linear.x -= run_vel_;
      else
        cmd_vel.linear.x -= walk_vel_;
    }

    temp = judgeKey(msg->axes[AXIS_LEFT_RIGHT]);
    if (temp == 1)
    {
      if (speedup_)
        cmd_vel.linear.y += run_vel_y_;
      else
        cmd_vel.linear.y += walk_vel_y_;
    }
    else if (temp == -1)
    {
      if (speedup_)
        cmd_vel.linear.y -= run_vel_y_;
      else
        cmd_vel.linear.y -= walk_vel_y_;
    }

    pub.publish(cmd_vel);
*/
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  ros::NodeHandle n;
  WeJoyTeleop joy;
  ros::Subscriber sub = n.subscribe("joy", 1, &WeJoyTeleop::joySubCallback, &joy);
  ros::spin();
  return 0;
}
