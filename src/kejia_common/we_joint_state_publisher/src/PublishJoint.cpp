#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <we_msgs/MotorAngles.h>

class JointPublisher
{

 public:
    ros::NodeHandle nh;
    ros::NodeHandle private_n;
    ros::Publisher joint_pub;
    ros::Subscriber joint_sub;

    double degree;
    // robot state
  //  double tilt,pan,shoulderz,shouldery,elbow,wristy,wristz,paw,elevator;


    JointPublisher()
    {
        degree = M_PI/180; 
        joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        joint_sub=nh.subscribe("motor_angle",1000,&JointPublisher::messageCallBack,this);


    }
    void messageCallBack(const we_msgs::MotorAnglesPtr & msg)
    {
        // message declarations
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp=msg->stamp;
        joint_state.name.resize(8);
        joint_state.position.resize(8);

        joint_state.name[7]="ev";
        joint_state.position[7]=msg->elevator;

/*        joint_state.name[1]="pan_to_armbase";
        joint_state.position[1]=msg->pan*degree;

        joint_state.name[2]="tilt_to_pan";
        joint_state.position[2]=msg->tilt*degree;
*/
        joint_state.name[0]="sz";
        joint_state.position[0]=msg->shoulderz*degree;

        joint_state.name[1]="sy";
        joint_state.position[1]=msg->shouldery*degree;

        joint_state.name[2]="el";
        joint_state.position[2]=msg->elbow*degree;

        joint_state.name[3]="wy";
        joint_state.position[3]=msg->wristy*degree;

        joint_state.name[4]="wz";
        joint_state.position[4]=msg->wristz*degree;

        joint_state.name[5]="left_gripper_to_wrist";
        joint_state.position[5]=msg->paw*degree;

        joint_state.name[6]="right_gripper_to_wrist";
        joint_state.position[6]=-(msg->paw)*degree;

        joint_pub.publish(joint_state);



    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_publisher");
    ros::NodeHandle n;
    JointPublisher jointPublisher;
    ros::spin();

    return 0;
}

