#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_Q 0x71

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_E_CAP 0x45
#define KEYCODE_Q_CAP 0x51

class TeleopNode {
private:
	double walk_vel_;
	double run_vel_;
	double yaw_rate_;
	double yaw_rate_run_;
	double walk_vel_y;
	double run_vel_y;

	geometry_msgs::Twist cmdvel_;
	ros::NodeHandle n_;
	ros::Publisher pub_;

public:
	TeleopNode() {
		pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		ros::NodeHandle n_private("~");
		n_private.param("walk_vel", walk_vel_, 0.2);
		n_private.param("run_vel", run_vel_, 0.5);
		n_private.param("walk_vel_y", walk_vel_y, 0.2);
		n_private.param("run_vel_y", run_vel_y, 0.5);
		n_private.param("yaw_rate", yaw_rate_, 0.3);
		n_private.param("yaw_rate_run", yaw_rate_run_, 0.6);
	}

	~TeleopNode() {
	}
	void keyboardLoop();

	void stopRobot() {
		cmdvel_.linear.x = 0.0;
		cmdvel_.linear.y = 0.0;
		cmdvel_.angular.z = 0.0;
		pub_.publish(cmdvel_);
	}
};

TeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop");
	TeleopNode tbk;

	boost::thread t(boost::bind(&TeleopNode::keyboardLoop, &tbk));

	ros::spin();

	t.interrupt();
	t.join();
	tbk.stopRobot();
	tcsetattr(kfd, TCSANOW, &cooked);

	return (0);
}

void TeleopNode::keyboardLoop() {
	char c;
	double max_tv = walk_vel_;
	double max_rv = yaw_rate_;
	double max_tv_y = walk_vel_y;
	bool dirty = false;
	bool clear = false;
	int speed = 0;
	int speedy = 0;
	int turn = 0;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("Use WASD keys to control the robot");
	puts("Press Shift to move faster");

	struct pollfd ufd;
	ufd.fd = kfd;
	ufd.events = POLLIN;

	for (;;) {
		boost::this_thread::interruption_point();

		// get the next event from the keyboard
		int num;

		if ((num = poll(&ufd, 1, 350)) < 0) {
			perror("poll():");
			return;
		} else if (num > 0) {
			if (read(kfd, &c, 1) < 0) {
				perror("read():");
				return;
			} else {
				switch (c) {
				case KEYCODE_W:
					max_tv = walk_vel_;
					speed = 1;
					speedy = 0;
					turn = 0;
					dirty = true;
					break;
				case KEYCODE_S:
					max_tv = walk_vel_;
					speed = -1;
					speedy = 0;
					turn = 0;
					dirty = true;
					break;
				case KEYCODE_Q:
					max_rv = yaw_rate_;
					speed = 0;
					speedy = 0;
					turn = 1;
					dirty = true;
					break;
				case KEYCODE_E:
					max_rv = yaw_rate_;
					speed = 0;
					speedy = 0;
					turn = -1;
					dirty = true;
					break;
				case KEYCODE_A:
					max_tv_y = walk_vel_y;
					speed = 0;
					speedy = 1;
					turn = 0;
					dirty = true;
					break;
				case KEYCODE_D:
					max_tv_y = walk_vel_y;
					speed = 0;
					speedy = -1;
					turn = 0;
					dirty = true;
					break;

				case KEYCODE_W_CAP:
					max_tv = run_vel_;
					speed = 1;
					speedy = 0;
					turn = 0;
					dirty = true;
					break;
				case KEYCODE_S_CAP:
					max_tv = run_vel_;
					speed = -1;
					speedy = 0;
					turn = 0;
					dirty = true;
					break;
				case KEYCODE_Q_CAP:
					max_rv = yaw_rate_run_;
					speed = 0;
					speedy = 0;
					turn = 1;
					dirty = true;
					break;
				case KEYCODE_E_CAP:
					max_rv = yaw_rate_run_;
					speed = 0;
					speedy = 0;
					turn = -1;
					dirty = true;
					break;
				case KEYCODE_A_CAP:
					max_tv_y = run_vel_y;
					speed = 0;
					speedy = 1;
					turn = 0;
					dirty = true;
					break;
				case KEYCODE_D_CAP:
					max_tv_y = run_vel_y;
					speed = 0;
					speedy = -1;
					turn = 0;
					dirty = true;
					break;

				default:
					max_tv = walk_vel_;
					max_tv_y = walk_vel_y;
					max_rv = yaw_rate_;
					speed = 0;
					speedy = 0;
					turn = 0;
					dirty = false;
				}

				cmdvel_.linear.x = speed * max_tv;
				cmdvel_.linear.y = speedy * max_tv_y;
				cmdvel_.angular.z = turn * max_rv;
				pub_.publish(cmdvel_);
			}

		}

	}
}

