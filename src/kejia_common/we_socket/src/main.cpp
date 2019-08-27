#include <ros/ros.h>
#include "WESocket.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "we_socket");
  WESocket socket;

  ros::spin();
  return 0;
}
