#ifndef GPSRGEN_H
#define GPSRGEN_H

#include "furnitures.h"
#include "rooms.h"
#include <vector>
#include <string>
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <boost/thread.hpp>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<tf/tf.h>
#include "we_console/WEConsole.h"
#include <visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

using namespace std;

class GPSRGen :  public WEConsole
{
public:
    GPSRGen();
    void cmdReceived(const char * cmd);
    void publishActiveGraph();
    void readConfig();
    void showConfig();
    void registConfig();
    void showEntity(string name);
    void addReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    void addPose(string name, string type ,double x, double y, double z);
    void onlyReceived(const geometry_msgs::PoseStampedConstPtr &msg);
    void onlyPose(string name,string type ,double x, double y, double z);
    void deleteReceived(const geometry_msgs::PointStampedConstPtr &msg);
    void delPose(string name, double x, double y);
    void pub();
    vector<BigObject*>objects;
    int object_index;
    vector<Pose>poses;
    vector<INFO>infoes;
    vector<string> name;
    string target;
    string type;
    vector<Entity> m_entities;

    ros::NodeHandle private_nh;
    ros::Publisher particle_pub;
    ros::Publisher maker_arry_pub;
    ros::Subscriber delete_sub;
    ros::Subscriber add_sub;
    ros::Subscriber only_sub;
    boost::thread *pub_thread;
};

#endif // GPSRGEN_H
