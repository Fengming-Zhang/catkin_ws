#include "gpsrgen.h"
#include "xform.h"

GPSRGen::GPSRGen():
    WEConsole("generator", boost::bind(&GPSRGen::cmdReceived, this, _1)),
    private_nh("~")
{
    particle_pub = private_nh.advertise<geometry_msgs::PoseArray>("/mani_places", 10);
    delete_sub = private_nh.subscribe("/delete", 1, &GPSRGen::deleteReceived, this);
    maker_arry_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/objects", 2);
    add_sub = private_nh.subscribe("/add", 1, &GPSRGen::addReceived, this);
    only_sub = private_nh.subscribe("/only", 1, &GPSRGen::onlyReceived, this);
    readConfig();
    showConfig();
    registConfig();
    this->infoes.clear();
    this->poses.clear();
    target = "all";
    for (int i = 0; i < objects.size(); i++)
    {
        objects[i]->generatePose();
        for (int j = 0; j < objects[i]->poses.size(); j++)
        {
//            cout << this->objects[i]->name << endl;
//            cout << objects[i]->poses[j].getPoseX() << ", "<< objects[i]->poses[j].getPoseY()<< ", "<< objects[i]->poses[j].getPoseR()<<endl;
            this->poses.push_back(objects[i]->poses[j]);
            this->infoes.push_back(INFO(this->objects[i]->name, this->objects[i]->type, objects[i]->poses[j]));
        }

    }
    ROS_INFO("DRAWING PLACES %d", poses.size());
    pub_thread = new boost::thread(boost::bind(&GPSRGen::pub, this));

}

void GPSRGen::cmdReceived(const char * cmd)
{
    float f1, f2, f3, f4, f5;
    int d1, d2, d3, d4;

    if (PEEK_CMD(cmd, "start"))
    {
        object_index = 0;
        target = objects[object_index]->name;
        type = objects[object_index]->type;
        cout << "IN OBJECT: " << target << endl;
        cout << "IN OBJECT TYPE:" << type << endl;
        publishActiveGraph();
    }
    if (PEEK_CMD(cmd, "next"))
    {
        object_index++;
        if (object_index >= objects.size())
        {
            object_index = objects.size();
            cout << "END!" << endl;
        }
        else
        {
            target = objects[object_index]->name;
            type = objects[object_index]->type;
            cout << "IN OBJECT: " << target << endl;
            cout << "IN OBJECT TYPE:" << type << endl;
            publishActiveGraph();
        }
    }
    if (PEEK_CMD(cmd, "clear"))
    {
        for(vector<INFO>::iterator Iter = infoes.begin(); Iter != infoes.end(); Iter++)
        {
            if(Iter->name == target && Iter->type == type)
            {
                infoes.erase(Iter);
                Iter = infoes.begin();
            }
        }
    }
    if (PEEK_CMD(cmd, "reset"))
    {
        for (int i = 0; i < objects.size(); i++)
        {
            if (objects[i]->name == target && objects[i]->type == type)
            {
                for (int j = 0; j < objects[i]->poses.size(); j++)
                {
                    this->infoes.push_back(INFO(this->objects[i]->name, objects[i]->type ,objects[i]->poses[j]));
                }
            }
        }
    }
    if (PEEK_CMD_N(cmd, "select", 6))
    {
        string name(cmd);
        name = name.substr(name.find(' ') + 1);
        target = name;
        int i = 0;
        for (i = 0; i < objects.size(); i++)
        {
          if (objects[i]->name == target)
          {
             break;
          }
        }
        object_index = i;
        publishActiveGraph();
    }
    if (PEEK_CMD(cmd, "print bigobjs"))
    {
        int id = 110;
        for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
        {
            if(info->type != "room")
            {
                cout << "'" << info->name << "'=>array('loc_id'=>'" << id << "','pose'=>'(" << info->position.getPoseX() <<"," << info->position.getPoseY() << "," << info->position.getPoseR()<< ")')," << endl;
                id++;
            }
        }
    }
    if (PEEK_CMD(cmd, "print rooms"))
    {
        int id = 101;
        for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
        {
            if(info->type == "room")
            {
                cout << "'" << info->name << "'=>array('loc_id'=>'" << id << "','point'=>'(" << info->position.getPoseX() << "," << info->position.getPoseY() << "," << info->position.getPoseR() <<")','id'=>'" << info->name << "'),"<< endl;
                id++;
            }
        }
    }
    if (PEEK_CMD(cmd, "print py"))
    {
        vector<Pose> print_pose;
        for (int i = 0; i < objects.size(); i++)
        {
            target = objects[i]->name;
            print_pose.clear();

            for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
            {
              if (info->name == target && info->type != "room")
              {
                    print_pose.push_back(info->position);
              }
            }
            if (print_pose.size() != 0)
            {
                cout << "\"" << target << "\":" << endl;
                cout << "[" << endl;
                for (vector<Pose>::iterator pose = print_pose.begin(); pose != print_pose.end(); pose++)
                {
                    cout << "[[" << pose->getPoseX()<< "," << pose->getPoseY()<<","<< pose->getPoseR()<<"],[0,-25,25]]," << endl;
                }
                cout << "]," << endl;
            }
        }
    }
}

void GPSRGen::pub()
{
    ros::Rate pub_rate(4);
    while (ros::ok())
    {
        showEntity(target);
        publishActiveGraph();
        pub_rate.sleep();
    }
}

void GPSRGen::publishActiveGraph()
{
  visualization_msgs::MarkerArray rooms;
  {
      rooms.markers.clear();
      visualization_msgs::Marker room;
      room.header.frame_id = "map";
      room.header.stamp = ros::Time::now();
      room.ns = "active target";
      room.action = visualization_msgs::Marker::ADD;
      room.type = visualization_msgs::Marker::CUBE;
      srand((unsigned int)(ros::Time::now().toNSec()));
      room.color.a = 0.5;

      const int colorNum = 1;
      std_msgs::ColorRGBA colors[colorNum];

      for (int i = 0; i < colorNum; i++)
      {
        colors[i].r = (float)(((float)rand()) / RAND_MAX);
        colors[i].g = (float)(((float)rand()) / RAND_MAX);
        colors[i].b = (float)(((float)rand()) / RAND_MAX);
        colors[i].a = 0.5;
      }

      for (size_t i = 0; i < objects.size(); i++)
      {
        if (objects[i]->name == target)
        {
            room.pose.position.x = (objects[i]->ranges.xmin + objects[i]->ranges.xmax) / 2.0;
            room.pose.position.y = (objects[i]->ranges.ymin + objects[i]->ranges.ymax) / 2.0;
            room.pose.position.z = 0.01;
            room.pose.orientation.x = 0;
            room.pose.orientation.y = 0;
            room.pose.orientation.z = 0;
            room.pose.orientation.w = 1.0;
            room.scale.x = fabs(objects[i]->ranges.xmin - objects[i]->ranges.xmax);
            room.scale.y = fabs(objects[i]->ranges.ymin - objects[i]->ranges.ymax);
            room.scale.z = 0.02;
            room.id = i;
            room.color = colors[0];

            rooms.markers.push_back(room);
        }
      }
  }
  maker_arry_pub.publish(rooms);
}

void GPSRGen::readConfig()
{
    string m_entitypath;
    if (private_nh.getParam("/gpsr_gen/entity_path", m_entitypath))
        cout << "get param entity_path success" << endl;
    else
        cout << "get param entity_path fail" << endl;
    m_entities.clear();
    FILE *fp = fopen(m_entitypath.c_str(), "r");
    if (fp)
    {
      Entity ee;
      char str[256];
      while (fscanf(fp, "%s %lf %lf %lf %lf\n", str, &ee.x1, &ee.y1, &ee.x2, &ee.y2) == 5)
      {
        string name(str);
        unsigned int pos = name.find_first_of('.');
        if (pos == string::npos)
        {
          ee.entity_name = name;
        }
        else
        {
          ee.class_id = name.substr(0, pos);
          ee.entity_name = name.substr(pos + 1);
        }
        m_entities.push_back(ee);
      }
      fclose(fp);
    }
    else
    {
        ROS_ERROR("Can't find file:%s",m_entitypath.c_str());
    }

}

void GPSRGen::registConfig()
{
    for (int i = 0; i < m_entities.size(); i++)
    {
        Entity &e = m_entities[i];
        if (e.class_id == "room")
        {
            Room* room = new Room(e);
            objects.push_back(room);
        }
        if (e.class_id == "furniture")
        {
            Furniture* fur = new Furniture(e);
            objects.push_back(fur);
        }
    }
}
void GPSRGen::showConfig()
{
    for (int i = 0; i < m_entities.size(); i++)
    {
        Entity &e = m_entities[i];
        cout << e.class_id << e.entity_name << e.x1 << e.y1 << e.x2 << e.y2 << endl;
    }
}

void GPSRGen::showEntity(string name)
{
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/map";
    cloud_msg.poses.clear();
    int i = 0;
    if (name == "all")
    {
        for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
        {
            {
              i++;
            }
        }
        {
            cloud_msg.poses.resize(i);
            i = 0;
            for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
            {
              {
                tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(info->position.getPoseR()), tf::Vector3(info->position.getPoseX(), info->position.getPoseY(), 0)), cloud_msg.poses[i]);
                i++;
    //            cout << info->position.getPoseX() << ", "<< info->position.getPoseY()<< ", "<< info->position.getPoseR()<<endl;
              }
            }
    //        cout << cloud_msg.poses.size() << endl;
        }
    }
    else
    {
        for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
        {
            if (info->name == name)
            {
              i++;
            }
        }
        {
            cloud_msg.poses.resize(i);
            i = 0;
            for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
            {
              if (info->name == name)
              {
                tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(info->position.getPoseR()), tf::Vector3(info->position.getPoseX(), info->position.getPoseY(), 0)), cloud_msg.poses[i]);
                i++;
    //            cout << info->position.getPoseX() << ", "<< info->position.getPoseY()<< ", "<< info->position.getPoseR()<<endl;
              }
            }
    //        cout << cloud_msg.poses.size() << endl;
        }
    }
    particle_pub.publish(cloud_msg);
}

void GPSRGen::deleteReceived(const geometry_msgs::PointStampedConstPtr &msg)
{
    double x = msg->point.x;
    double y = msg->point.y;
    delPose(target, x, y);
}

void GPSRGen::addReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double r = tf::getYaw(msg->pose.pose.orientation);
    addPose(target,type ,x, y, r);
}

void GPSRGen::onlyReceived(const geometry_msgs::PoseStampedConstPtr &msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double r = tf::getYaw(msg->pose.orientation);
    onlyPose(target,type ,x, y, r);
}

void GPSRGen::onlyPose(string name, string type, double x, double y, double z)
{
    double mindist = 9999;
    vector<INFO>::iterator select;
    int i = 0;
    for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
    {
      if (info->name == name)
      {
          i++;
          double tmp_x = info->position.getPoseX();
          double tmp_y = info->position.getPoseY();
          double tmp_z = info->position.getPoseR();
          double diff = fabs(x - tmp_x) + fabs(y - tmp_y) + 0.1 * fabs(z - tmp_z);
          if (diff < mindist)
          {
              mindist = diff;
              select = info;
          }

      }
    }
    if (mindist < 0.2)
    {
        x = select->position.getPoseX();
        y = select->position.getPoseY();
        z = select->position.getPoseR();
        for(vector<INFO>::iterator Iter = infoes.begin(); Iter != infoes.end(); Iter++)
        {
            if(Iter->name == target)
            {
                infoes.erase(Iter);
                Iter = infoes.begin();
            }
        }
        addPose(target,type ,x, y, z);
    }
}
void GPSRGen::addPose(string name, string type ,double x, double y, double z)
{
    infoes.push_back(INFO(name, type ,Pose(x,y,z)));
}

void GPSRGen::delPose(string name, double x, double y)
{
    double mindist = 9999;
    vector<INFO>::iterator select;
    int i = 0;
    for (vector<INFO>::iterator info = infoes.begin(); info != infoes.end(); info++)
    {
      if (info->name == name)
      {
          i++;
          double tmp_x = info->position.getPoseX();
          double tmp_y = info->position.getPoseY();
          double diff = fabs(x - tmp_x) + fabs(y - tmp_y);
          if (diff < mindist)
          {
              mindist = diff;
              select = info;
          }

      }
    }
    if (mindist < 0.2)
    {
        infoes.erase(select);
    }
}
