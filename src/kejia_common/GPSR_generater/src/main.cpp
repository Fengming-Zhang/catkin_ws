#include "gpsrgen.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Gconfig");
    ros::NodeHandle nh;
    GPSRGen gen;
    ros::spin();
    return 0;
}
