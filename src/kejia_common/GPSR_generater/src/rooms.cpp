#include "rooms.h"

Room::Room(Entity e) : BigObject(e)
{
    cout << "room!!!!!!" << endl;
    dist = 1.2;
}

void Room::generatePose()
{
    poses.clear();
    poses.push_back(Pose(ranges.xmin + dist, ranges.ymin + dist, 3.14159 / 4));
    poses.push_back(Pose(ranges.xmin + dist, ranges.ymax - dist, -3.14159 / 4));
    poses.push_back(Pose(ranges.xmax - dist, ranges.ymin + dist, 3.14159 / 4 * 3));
    poses.push_back(Pose(ranges.xmax - dist, ranges.ymax - dist, -3.14159 / 4 * 3));
}
