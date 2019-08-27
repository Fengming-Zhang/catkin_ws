#include "furnitures.h"

Furniture::Furniture(Entity e) : BigObject(e)
{
    cout << "furniture!!!!!!" << endl;
    XNum = (ranges.xmax - ranges.xmin) / 0.8;
    YNum = (ranges.ymax - ranges.ymin) / 0.8;
    if (XNum == 0 && YNum == 0)
    {
        XNum = 1;
        YNum = 1;
    }
    dist = 0.70;
}

void Furniture::generatePose()
{
    poses.clear();
    //left
    for (int i = 1; i <= YNum; i++)
    {
        float x = ranges.xmin - dist;
        float y = i * (ranges.ymax - ranges.ymin) / (YNum + 1) + ranges.ymin;
        float r = 0;
        poses.push_back(Pose(x, y, r));
    }

    //up
    for (int i = 1; i <= XNum; i++)
    {
        float x = i * (ranges.xmax - ranges.xmin) / (XNum + 1) + ranges.xmin;
        float y = ranges.ymax + dist;
        float r = -3.14159 / 2;
        poses.push_back(Pose(x, y, r));
    }

    //right
    for (int i = 1; i <= YNum; i++)
    {
        float x = ranges.xmax + dist;
        float y = i * (ranges.ymax - ranges.ymin) / (YNum + 1) + ranges.ymin;
        float r = 3.14159;
        poses.push_back(Pose(x, y, r));
    }

    //down
    for (int i = 1; i <= XNum; i++)
    {
        float x = i * (ranges.xmax - ranges.xmin) / (XNum + 1) + ranges.xmin;
        float y = ranges.ymin - dist;
        float r = 3.14159 / 2;
        poses.push_back(Pose(x, y, r));
    }

}
