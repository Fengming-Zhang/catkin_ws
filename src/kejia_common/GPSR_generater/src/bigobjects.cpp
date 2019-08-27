#include "bigobjects.h"

BigObject::BigObject(Entity e)
{
    name = e.entity_name;
    type = e.class_id;
    ranges.xmin = (e.x1 <= e.x2)?e.x1:e.x2;
    ranges.xmax = (e.x1 >  e.x2)?e.x1:e.x2;
    ranges.ymin = (e.y1 <= e.y2)?e.y1:e.y2;
    ranges.ymax = (e.y1 >  e.y2)?e.y1:e.y2;

    cout << name << ranges.xmin << ranges.xmax << ranges.ymin << ranges.ymax << endl;
}

void BigObject::show()
{
    for (int i = 0; i < poses.size(); i++)
    {
        cout << poses[i].getPoseX() << poses[i].getPoseY() << poses[i].getPoseR() << endl;
    }
}
