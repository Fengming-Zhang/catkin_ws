#include "pose.h"

INFO::INFO(string name, string type, Pose pose)
{
    this->name = name;
    this->type = type;
    this->position = pose;
}
Pose::Pose()
{
    _x=_y=_r=_h=0;
}

Pose::Pose(float x, float y, float r, float h)
{
    _x = x;
    _y = y;
    _r = r;
    _h = h;
}

void Pose::setPose(float x, float y, float r, float h)
{
    _x = x;
    _y = y;
    _r = r;
    _h = h;
}
