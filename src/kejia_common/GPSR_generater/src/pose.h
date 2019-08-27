#ifndef POSE_H
#define POSE_H
#include<string>
using namespace std;
class Pose
{
public:
    Pose();
    Pose(float x, float y, float r, float h = 0.8);
    void setPose(float x, float y, float r, float h = 0.8);
    inline float getPoseX(){return _x;}
    inline float getPoseY(){return _y;}
    inline float getPoseR(){return _r;}
    inline float getPoseH(){return _h;}
private:
    float _x;
    float _y;
    float _r;
    float _h;
};

struct IPose{
    int index;
    Pose position;
};

class INFO{
public:
    INFO(string name, string type ,Pose pose);
    string name;
    string type;
    Pose position;
};
#endif // POSE_H
