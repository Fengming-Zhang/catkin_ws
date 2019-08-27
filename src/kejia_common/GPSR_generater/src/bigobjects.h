#ifndef BIGOBJECTS_H
#define BIGOBJECTS_H

#include<vector>
#include"pose.h"
#include<string>
#include<iostream>
#include"entity.h"

using namespace std;

typedef struct Range{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
}Range;

class BigObject
{
public:
    BigObject(Entity e);
    void show();
    void addPose();
    void decPose();
    virtual void generatePose(){}
    virtual void generateConfig(){}
    vector<Pose> poses;//Manipulate places
    string name;
    string type;
    Range ranges;//Room's ranges
protected:
    void isInside();

};

#endif // BIGOBJECTS_H
