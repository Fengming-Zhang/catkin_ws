#ifndef FURNITURES_H
#define FURNITURES_H
#include "bigobjects.h"

class Furniture : public BigObject
{
public:
    Furniture(Entity e);
    int XNum;
    int YNum;

    float dist;

    void generateConfig(){}
    void generatePose();
};

#endif // FURNITURES_H
