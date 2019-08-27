#ifndef ROOMS_H
#define ROOMS_H
#include "bigobjects.h"

class Room : public BigObject
{
public:
    Room(Entity e);

    float dist;
    void generateConfig(){}
    void generatePose();
};

#endif // ROOMS_H
