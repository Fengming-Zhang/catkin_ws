#ifndef ENTITY_H
#define ENTITY_H

#include <string>
using namespace std;

struct EntityEntry
{
  string entity_name;
  string class_id;
};

struct Entity : EntityEntry
{
  double x1, x2, y1, y2;
};

#endif // ENTITY_H
