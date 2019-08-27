#ifndef MAPIO_H
#define MAPIO_H
#include <cmath>
#include <vector>
#include <string>


using std::string;

struct EntityEntry
{
  string entity_name;
  string class_id;
};

struct Entity : EntityEntry
{
  double x1, x2, y1, y2;
  inline double xsize()
  {
    return fabs(x2 - x1);
  }
  inline double ysize()
  {
    return fabs(y2 - y1);
  }
  inline bool in_xbox(double x, double y)
  {
    return (x - x1) * (x - x1 - 0.12) <= 0 && (y - y2 + 0.12) * (y - y2) <= 0;
  }
};
class TopoMap
{
public:


   void load_entities(const char *  file);
   bool save_entities(const char *  file);
   bool empty();
   void clear();
   void addAnEntity(const char* type, const char* name, double x1, double y1, double x2, double y2);
   std::vector<Entity> & getEntityVector();
//private:
   std::vector<Entity> m_entities;
};

#endif // MAPIO_H
