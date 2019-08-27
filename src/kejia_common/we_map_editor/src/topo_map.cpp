#include <stdio.h>
#include "topo_map.h"
#include <string>
using namespace std;
void TopoMap::load_entities(const char *  file)
{
  m_entities.clear();
  FILE *fp = fopen(file, "r");
  if (fp)
  {
    Entity ee;
    char str[256];
    while (fscanf(fp, "%s %lf %lf %lf %lf\n", str, &ee.x1, &ee.y1, &ee.x2, &ee.y2) == 5)
    {
      string name(str);
      unsigned int pos = name.find_first_of('.');
      if (pos == string::npos)
      {
        ee.entity_name = name;
        printf("\nAttention:Entity id is nou found,this maybe cause some problems!\n");

      }
      else
      {
        ee.class_id = name.substr(0, pos);
        ee.entity_name = name.substr(pos + 1);
      }
      m_entities.push_back(ee);
    }
    fclose(fp);
  }
}

bool TopoMap::save_entities(const char *  file)
{
  if (m_entities.size() == 0)
  return true;

  FILE *fp = fopen(file, "w");
  if (fp)
  {
    for (int i = 0; i < (int)m_entities.size(); i++)
    {
      Entity &ee = m_entities[i];
      fprintf(fp, "%s.%s\t %8.3lf %8.3lf %8.3lf %8.3lf\n", ee.class_id.c_str(), ee.entity_name.c_str(), ee.x1, ee.y1,
          ee.x2, ee.y2);
    }
    fclose(fp);
    return true;
  }
  return false;
}

void TopoMap::addAnEntity(const char* type, const char* name, double x1, double y1, double x2, double y2)
{
    Entity ee;
    ee.class_id = string(type);
    ee.entity_name = string(name);
    ee.x1 = x1;
    ee.y1 = y1;
    ee.x2 = x2;
    ee.y2 = y2;
    m_entities.push_back(ee);
}

bool TopoMap::empty()
{
    return m_entities.size() == 0;
}


std::vector<Entity> & TopoMap::getEntityVector()
{
    return m_entities;

}
void TopoMap::clear()
{
    m_entities.clear();
}
