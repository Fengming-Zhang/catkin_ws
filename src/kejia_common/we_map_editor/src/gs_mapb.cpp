#include "gs_mapb.h"

namespace gslam
{
const GridMapBase::Cell GridMapBase::m_unknown = GridMapBase::Cell();

GridMapBase::GridMapBase()
{

}

void GridMapBase::init(int mapSizeX, int mapSizeY, double delta)
{
  m_storage.init((mapSizeX + 3) / 4 * 4, mapSizeY);
  m_worldSizeX = mapSizeX * delta;
  m_worldSizeY = mapSizeY * delta;
  m_delta = delta;
  m_center = Point(0.5 * m_worldSizeX, 0.5 * m_worldSizeY);
  m_sizeX2 = m_mapSizeX >> 1;
  m_sizeY2 = m_mapSizeY >> 1;
}

void GridMapBase::init(const Point& center, double worldSizeX, double worldSizeY, double delta)
{
  m_storage.init(((int)ceil(worldSizeX / delta) + 3) / 4 * 4, (int)ceil(worldSizeY / delta));
  m_center = center;
  m_worldSizeX = worldSizeX;
  m_worldSizeY = worldSizeY;
  m_delta = delta;
  m_mapSizeX = m_storage.getXSize();
  m_mapSizeY = m_storage.getYSize();
  m_sizeX2 = m_mapSizeX >> 1;
  m_sizeY2 = m_mapSizeY >> 1;
}

void GridMapBase::init(const Point& center, double xmin, double ymin, double xmax, double ymax, double delta, int mapX, int mapY)
{
  m_storage.init(mapX,mapY);
  m_center = center;
  m_worldSizeX = xmax - xmin;
  m_worldSizeY = ymax - ymin;
  m_delta = delta;
  m_mapSizeX = m_storage.getXSize();
  m_mapSizeY = m_storage.getYSize();
  m_sizeX2 = (int)round((m_center.x - xmin) / m_delta);
  m_sizeY2 = (int)round((m_center.y - ymin) / m_delta);
}

void GridMapBase::resize(double xmin, double ymin, double xmax, double ymax)
{
  IntPoint imin = world2map(xmin, ymin);
  IntPoint imax = world2map(xmax, ymax);
  int pxmin, pymin, pxmax, pymax;
  pxmin = (int)floor((float)imin.x);
  pxmax = (int)ceil((float)imax.x);
  pxmax = pxmin + (pxmax - pxmin + 3) / 4 * 4;
  pymin = (int)floor((float)imin.y);
  pymax = (int)ceil((float)imax.y);
  m_storage.resize(pxmin, pymin, pxmax, pymax);
  m_mapSizeX = m_storage.getXSize();
  m_mapSizeY = m_storage.getYSize();
  m_worldSizeX = xmax - xmin;
  m_worldSizeY = ymax - ymin;
  m_sizeX2 -= pxmin;
  m_sizeY2 -= pymin;
}

void GridMapBase::grow(double xmin, double ymin, double xmax, double ymax)
{
  IntPoint imin = world2map(xmin, ymin);
  IntPoint imax = world2map(xmax, ymax);
  if (isInside(imin) && isInside(imax))
    return;
  imin = min(imin, IntPoint(0, 0));
  imax = max(imax, IntPoint(m_mapSizeX - 1, m_mapSizeY - 1));
  int pxmin, pymin, pxmax, pymax;
  pxmin = (int)floor((float)imin.x);
  pxmax = (int)ceil((float)imax.x);
  pymin = (int)floor((float)imin.y);
  pymax = (int)ceil((float)imax.y);
  m_storage.resize(pxmin, pymin, pxmax, pymax);
  m_mapSizeX = m_storage.getXSize();
  m_mapSizeY = m_storage.getYSize();
  m_worldSizeX = xmax - xmin;
  m_worldSizeY = ymax - ymin;
  m_sizeX2 -= pxmin;
  m_sizeY2 -= pymin;
}

bool GridMapBase::load(const string &path)
{
  cout << "map path: " << path << endl;
  FILE *fp = fopen(path.c_str(), "rb");
  if (fp)
  {
    double header[8];
    fread(header, sizeof(double), 8, fp);

    int mapSize[2];
    fread(mapSize, sizeof(int),2, fp);
    init(Point(header[0], header[1]), header[2], header[3], header[4], header[5], header[6],mapSize[0], mapSize[1]);

    int cellCount = 0;
    for (int y = 0; y < m_mapSizeY; y++)
    {
      for (int x = 0; x < m_mapSizeX; x++)
      {
        Cell p;
        fread(&p.acc.x, sizeof(float), 1, fp);
        fread(&p.acc.y, sizeof(float), 1, fp);
        fread(&p.n, sizeof(int), 1, fp);
        fread(&p.visits, sizeof(int), 1, fp);
        p.calcocc();
        p.area = p.n > 0;
        cell(x, y) = p;
        cellCount++;
      }
    }

    fclose(fp);
    printf("------------------- Map info --------------------\n");
    printf("\tcenter: [%.4f, %.4f]\n", header[0], header[1]);
    printf("\tlt pose: [%.4f, %.4f]\n", header[2], header[3]);
    printf("\trd pose: [%.4f, %.4f]\n", header[4], header[5]);
    printf("\tdelta: %.4f\n", header[6]);
    printf("\tmap size x: %d - %d\n", mapSize[0],m_mapSizeX);
    printf("\tmap size y: %d - %d\n", mapSize[1],m_mapSizeY);
    printf("\tcell size: %d\n", cellCount);
    printf("\tsizeof double: %.d\n", sizeof(double));
    printf("--------------------------------------------------\n");

    //denoise(0);
    //saveGridMap("slam/gridmap_6-6-x.map");
    return true;
  }

  return false;
}

void GridMapBase::save(const string &path)
{
  FILE *fp = fopen(path.c_str(), "wb");
  if (fp)
  {
    Point pmin = map2world(IntPoint(0, 0));
    Point pmax = map2world(IntPoint(m_mapSizeX, m_mapSizeY));

    double header[8] = {m_center.x, m_center.y, pmin.x, pmin.y, pmax.x, pmax.y, m_delta, 0};
    int mapSize[2] = {m_mapSizeX,m_mapSizeY};

    fwrite(&header[0], sizeof(double), 8, fp);
    fwrite(&mapSize[0], sizeof(int), 2, fp);

    for (int y = 0; y < m_mapSizeY; y++)
    {
      for (int x = 0; x < m_mapSizeX; x++)
      {
        Cell &p = cell(x, y);
        fwrite(&p.acc.x, sizeof(float), 1, fp);
        fwrite(&p.acc.y, sizeof(float), 1, fp);
        fwrite(&p.n, sizeof(int), 1, fp);
        fwrite(&p.visits, sizeof(int), 1, fp);
      }
    }
    fclose(fp);
  }
}

int GridMapBase::denoise(int act)
{
  int num = 0;

  for (int y = 1, i = 0; y < m_mapSizeY - 1; y++)
  {
    for (int x = 1; x < m_mapSizeX - 1; x++)
    {
      if (m_storage.m_cells[x][y].n)
      {
        int cc = ((double)m_storage.m_cells[x - 1][y] > 0.5) + ((double)m_storage.m_cells[x + 1][y] > 0.5)
            + ((double)m_storage.m_cells[x][y - 1] > 0.5) + ((double)m_storage.m_cells[x][y + 1] > 0.5);

        if (cc == 0)
        {
          m_storage.m_cells[x][y].n = 0;
          m_storage.m_cells[x][y].calcocc();
          num++;
        }
        else if (act == 1)
        {
          m_storage.m_cells[x][y].area = m_storage.m_cells[x][y].n > 0;
        }
      }
    }
  }

  return num;
}

void GridMapBase::sweep()
{
  int num = 0;
  for (int y = 1, i = 0; y < m_mapSizeY - 1; y++)
  {
    for (int x = 1; x < m_mapSizeX - 1; x++)
    {
      if (m_storage.m_cells[x][y].area == 0 && m_storage.m_cells[x][y].visits)
      {
        m_storage.m_cells[x][y].occupied = 255;
      }
    }
  }
}

void GridMapBase::calcooc()
{
  int num = 0;
  for (int y = 0, i = 0; y < m_mapSizeY; y++)
  {
    for (int x = 0; x < m_mapSizeX; x++)
    {
      m_storage.m_cells[x][y].calcocc();
    }
  }
}

int GridMapBase::edit(double cx, double cy, double lx, double ly, int action)
{
  IntPoint p1 = world2map(cx - lx, cy - ly);
  IntPoint p2 = world2map(cx + lx, cy + ly);
  if (action == 0)
  {

  }
  return 0;
}

int GridMapBase::getRange(Point lasp, float delta, float datas[])
{
  IntPoint ip = world2map(lasp);
  int idx = 0;
  for (float rot = 0; rot < 6.28318f; rot += delta, idx += 1)
  {
    int x, y, i = 0;
    float s = sin(rot);
    float c = cos(rot);
    while (1)
    {
      i++;
      x = int(ip.x + i * c + 0.5);
      y = int(ip.y + i * s + 0.5);
      if (!(x >= 0 && x < m_storage.m_xsize && y >= 0 && y < m_storage.m_ysize))
      {
        datas[idx] = i * m_delta;
        break;
      }
      else if (m_storage.cell(x, y).occupied < 240 || m_storage.cell(x, y).area > 1)
      {
        datas[idx] = i * m_delta;
        if (datas[idx] < 0.225)
          continue;
        else
          break;
      }
    }
  }
  return 0;
}

void GridMapBase::convertoImage(QImage & image)
{
    image = QImage(QSize(m_mapSizeX,m_mapSizeY), QImage::Format_Indexed8);

    QRgb value;
    value = qRgb(0, 0, 0); // 0xff7aa327
    image.setColor(0, value);

    value = qRgb(255, 255, 255); // 0xffedba31
    image.setColor(1, value);

    value = qRgb(185, 219, 217); // 0xffbd9527
    image.setColor(2, value);
    for (int y = 0; y < m_mapSizeY; y++)
      {
        for (int x = 0; x < m_mapSizeX; x++)
        {
          double v = cell(IntPoint(x, y));
          if (v >= 0.1)
              image.setPixel(x,y,0);
          else if (v >= 0 && v < 0.1)
            image.setPixel(x,y,1);
          else
            image.setPixel(x,y,2);
        }
      }
}

}
