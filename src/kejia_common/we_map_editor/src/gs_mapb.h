#ifndef __gs_mapb_h_
#define __gs_mapb_h_

#include <stdio.h>
#include "gs_common.h"
#include <QImage>

namespace gslam
{
class GridMapBase
{
public:
  const static int LASER_MAXBEAM = 1100;
  const static int SIGHT_INCRE = 1;

  enum AccessibilityState
  {
    Outside = 0x0, Inside = 0x1, Allocated = 0x2
  };

  struct Cell
  {
    Cell() :
        acc(0, 0)
    {
      n = 0;
      visits = 0;
      occupied = 127;
      room = 255;
      path = 255;
      area = 0;
    }
    inline Point mean() const
    {
      return 1. / n * Point(acc.x, acc.y);
    }
    inline operator double() const
    {
      return visits ? (double)n * SIGHT_INCRE / (double)visits : -1;
    }
    inline operator int() const
    {
      return occupied;
    }
    inline void add(const Cell& p)
    {
      acc = acc + p.acc;
      n += p.n;
      visits += p.visits;
    }
    inline void calcocc()
    {
      if (visits)
        occupied = 255 - 255 * (double)n * SIGHT_INCRE / (double)visits;
    }
    inline double entropy() const
    {
      if (!visits)
        return -log(.5);
      if (n == visits || n == 0)
        return 0;
      double x = (double)n * SIGHT_INCRE / (double)visits;
      return -(x * log(x) + (1 - x) * log(1 - x));
    }
    inline void update(bool value, const Point& p = Point(0, 0))
    {
      if (value)
      {
        acc.x += static_cast<float>(p.x);
        acc.y += static_cast<float>(p.y);
        n++;
        visits += SIGHT_INCRE;
        if (occupied >= 12)
          occupied -= 12;
      }
      else
      {
        visits++;
        if (occupied <= 250)
          occupied += 5;
      }
    }

    FloatPoint acc;
    int n, visits;
    unsigned char occupied;
    unsigned char path;
    unsigned char area; /* 1--wall */
    unsigned char room; /* same in topograph */
  };

  template<class CellT>
    class Array2D
    {
    public:
      Array2D(int xsize = 0, int ysize = 0) :
          m_cells(0)
      {
        init(xsize, ysize);
      }
      Array2D(const Array2D &g) :
          m_cells(0)
      {
        *this = g;
      }
      Array2D& operator=(const Array2D &g)
      {
        if (m_xsize != g.m_xsize || m_ysize != g.m_ysize || !m_cells)
        {
          clear();
          init(g.m_xsize, g.m_ysize);
        }
        for (int x = 0; x < m_xsize; x++)
          for (int y = 0; y < m_ysize; y++)
            m_cells[x][y] = g.m_cells[x][y];
        return *this;
      }
      ~Array2D()
      {
        clear();
      }
      void init(int xsize, int ysize)
      {
        m_xsize = xsize;
        m_ysize = ysize;
        if (m_xsize > 0 && m_ysize > 0)
        {
          m_cells = new CellT*[m_xsize];
          CellT *cells = new CellT[m_xsize * m_ysize];
          for (int i = 0; i < m_xsize; i++)
            m_cells[i] = cells + i * m_ysize;
        }
        else
        {
          m_xsize = m_ysize = 0;
          m_cells = 0;
        }
      }
      void clear()
      {
        if (m_cells)
        {
          delete[] m_cells[0];
          delete[] m_cells;
        }
        m_cells = 0;
        m_xsize = 0;
        m_ysize = 0;
      }
      void resize(int xmin, int ymin, int xmax, int ymax)
      {
        int xsize = xmax - xmin;
        int ysize = ymax - ymin;

        Cell ** newcells = new CellT *[xsize];
        CellT *cells = new CellT[xsize * ysize];
        for (int x = 0; x < xsize; x++)
        {
          newcells[x] = cells + x * ysize;
        }
        int dx = xmin < 0 ? 0 : xmin;
        int dy = ymin < 0 ? 0 : ymin;
        int Dx = xmax < this->m_xsize ? xmax : this->m_xsize;
        int Dy = ymax < this->m_ysize ? ymax : this->m_ysize;
        for (int x = dx; x < Dx; x++)
        {
          for (int y = dy; y < Dy; y++)
          {
            newcells[x - xmin][y - ymin] = this->m_cells[x][y];
          }
        }
        delete[] this->m_cells[0];
        delete[] this->m_cells;
        this->m_cells = newcells;
        this->m_xsize = xsize;
        this->m_ysize = ysize;
      }
      inline bool isInside(int x, int y) const
      {
        return x >= 0 && y >= 0 && x < m_xsize && y < m_ysize;
      }
      inline const CellT& cell(int x, int y) const
      {
        assert(isInside(x,y));
        return m_cells[x][y];
      }
      inline CellT& cell(int x, int y)
      {
        assert(isInside(x,y));
        return m_cells[x][y];
      }

      inline AccessibilityState cellState(int x, int y) const
      {
        return (AccessibilityState)(isInside(x, y) ? (Inside | Allocated) : Outside);
      }

      inline bool isInside(const IntPoint& p) const
      {
        return isInside(p.x, p.y);
      }
      inline const CellT& cell(const IntPoint& p) const
      {
        return cell(p.x, p.y);
      }
      inline CellT& cell(const IntPoint& p)
      {
        return cell(p.x, p.y);
      }
      inline AccessibilityState cellState(const IntPoint& p) const
      {
        return cellState(p.x, p.y);
      }

      inline int getXSize() const
      {
        return m_xsize;
      }
      inline int getYSize() const
      {
        return m_ysize;
      }
      inline CellT** cells()
      {
        return m_cells;
      }

      CellT ** m_cells;
      int m_xsize, m_ysize;
    };

  typedef Array2D<Cell> Storage;

  //////////////////////////////////////////////////////////////////////////
  // for grid map representation and access
  Point m_center;
  double m_worldSizeX, m_worldSizeY, m_delta;
  int m_mapSizeX, m_mapSizeY;
  int m_sizeX2, m_sizeY2;
  Storage m_storage;

  static const Cell m_unknown;

  GridMapBase();

  void init(int mapSizeX, int mapSizeY, double delta);
  void init(const Point& center, double worldSizeX, double worldSizeY, double delta);
  void init(const Point& center, double xmin, double ymin, double xmax, double ymax, double delta, int mapX, int mapY);
  void resize(double xmin, double ymin, double xmax, double ymax);
  void grow(double xmin, double ymin, double xmax, double ymax);
  bool load(const string &path);
  void save(const string &path);
  void sweep();
  void calcooc();
  int denoise(int act);
  int edit(double cx, double cy, double lx, double ly, int action);

  inline IntPoint world2map(const Point& p) const
  {
    return IntPoint((int)round((p.x - m_center.x) / m_delta) + m_sizeX2,
                    (int)round((p.y - m_center.y) / m_delta) + m_sizeY2);
  }
  inline Point map2world(const IntPoint& p) const
  {
    return Point((p.x - m_sizeX2) * m_delta, (p.y - m_sizeY2) * m_delta) + m_center;
  }
  inline IntPoint world2map(double x, double y) const
  {
    return world2map(Point(x, y));
  }
  inline Point map2world(int x, int y) const
  {
    return map2world(IntPoint(x, y));
  }

  inline Point getCenter() const
  {
    return m_center;
  }
  inline double getWorldSizeX() const
  {
    return m_worldSizeX;
  }
  inline double getWorldSizeY() const
  {
    return m_worldSizeY;
  }
  inline int getMapSizeX() const
  {
    return m_mapSizeX;
  }
  inline int getMapSizeY() const
  {
    return m_mapSizeY;
  }

  inline double getDelta() const
  {
    return m_delta;
  }
  inline double getMapResolution() const
  {
    return m_delta;
  }
  inline double getResolution() const
  {
    return m_delta;
  }
  inline void getSize(double & xmin, double& ymin, double& xmax, double& ymax) const
  {
    Point min = map2world(0, 0), max = map2world(IntPoint(m_mapSizeX - 1, m_mapSizeY - 1));
    xmin = min.x, ymin = min.y, xmax = max.x, ymax = max.y;
  }

  inline Cell& cell(int x, int y)
  {
    return cell(IntPoint(x, y));
  }
  inline Cell& cell(const IntPoint& p)
  {
    return m_storage.cell(p);
  }

  inline const Cell& cell(int x, int y) const
  {
    return cell(IntPoint(x, y));
  }
  inline const Cell& cell(const IntPoint& p) const
  {
    AccessibilityState s = m_storage.cellState(p);
    if (s & Allocated)
      return m_storage.cell(p);
    return m_unknown;
  }

  inline Cell& cell(double x, double y)
  {
    return cell(Point(x, y));
  }
  inline Cell& cell(const Point& p)
  {
    IntPoint ip = world2map(p);
    AccessibilityState s = m_storage.cellState(ip);
    if (!s & Inside)
      assert(0);
    return m_storage.cell(ip);
  }

  inline const Cell& cell(double x, double y) const {	return cell(Point(x, y)); }

  inline bool isInside(int x, int y) const
  {
    return m_storage.cellState(IntPoint(x, y)) & Inside;
  }
  inline bool isInside(const IntPoint& p) const
  {
    return m_storage.cellState(p) & Inside;
  }

  inline bool isInside(double x, double y) const
  {
    return m_storage.cellState(world2map(x, y)) & Inside;
  }
  inline bool isInside(const Point& p) const
  {
    return m_storage.cellState(world2map(p)) & Inside;
  }

  inline const Cell& cell(const Point& p) const
  {
    return cell(p);
  }

  inline Storage& storage()
  {
    return m_storage;
  }
  inline const Storage& storage() const
  {
    return m_storage;
  }

  int getRange(Point lasp, float delta, float datas[]);

  void convertoImage(QImage & image);
};
}

#endif  //__gs_mapb_h_
