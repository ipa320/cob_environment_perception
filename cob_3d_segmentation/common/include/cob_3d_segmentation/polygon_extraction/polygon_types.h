#ifndef POLYGON_TYPES_H_
#define POLYGON_TYPES_H_

#include <vector>

namespace cob_3d_segmentation
{
  class PolygonPoint
  {
  public:
    PolygonPoint() : x(0), y(0) { }
    PolygonPoint(int ctor_x, int ctor_y) : x(ctor_x), y(ctor_y) { }

    virtual ~PolygonPoint() { };

    inline bool operator<(const PolygonPoint& rhs) const
    {
      if(y==rhs.y) { return x<rhs.x; }
      return y<rhs.y;
    }

    static int getInd(const int x, const int y) { return x + y * 640; }

  public:
    int x,y;
  };


  template <typename TPoint>
  class PolygonContours
  {
  public:
    virtual ~PolygonContours() { };

    void addPolygon() { polys_.push_back(std::vector<TPoint>()); }
    void removePolygon() { polys_.erase(polys_.end()-1); }
    void addPoint(int x, int y) { polys_.back().push_back(TPoint(x,y)); }
    /*void removeLastPoints(typename std::vector<TPoint>::size_type n)
    {
      if(polys_.back().size() > n) { polys_.back().resize(polys_.back().size() - n); }
      }*/
    void removeLastPoints(int n)
    {
      if(n && polys_.back().size() > n)
      {
        polys_.back().erase( polys_.back().begin()+(polys_.back().size()-n-1), polys_.back().end() );
      }
    }

  public:
    std::vector<std::vector<TPoint> > polys_;
  };
}

#endif
