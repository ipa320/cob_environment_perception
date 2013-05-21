/*
 * polygon_merger.cpp
 *
 *  Created on: 04.07.2012
 *      Author: josh
 */



#include "cob_3d_mapping_slam/curved_polygons/polygon_merger.h"



#include <boost/polygon/polygon.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

using namespace Slam_CurvedPolygon;
using namespace boost::polygon::operators;

typedef boost::polygon::polygon_with_holes_data<int> BoostPolygon;
typedef BoostPolygon::point_type BoostPoint;
typedef std::vector<BoostPolygon> BoostPolygonSet;
typedef boost::polygon::polygon_90_set_traits<BoostPolygonSet> BoostTraits;


void getBoostPolygon2D(const PolygonData &d, BoostPolygonSet &r) {
  BoostPolygon poly;
  std::vector<BoostPoint> seg;
  for(size_t i=0; i+1<d.get().size(); i+=2)
  {
//    std::cout<<"coord "<<d.get()[i]<<" "<<d.get()[i+1]<<"\n";
    if(d.get()[i]==d.get()[i] && d.get()[i+1]==d.get()[i+1] && std::abs(d.get()[i])<10000 && std::abs(d.get()[i+1])<10000)
      seg.push_back( BoostPoint(d.get()[i]*10000,d.get()[i+1]*10000) );
  }

  poly.set(seg.begin(), seg.end());
  r.push_back(poly);
}

void getPolygon2D(const BoostPolygonSet &poly, PolygonData &d) {
  for(size_t k=0; k<poly.size(); k++) {

    for(BoostPolygon::iterator_type it=poly[k].begin(); it!=poly[k].end(); it++) {
      if(it+1==poly[k].end()) break; //last is double

      //std::cout<<"coord "<<it->x()/10000.f<<" "<<it->y()/10000.f<<"\n";
      d.add(it->x()/10000.f,it->y()/10000.f);
    }
    break;

  }
}

void Slam_CurvedPolygon::mergePolygons(const PolygonData &a, const PolygonData &b, PolygonData &out)
{
  out.clear();

  BoostPolygonSet s1, s2;
  getBoostPolygon2D(a, s1);
  getBoostPolygon2D(b, s2);

  try {
    s1|=s2;
  }
  catch(...) {
    return;
  }

  getPolygon2D(s1, out);
}

float Slam_CurvedPolygon::unionPolygons(const PolygonData &a, const PolygonData &b, PolygonData &out, bool *sw)
{
  out.clear();

  if(a.get().size()<1||b.get().size()<1)
    return 0.f;

  BoostPolygonSet s1, s2;
  getBoostPolygon2D(a, s1);
  getBoostPolygon2D(b, s2);

  const float a1 = boost::polygon::area(s1);
  const float a2 = boost::polygon::area(s2);

  if(sw)
    *sw = a1>a2;

  try {
    s1&=s2;
  }
  catch(...) {
    std::cerr<<"ERRROR\n";
    return 0.f;
  }

  if(s1.size()<1)
    return 0.f;

  const float a3 = boost::polygon::area(s1);
  getPolygon2D(s1, out);

  //printf("here %f\n",2*a3/(a1+a2));

  return a3/(std::min(a1,a2));
}

float Slam_CurvedPolygon::area(const PolygonData &a)
{
  if(a.get().size()<1)
    return 0.f;

  BoostPolygonSet s1;
  getBoostPolygon2D(a, s1);

  return boost::polygon::area(s1);
}
