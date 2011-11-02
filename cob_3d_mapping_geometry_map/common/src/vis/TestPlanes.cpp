/*
 * TestPlanes.cpp
 *
 *  Created on: 02.11.2011
 *      Author: goa-hh
 */

#include "../../include/cob_3d_mapping_geometry_map/vis/TestPlanes.h"

TestPlanes::TestPlanes() {
	// TODO Auto-generated constructor stub

}

TestPlanes::~TestPlanes() {
	// TODO Auto-generated destructor stub
}

void
TestPlanes::plane_1(GeometryMap::MapEntry& m_p)
{
	  m_p->id = 0;
	  m_p->normal << 0,1,0;
	  m_p->d = -1;
	  std::vector<Eigen::Vector3f> vv;
	  Eigen::Vector3f v;
	  v << 1,0,1;
	  vv.push_back(v);
	  v << 2,1,1;
	  vv.push_back(v);
	  v << 1,2,1;
	  vv.push_back(v);
	  v << 0,1,1;
	  vv.push_back(v);
	  m_p->polygon_world.push_back(vv);

}

void
TestPlanes::plane_2(GeometryMap::MapEntry& m_p)
{
	  m_p->id = 0;
	  m_p->normal << 0,1,0;
	  m_p->d = 2;
	  std::vector<Eigen::Vector3f> vv;
	  Eigen::Vector3f v;
	  v << 3,2,1;
	  vv.push_back(v);
	  v << -3,2,1;
	  vv.push_back(v);
	  v << -4,2,5;
	  vv.push_back(v);
	  v << 2,2,4;
	  vv.push_back(v);
	  m_p->polygon_world.push_back(vv);

}
