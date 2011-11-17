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
TestPlanes::plane_1(MapEntryPtr m_p)
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
TestPlanes::plane_2(MapEntryPtr m_p)
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

void
TestPlanes::overlap(MapEntryPtr m_p , MapEntryPtr m_p2 , double x)
{
	  m_p->id = 0;
	  m_p->normal << 0,0,1;
	  m_p->d = 1;
	  std::vector<Eigen::Vector3f> vv;
	  Eigen::Vector3f v;
	  v << 0,0,1;
	  vv.push_back(v);
	  v << 0,2,1;
	  vv.push_back(v);
	  v << -2,2,1;
	  vv.push_back(v);
	  v << -2,0,1;
	  vv.push_back(v);
	  m_p->polygon_world.push_back(vv);

	  m_p2->id = 0;
	  m_p2->normal << 0,0,1;
	  m_p2->d = 1;
	  std::vector<Eigen::Vector3f> vv2;
	  Eigen::Vector3f v2;
	  v2 << 0,x,1;
	  vv2.push_back(v2);
	  v2 << 0, 2+x,1;
	  vv2.push_back(v2);
	  v2 << -2, 2+x,1;
	  vv2.push_back(v2);
	  v2 << -2,x,1;
	  vv2.push_back(v2);
	  m_p2->polygon_world.push_back(vv2);
}

void
TestPlanes::moveplane(MapEntryPtr m_p , MapEntryPtr m_p2 , double x_1 , double x_2 , double x_3)
{
	  m_p->id = 0;
	  m_p->normal << 0,0,1;
	  m_p->d = 1;
	  std::vector<Eigen::Vector3f> vv;
	  Eigen::Vector3f v;
	  v << 0,0,1;
	  vv.push_back(v);
	  v << 0,1,1;
	  vv.push_back(v);
	  v << 1,1,1;
	  vv.push_back(v);
	  v << 1,0,1;
	  vv.push_back(v);
	  m_p->polygon_world.push_back(vv);

	  m_p2->id = 0;
	  m_p2->normal << 0,0,1;
	  m_p2->d = 1;
	  std::vector<Eigen::Vector3f> vv2;
	  Eigen::Vector3f v2;
	  v2 << x_1,x_2,1+x_3;
	  vv2.push_back(v2);
	  v2 << x_1,1+x_2 ,1+x_3;
	  vv2.push_back(v2);
	  v2 << 1+x_1, 1+x_2,1+x_3;
	  vv2.push_back(v2);
	  v2 << 1+x_1,x_2,1+x_3;
	  vv2.push_back(v2);
	  m_p2->polygon_world.push_back(vv2);
}

void
TestPlanes::moveX1X2(MapEntryPtr m_p , double x_1 , double x_2 , double x_3)
{
	  m_p->id = 0;
	  m_p->normal << 0,0,1;
	  m_p->d = 1;
	  std::vector<Eigen::Vector3f> vv;
	  Eigen::Vector3f v;
	  v << x_1,x_2,x_3;
	  vv.push_back(v);
	  v << x_1,1+x_2 ,x_3;
	  vv.push_back(v);
	  v << 1+x_1, 1+x_2,x_3;
	  vv.push_back(v);
	  v << 1+x_1,x_2,x_3;
	  vv.push_back(v);
	  m_p->polygon_world.push_back(vv);
}
void
TestPlanes::figure_1(MapEntryPtr m_p , MapEntryPtr m_p2 ,MapEntryPtr m_p3 , MapEntryPtr m_p4 ,MapEntryPtr m_p5 )
{
	  m_p->id = 0;
	  m_p->normal << 1,0,0;
	  m_p->d = 0;
	  std::vector<Eigen::Vector3f> vv;
	  Eigen::Vector3f v;
	  v << 0,0,0;
	  vv.push_back(v);
	  v << 0,1,0;
	  vv.push_back(v);
	  v << 0,1,1;
	  vv.push_back(v);
	  v << 0,0,1;
	  vv.push_back(v);
	  m_p->polygon_world.push_back(vv);

	  std::vector<Eigen::Vector3f> vv_2;
	  Eigen::Vector3f v_2;
	  v_2 << 0,2,0;
	  vv_2.push_back(v_2);
	  v_2 << 0,3,0;
	  vv_2.push_back(v_2);
	  v_2 << 0,3,1;
	  vv_2.push_back(v_2);
	  v_2 << 0,2,1;
	  vv_2.push_back(v_2);
	  m_p->polygon_world.push_back(vv_2);

	  m_p2->id = 0;
	  m_p2->normal << 0,0,1;
	  m_p2->d = 1;
	  std::vector<Eigen::Vector3f> vv2;
	  Eigen::Vector3f v2;
	  v2 << 0,1,1;
	  vv2.push_back(v2);
	  v2 << 1,1,1;
	  vv2.push_back(v2);
	  v2 << 1,2,1;
	  vv2.push_back(v2);
	  v2 << 0,2,1;
	  vv2.push_back(v2);
	  m_p2->polygon_world.push_back(vv2);

	  m_p3->id = 0;
	  m_p3->normal << 0,-1,0;
	  m_p3->d = 1;
	  std::vector<Eigen::Vector3f> vv3;
	  Eigen::Vector3f v3;
	  v3 << 0,1,0;
	  vv3.push_back(v3);
	  v3 << 1,1,0;
	  vv3.push_back(v3);
	  v3 << 1,1,1;
	  vv3.push_back(v3);
	  v3 << 0,1,1;
	  vv3.push_back(v3);
	  m_p3->polygon_world.push_back(vv3);

	  m_p4->id = 0;
	  m_p4->normal << 0,-1,0;
	  m_p4->d = 2;
	  std::vector<Eigen::Vector3f> vv4;
	  Eigen::Vector3f v4;
	  v4 << 0,2,0;
	  vv4.push_back(v4);
	  v4 << 1,2,0;
	  vv4.push_back(v4);
	  v4 << 1,2,1;
	  vv4.push_back(v4);
	  v4 << 0,2,1;
	  vv4.push_back(v4);
	  m_p4->polygon_world.push_back(vv4);

	  m_p5->id = 0;
	  m_p5->normal << 1,0,0;
	  m_p5->d = 1;
	  std::vector<Eigen::Vector3f> vv5;
	  Eigen::Vector3f v5;
	  v5 << 1,1,0;
	  vv5.push_back(v5);
	  v5 << 1,2,0;
	  vv5.push_back(v5);
	  v5 << 1,2,1;
	  vv5.push_back(v5);
	  v5 << 1,1,1;
	  vv5.push_back(v5);
	  m_p5->polygon_world.push_back(vv5);

}

void
TestPlanes::rotate(MapEntryPtr m_p , double x_1 , double x_2 , double alpha )
{
	  double alpha_rad = alpha *M_PI / 180;
	  m_p->id = 0;
	  m_p->normal << cos(-alpha_rad),-sin(-alpha_rad),0;
	  m_p->d = x_1*cos(-alpha_rad)-x_2*sin(-alpha_rad);
	  std::vector<Eigen::Vector3f> vv;
	  Eigen::Vector3f v;
	  v << x_1,x_2,0;
	  vv.push_back(v);
	  v << x_1,x_2,1;
	  vv.push_back(v);
	  v << x_1-sin(alpha_rad),x_2+cos(alpha_rad),1;
	  vv.push_back(v);
	  v << x_1-sin(alpha_rad),x_2+cos(alpha_rad),0;
	  vv.push_back(v);
	  m_p->polygon_world.push_back(vv);
}
