/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
/*
 * TestPlanes.cpp
 *
 *  Created on: 02.11.2011
 *      Author: goa-hh
 */

#include "cob_3d_mapping_geometry_map/test/TestPlanes.h"

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

void
TestPlanes::rotateTable(MapEntryPtr m_p ,  double alpha )
{
	  m_p->id = 0;
	  m_p->normal << 0,-cos((90.0-alpha)/180*3.1415926535),sin((90.0-alpha)/180*3.1415926535);
	  m_p->d = 1;

	  std::vector<Eigen::Vector3f> vv;
	  Eigen::Vector3f v;
	  v << 1,cos(alpha/180*3.1415926535),1+sin(alpha/180*3.1415926535);
	  vv.push_back(v);
	  v << -1,cos(alpha/180*3.1415926535),1+sin(alpha/180*3.1415926535);
	  vv.push_back(v);
	  v << -1,-cos(alpha/180*3.1415926535),1-sin(alpha/180*3.1415926535);
	  vv.push_back(v);
	  v << 1,-cos(alpha/180*3.1415926535),1-sin(alpha/180*3.1415926535);
	  vv.push_back(v);
	  m_p->polygon_world.push_back(vv);
}
