/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description: Feature Map for storing and handling geometric features
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Heiko Hönnige,
 *
 * Date of creation: 10/2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>


// external includes
#include <boost/timer.hpp>
#include <Eigen/Geometry>
#include <pcl/win32_macros.h>
#include <pcl/common/transform.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/common/impl/transform.hpp>
#include "pcl_ros/transforms.h"

// TO DO
// trafo stimmt nicht
// ansicht bei schrägen eben stimmt au nicht muss wurzel nehmen

#include "cob_3d_mapping_geometry_map/vis/geometry_map_visualisation.h"


void
GeometryMapVisualisation::showPolygon(GeometryMap::MapEntryPtr polygon , int id)
{
	int number_of_points_ = polygon->polygon_world[id].size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& pc=*pc_ptr;
	pcl::PointCloud<pcl::PointXYZ> pc_out;


	int counter=0;
	int directionX_=1;
	int directionY_=1;
	for (int i =0  ;i<number_of_points_ ; i++)
	{
		double x=0;
		double m_forward_ , divisor_ , dividend_;


		if (i+1 < number_of_points_)
		{



		   dividend_=polygon->polygon_world[id][i+1][1]-polygon->polygon_world[id][i][1];
		   divisor_=polygon->polygon_world[id][i+1][0]-polygon->polygon_world[id][i][0];
		   m_forward_=dividend_/divisor_;

		   if(0 > divisor_) directionX_=-directionX_;
		   if(0 > dividend_) directionY_=-directionY_;


                         if(divisor_==0)
                         {
                           for( int j=0; j< directionY_*dividend_/0.01;j++)
                           {
                             pc.resize(counter+1);
                             pc.points[counter].x=polygon->polygon_world[id][i][0];
                             pc.points[counter].y =  directionY_ * x +polygon->polygon_world[id][i][1];
                             pc.points[counter].z=0;
                             x=x+directionX_*0.01;
                             counter++;
                           }
                         }
                         else
                         {
                           for( int j=0; j< directionX_ * divisor_/0.01;j++)
                           {
                             pc.resize(counter+1);
                             pc.points[counter].x=x+polygon->polygon_world[id][i][0];
                             pc.points[counter].y = m_forward_ * x +polygon->polygon_world[id][i][1];
                             pc.points[counter].z=0;
                             x=x+directionX_*0.01;
                             counter++;
                           }
                         }
		}
		else
		{

                    dividend_=polygon->polygon_world[id][0][1]-polygon->polygon_world[id][i][1];
                    divisor_=polygon->polygon_world[id][0][0]-polygon->polygon_world[id][i][0];
                    m_forward_=dividend_/divisor_;


                    if(0 > divisor_) directionX_=-directionX_;
                    if(0 > dividend_) directionY_=-directionY_;


                          if(divisor_==0)
                          {
                            for( int j=0; j<directionY_ * dividend_/0.01;j++)
                            {
                              pc.resize(counter+1);
                              pc.points[counter].x=polygon->polygon_world[id][i][0];
                              pc.points[counter].y = directionY_*x +polygon->polygon_world[id][i][1];
                              pc.points[counter].z=0;
                              x=x+directionX_*0.01;
                              counter++;
                            }
                          }
                          else
                          {
                            for( int j=0; j<directionX_ * divisor_/0.01;j++)
                            {
                              pc.resize(counter+1);
                              pc.points[counter].x=x+polygon->polygon_world[id][i][0];
                              pc.points[counter].y = m_forward_ * x +polygon->polygon_world[id][i][1];
                              pc.points[counter].z=0;
                              x=x+directionX_*0.01;
                              counter++;
                            }
                          }
		}

		directionX_=1;
		directionY_=1;
	}
	pc.resize(counter);
	pc.height=counter;
	pc.width=1;
	Eigen::Vector3f ft_pt;
	Eigen::Affine3f transform_from_world_to_plane;
	double l = -polygon->d/(polygon->polygon_world[id][0][0]+polygon->polygon_world[id][0][1]+polygon->polygon_world[id][0][2]); // warum minus ?
	ft_pt << l,l,l;
	getTransformationFromPlaneToWorld(polygon->normal, ft_pt, transform_from_world_to_plane);
	pcl::transformPointCloud(pc ,pc , transform_from_world_to_plane);
	//pcl::transformPointCloud(pc , pc , transform_from_world_to_plane);
	pcl::visualization::PCLVisualizer viewer ("3D viewer");
	viewer.setBackgroundColor(1,1,1);
	viewer.addCoordinateSystem(1.0f);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pc_col (pc_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(pc_ptr,pc_col,"pc");
	  while (!viewer.wasStopped ())
	  {
	    viewer.spinOnce (100);
	  }





	pcl::io::savePCDFile ("/home/goa-hh/pcl_daten/test.pcd", pc, true);
//	for (int i =0;i<pc.size();i++)       pcl::visualization::

//	{
//		ROS_INFO_STREAM("x  " <<pc.points[i].x <<"y   " << pc.points[i].y);
//	}

}
void
GeometryMapVisualisation::getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,
                                                   const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
{
  Eigen::Vector3f u, v;
  getCoordinateSystemOnPlane(normal, u, v);
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal,  origin, transformation);
  transformation = transformation.inverse();
}

void
GeometryMapVisualisation::getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
                                            Eigen::Vector3f &u, Eigen::Vector3f &v)
{
  v = normal.unitOrthogonal ();
  u = normal.cross (v);
}


int main (int argc, char** argv)
{
          GeometryMapVisualisation gmv;
	  GeometryMap::MapEntryPtr m_p = GeometryMap::MapEntryPtr(new GeometryMap::MapEntry());
	  m_p->id = 0;
	  m_p->normal << 0,0,1;
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
//	  v << 1,0,1;
//	  vv.push_back(v);
//	  v << 1,1,1;
//	  vv.push_back(v);
//	  v << 0,1,1;
//	  vv.push_back(v);
//	  v << 0,0,1;
//	  vv.push_back(v);

	  m_p->polygon_world.push_back(vv);
	  gmv.showPolygon(m_p,0);
}



