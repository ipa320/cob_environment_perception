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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2011
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
//#include <pcl/common/transform.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
//#include <pcl/common/impl/transform.hpp>



#include "cob_3d_mapping_geometry_map/geometry_map.h"
using namespace cob_3d_mapping;





void
GeometryMap::addMapEntry(boost::shared_ptr<Polygon>& p_ptr)

{


	std::ofstream os ("/home/goa-tz/debug/DBG",std::ios::app);

	Polygon& p = *p_ptr;

	cob_3d_mapping::merge_config  limits;
	limits.d_thresh=d_;
	limits.angle_thresh=cos_angle_;
	limits.weighting_method="COMBINED";


	// find out polygons, to merge with
	std::vector<int> intersections;

	if (map_polygon_.size()> 0) {

		p.isMergeCandidate(map_polygon_,limits,intersections);
		// std::cout<<"intersections size = "<<intersections.size()<<std::endl;





		// if polygon has to be merged ...

		if(intersections.size()>0)
		{

			std::vector<boost::shared_ptr<Polygon> > merge_candidates;

			for(int i=0;i<(int)intersections.size();i++)
			{

				merge_candidates.push_back(map_polygon_[intersections[i]]);

				//							os<<"_____________________"<<std::endl;
				//							os<<"MAP:                "<<intersections[i]<<std::endl;
				//
				//							Polygon& p_new = *map_polygon_[intersections[i]];
				//							os<<"ID: "<<p_new.id<<std::endl;
				//							os<<"D:  "<<p_new.d<<std::endl;
				//
				//							os<<"_____________________"<<std::endl;
				//							os<<"NEW POLYGON:\n"<<std::endl;
				//
				//							os<<"ID: "<<p_ptr->id<<std::endl;
				//							os<<"D:  "<<p_ptr->d<<std::endl;


			}
			// merge polygon with merge candidates
			std::cout<<"merging with "<<merge_candidates.size()<<" shapes..."<<std::endl;
			//std::cout <<"c before: "<< p.centroid(0)<<", "<<p.centroid(1)<<", "<<p.centroid(2)<<std::endl;
			p.merge(merge_candidates);
			//std::cout <<"c after : "<< p.centroid(0)<<", "<<p.centroid(1)<<", "<<p.centroid(2)<<std::endl;


			//	  std::cout<<"size +- "<< 1 -merge_candidates.size()<<std::endl;



		}
		//if polygon does not have to be merged , add new polygon
		else
		{


			p.assignMembers();
			map_polygon_.push_back(p_ptr);
			new_id_++;


			//	std::cout<<"size +1"<<std::endl;
		}
	}

	else{

		p.assignMembers();
		p.assignWeight(limits.weighting_method);
		map_polygon_.push_back(p_ptr);

		new_id_++;
	}
	if(save_to_file_) saveMap(file_path_);
	std::cout<<"Map Size POLYGON"<<map_polygon_.size()<<"\n";


}

void
GeometryMap::addMapEntry(boost::shared_ptr<Cylinder>& c_ptr)

{


	Cylinder& c = *c_ptr;

	cob_3d_mapping::merge_config  limits;
	limits.d_thresh=d_;
	limits.angle_thresh=cos_angle_;
	//limits.weighting_method="AREA";
//	limits.weighting_method="COUNTER";
	limits.weighting_method="COMBINED";

	// find out polygons, to merge with
	std::vector<int> intersections;
//	if (map_cylinder_.size()> 0 )
//	{
		c.isMergeCandidate(map_cylinder_,limits,intersections);
		// std::cout<<"intersections size = "<<intersections.size()<<std::endl;
		std::cout<<"Intersection Size: "<<intersections.size()<<"\n";


		// if polygon has to be merged ...
		if(intersections.size()>0)
		{
			std::vector<boost::shared_ptr<Cylinder> > merge_candidates;

			for(int i=0;i<(int)intersections.size();i++)
			{

				merge_candidates.push_back(map_cylinder_[intersections[i]]);
			}
			// merge polygon with merge candidates
			c.merge(merge_candidates);

			//	  std::cout<<"size +- "<< 1 -merge_candidates.size()<<std::endl;
		}
//	}
	//if polygon does not have to be merged , add new polygon
	else
	{


		c.assignMembers(c.axes_[1],c.axes_[2],c.origin_);
		map_cylinder_.push_back(c_ptr);
		new_id_++;

		//	std::cout<<"size +1"<<std::endl;
	}

	std::cout<<"Map Size CYLINDER="<<map_cylinder_.size()<<std::endl;

//	if(save_to_file_) saveMap(file_path_);


}






void
GeometryMap::printMapEntry(cob_3d_mapping::Polygon& p)
{
	for(int i=0; i< (int)p.contours.size(); i++)
	{
		std::cout << i << std::endl;
		for(int j=0; j< (int)p.contours[i].size(); j++)
		{
			std::cout << "(" << p.contours[i][j](0) << ", " << p.contours[i][j](1) << ", " << p.contours[i][j](2) << ")\n";
		}
	}
	std::cout << "Normal: (" << p.normal(0) << ", " << p.normal(1) << ", " << p.normal(2) << ")\n";
	std::cout << "d: " << p.d << std::endl;
	std::cout << "Transformation:\n" << p.transform_from_world_to_plane.matrix() << "\n";
}


void
GeometryMap::printMap()
{
	std::stringstream ss;

	ss << "/home/goa-tz/GM_test/map/outputfile_" << counter_output << ".txt";
	std::ofstream outputFile2;
	outputFile2.open(ss.str().c_str());


	for(int i=0; i< (int)map_polygon_.size(); i++)
	{

		Polygon& p =*map_polygon_[i];

		outputFile2 <<"ID: " << i << "trafo " << std::endl <<  p.transform_from_world_to_plane.matrix() <<std::endl;
		outputFile2 << "normal:" << std::endl << p.normal << std::endl << "d: " << p.d << std::endl;
		outputFile2 << "Polygon:\n";
		for(int i=0; i< (int)p.contours.size(); i++)
		{
			outputFile2 << i << std::endl;
			for(int j=0; j< (int)p.contours[i].size(); j++)
			{
				outputFile2 << "(" << p.contours[i][j](0) << ", " << p.contours[i][j](1) << ", " << p.contours[i][j](2) << ")\n";
			}
		}
		outputFile2 << "----------------------------";

	}
	outputFile2.close();
	counter_output++;
}


void
GeometryMap::saveMapEntry(std::string path, int ctr, cob_3d_mapping::Polygon& p)
{
	std::stringstream ss;
	ss << path << "polygon_" << ctr << ".pl";
	std::ofstream plane_file;
	plane_file.open (ss.str().c_str());
	plane_file << p.normal(0) << " " << p.normal(1) << " " << p.normal(2) << " " << p.d;
	ss.str("");
	ss.clear();
	plane_file.close();
	for(int i=0; i< (int)p.contours.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ> pc;
		ss << path << "polygon_" << ctr << "_" << i << ".pcd";
		for(int j=0; j< (int)p.contours[i].size(); j++)
		{
			pcl::PointXYZ pt;
			pt.x = p.contours[i][j](0);
			pt.y = p.contours[i][j](1);
			pt.z = p.contours[i][j](2);
			pc.points.push_back(pt);
		}
		//std::cout << ss.str() << std::endl;
		pcl::io::savePCDFileASCII (ss.str(), pc);
		ss.str("");
		ss.clear();
	}
}


void
GeometryMap::saveMap(std::string path)
{

	//	only for polygons
	static int ctr=0;
	std::stringstream ss;
	ss << path << "/" << ctr << "_";
	//std::cout << ctr << " Saving map with " << map_.size() << " entries..." << std::endl;
	for(size_t i=0; i< map_polygon_.size(); i++)
	{
		saveMapEntry(ss.str(), i, *map_polygon_[i]);
	}
	ctr++;
}


void
GeometryMap::clearMap()
{
	map_polygon_.clear();
	map_cylinder_.clear();
}




float
GeometryMap::rounding(float x)

{
	x *= 10000;
	x += 0.5;
	x = floor(x);
	x /= 10000;
	return x;
}



void
GeometryMap::colorizeMap()
{


	//coloring for polygon
	for(unsigned int i=0; i<map_polygon_.size(); i++)
	{
		if(fabs(map_polygon_[i]->normal[2]) < 0.1) //plane is vertical
		{
			map_polygon_[i]->color[0] = 0.5;
			map_polygon_[i]->color[1] = 0.5;
			map_polygon_[i]->color[2] = 0;
			map_polygon_[i]->color[3] = 1;
		}
		else if(fabs(map_polygon_[i]->normal[0]) < 0.12 && fabs(map_polygon_[i]->normal[1]) < 0.12 && fabs(map_polygon_[i]->normal[2]) > 0.9) //plane is horizontal
		{
			map_polygon_[i]->color[0] = 0;
			map_polygon_[i]->color[1] = 0.5;
			map_polygon_[i]->color[2] = 0;
			map_polygon_[i]->color[3] = 1;
		}
		else
		{
			map_polygon_[i]->color[0] = 1;
			map_polygon_[i]->color[1] = 1;
			map_polygon_[i]->color[2] = 1;
			map_polygon_[i]->color[3] = 1;
		}
	}

	//coloring for cylinder
	for(unsigned int i=0; i<map_cylinder_.size(); i++)
	{
		if(fabs(map_cylinder_[i]->axes_[0][0]) < 0.1 && fabs(map_cylinder_[i]->axes_[0][1]) < 0.1) //cylinder is vertical
		{
			map_cylinder_[i]->color[0] = 0.5;
			map_cylinder_[i]->color[1] = 0.5;
			map_cylinder_[i]->color[2] = 0;
			map_cylinder_[i]->color[3] = 1;
		}
		else if(fabs(map_cylinder_[i]->axes_[0][2]) < 0.12) //plane is horizontal
		{
			map_cylinder_[i]->color[0] = 0;
			map_cylinder_[i]->color[1] = 0.5;
			map_cylinder_[i]->color[2] = 0;
			map_cylinder_[i]->color[3] = 1;
		}
		else
		{
			map_cylinder_[i]->color[0] = 1;
			map_cylinder_[i]->color[1] = 1;
			map_cylinder_[i]->color[2] = 1;
			map_cylinder_[i]->color[3] = 1;
		}
	}


}



int main (int argc, char** argv)
{
	GeometryMap gm;
	GeometryMapVisualisation gmv;


	Eigen::Vector3f v;
	std::vector<Eigen::Vector3f> vv;
	PolygonPtr m_p1 = PolygonPtr(new Polygon());
	m_p1->id = 1;
	m_p1->normal << 0.000000,-1.000000,-0.000000;
	m_p1->d = 0;
	v << 0.500000,0.010000,0.500000;
	vv.push_back(v);
	v << 0.500000,0.010000,-0.500000;
	vv.push_back(v);
	v << -0.500000,0.010000,-0.500000;
	vv.push_back(v);
	v << -0.500000,0.010000,0.500000;
	vv.push_back(v);
	m_p1->contours.push_back(vv);
	m_p1->holes.push_back(0);
	gm.addMapEntry(m_p1);


	vv.clear();
	PolygonPtr m_p2 = PolygonPtr(new Polygon());
	m_p2->id = 2;
	m_p2->normal << -0.000000,1.000000,0.000000;
	m_p2->d = 0;
	v << 0.500000,-0.010000,0.500000;
	vv.push_back(v);
	v << 0.500000,-0.010000,-0.500000;
	vv.push_back(v);
	v << -0.500000,-0.010000,-0.500000;
	vv.push_back(v);
	v << -0.500000,-0.010000,0.500000;
	vv.push_back(v);
	m_p2->contours.push_back(vv);
	m_p2->holes.push_back(0);
	gm.addMapEntry(m_p2);

	std::cout<<"done"<<std::endl;
	return 1;
}




