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
#include <pcl/common/transform.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
//#include <pcl/common/impl/transform.hpp>



#include "cob_3d_mapping_geometry_map/geometry_map.h"
//#include "cob_3d_mapping_geometry_map/vis/geometry_map_visualisation.h"

using namespace cob_3d_mapping;

void
GeometryMap::addMapEntry(PolygonPtr p_ptr)
{


  Polygon& p = *p_ptr;


/*
	  outputFile << "-----------------------------------------------------------------------------" << std::endl ;
	  outputFile << "new merge counter:" << counter_output << std::endl;
	  outputFile << "normal:" << std::endl << p.normal << std::endl << "d: " << p.d << std::endl;
	  for(int i=0; i< p.contours.size(); i++)
	  {
		  outputFile << i << std::endl;
	    for(int j=0; j< p.contours[i].size(); j++)
	    {
	    	outputFile << "(" << p.contours[i][j](0) << ", " << p.contours[i][j](1) << ", " << p.contours[i][j](2) << ")\n";
	    }
	  }

*/

   std::vector<int> intersections;
   //search all interesection with maps
   searchIntersection(p , intersections);
	//std::cout << "zweite normal " << std::endl << p.normal << std::endl ;
    //std::cout << "intersections :" << intersections.size() << std::endl;
    //outputFile << "after intersections" << std::endl;
    //outputFile << "normal:" << std::endl << p.normal << std::endl << "d: " << p.d << std::endl << " size of intersections: " << intersections.size()<<std::endl;


   //
   if(intersections.size()>0)
   {
	//merging with map
   mergeWithMap(p_ptr , intersections);
   }

   else  //if we has no interesection then new Entry
   {
		Eigen::Vector3f ft_pt;
	//	double x = -average_d/(average_normal(0)+average_normal(1)+average_normal(2));
	//	ft_pt << 1,1,-average_normal(0)/average_normal(2)-average_normal(1)/average_normal(2)+average_d/average_normal(2);
	 getPointOnPlane(p.normal,p.d,ft_pt);
     Eigen::Affine3f transformation_from_plane_to_world;
     getTransformationFromPlaneToWorld(p.normal, ft_pt/*p.contours[0][0]*/, transformation_from_plane_to_world);
     p.transform_from_world_to_plane = transformation_from_plane_to_world.inverse();
     p.merged++;
     p.id = new_id_;
     computeCentroid(p);
     map_.push_back(p_ptr);
     new_id_++;
//     std::cout << "feature added" << std::endl;
//     outputFile <<"new entry " <<std::endl;
//     outputFile <<"ID: " << map_.size()-1 << "trafo " << std::endl << p.transform_from_world_to_plane.matrix() <<std::endl;
//     outputFile <<"fp " << ft_pt << std::endl;




   }


   //printMap();


   //outputFile << "-----------------------------------------------------------------------------" << std::endl ;
   // printGpcStructure(&gpc_p);
   //gpc_free_polygon(&gpc_p);
   if(save_to_file_) saveMap(file_path_);

   //GeometryMapVisualisation gmv;
   //   gmv.showPolygon(map_[0]);
   //	std::cout << "map 0 normal : " <<std::endl << map_[0]->normal << std::endl << " und d: " << map_[0]->d << std::endl;


}

//void
//GeometryMap::sortContours()

void
GeometryMap::computeCentroid(Polygon& p)
{
  std::vector<pcl::PointXYZ> centroid;
  centroid.resize(p.contours[0].size ());
  pcl::PointCloud<pcl::PointXYZ> poly_cloud;
  for (unsigned int i = 0; i < p.contours[0].size (); i++)
  {
    pcl::PointXYZ pt;
    pt.x = p.contours[0][i][0];
    pt.y = p.contours[0][i][1];
    pt.z = p.contours[0][i][2];
    poly_cloud.push_back(pt);
  }
  pcl::compute3DCentroid(poly_cloud,p.centroid);
}

void
GeometryMap::searchIntersection(Polygon& p,std::vector<int>& intersections)
{
  //  Polygon& p = *p_ptr;

  for(size_t i=0; i< map_.size(); i++)
  {

    Polygon& p_map = *(map_[i]);
    //  std::cout << "berechnung " << p_map.normal.dot(p.normal);
    if((p_map.normal.dot(p.normal)>0.95 && fabs(p_map.d-p.d)<0.1) ||
        (p_map.normal.dot(p.normal)<-0.95 && fabs(p_map.d+p.d)<0.1))

    {


      gpc_polygon gpc_result;
      gpc_polygon gpc_p_merge;
      gpc_polygon gpc_p_map;

      Eigen::Affine3f transformation_from_world_to_plane;

      transformation_from_world_to_plane = p_map.transform_from_world_to_plane;
      getGpcStructureUsingMap(p, transformation_from_world_to_plane, &gpc_p_merge);

      getGpcStructureUsingMap(p_map, transformation_from_world_to_plane, &gpc_p_map);
      gpc_polygon_clip(GPC_INT, &gpc_p_merge, &gpc_p_map, &gpc_result);
      // 		  	  std::cout << "num contours intersect: " << gpc_result.num_contours << std::endl;
      if(gpc_result.num_contours == 0)
      {
        //std::cout << "no intersection with map " << i << std::endl;
        //std::cout << p.normal << std::endl;
        continue;
      }
      // std::cout << "intersection with map " << i << std::endl;
      intersections.push_back(i);

    }
  }
}

void
GeometryMap::mergeWithMap(PolygonPtr p_ptr , std::vector<int> intersections)
{

	Polygon& p = *p_ptr;
	Eigen::Vector3f average_normal=p.normal;
	double average_d=p.d;
	int merge_counter=1;
//	std::cout << "avg normal " << std::endl << average_normal << std::endl ;
//	std::cout << "p merged " << std::endl << map_[0]->merged << std::endl ;
//	std::cout << "p merged *normal  " << std::endl << map_[0]->merged*map_[0]->normal << std::endl ;


//	outputFile <<"merging with maps:" <<std::endl;

	for(int i=0 ; i<intersections.size();i++)
	{
		 Polygon& p_map = *(map_[intersections[i]]);
	//	 outputFile << "map: " << intersections[i] <<std::endl;
		 if(p.normal.dot(p_map.normal)<0){
	//	if (p.normal.dot(p_map.normal)<-0.95){
			 p_map.normal=-p_map.normal;
			 p_map.d=-p_map.d;
		 }
//		 outputFile <<"wird dazu addiert:  d: "<< p_map.d  <<std::endl;
//		 outputFile <<"normale" <<std::endl << p_map.normal << std::endl;


	//	 std::cout << " add normal :" << std::endl << p_map.normal << std::endl;
		 average_normal +=  p_map.merged* p_map.normal;
		 average_d +=p_map.merged * p_map.d;
		 merge_counter += p_map.merged;

	}
//	std::cout << "avg normal " << std::endl << average_normal << std::endl ;
//	std::cout << "merge counter " << std::endl << merge_counter << std::endl ;

	average_normal=average_normal/merge_counter;
	average_d=average_d/merge_counter;
	average_normal.normalize();

//	outputFile << "new System" <<std::endl;
//	outputFile <<"normal" <<std::endl << average_normal<<std::endl<<"d: " <<average_d<<std::endl;

//	std::cout << "avg normal " << std::endl << average_normal << std::endl ;
//	std::cout << "avg d :"  << average_d << std::endl ;


	Eigen::Vector3f ft_pt;

	getPointOnPlane(average_normal,average_d,ft_pt);
//	outputFile <<"ft_pt: "<< ft_pt <<std::endl;


	Eigen::Affine3f transformation_from_plane_to_world;
	Eigen::Affine3f transformation_from_world_to_plane;
	getTransformationFromPlaneToWorld(average_normal, ft_pt, transformation_from_plane_to_world);
	transformation_from_world_to_plane = transformation_from_plane_to_world.inverse();

	gpc_polygon gpc_result;
	gpc_polygon gpc_p_map;

//	 ROS_INFO_STREAM("merge trafo " << std::endl << transformation_from_world_to_plane.matrix());

//	outputFile << "new Trafo " << std::endl;
//	outputFile << transformation_from_world_to_plane.matrix()<< std::endl;

	getGpcStructureUsingMap(p, transformation_from_world_to_plane, &gpc_result);

	for(int i=0 ; i<intersections.size();i++)
	{

		Polygon& p_map = *(map_[intersections[i]]);



		getGpcStructureUsingMap(p_map, transformation_from_world_to_plane, &gpc_p_map);
		gpc_polygon_clip(GPC_UNION, &gpc_result, &gpc_p_map, &gpc_result);


		if (i==0)
		{
			p_map.transform_from_world_to_plane=transformation_from_world_to_plane;
			p_map.d=average_d;
			p_map.normal=average_normal;
			if(merge_counter<9)
			p_map.merged=merge_counter;
			else
				p_map.merged=9;


		}

	//	printGpcStructure(&gpc_result);
	//	std::cout << "map" << std::endl;
	//	printGpcStructure(&gpc_p_map);


		if(i!=0)
		{
			removeMapEntry(intersections[i]);
		}
	}
//	printGpcStructure(&gpc_result);

	Polygon& p_map = *(map_[intersections[0]]);

	p_map.contours.resize(gpc_result.num_contours);
	p_map.holes.resize(gpc_result.num_contours);

	for(int j=0; j<gpc_result.num_contours; j++)
	{
	  p_map.contours[j].resize(gpc_result.contour[j].num_vertices);
	  p_map.holes[j] = gpc_result.hole[j];
          //std::cout << "contour " << j << " is " << gpc_result.hole[j] << std::endl;
	  for(int k=0; k<gpc_result.contour[j].num_vertices; k++)
	  {
		//TODO: set z to something else?
		Eigen::Vector3f point(gpc_result.contour[j].vertex[k].x, gpc_result.contour[j].vertex[k].y, 0);
		p_map.contours[j][k] = p_map.transform_from_world_to_plane.inverse()*point;
		//TODO: update normal, d, transformation...?
	  }
	}
	//std::cout << std::endl;
//	std::cout << "Normale average" << average_normal[0] <<average_normal[1] <<average_normal[2] << std::endl;
//	std::cout << "d average" << average_d << std::endl;

}

void
GeometryMap::removeMapEntry(int id)
{


	map_.erase(map_.begin()+id);

//	for (int i=id ;i< map_.size();i++)
//	{
//		map_[i]->id=map_[i]->id-1;
//	}



}

void
GeometryMap::getGpcStructure(Polygon& p, gpc_polygon* gpc_p)
{
  //printMapEntry(p);
  gpc_p->num_contours = p.contours.size();
  gpc_p->hole = (int*)malloc(p.contours.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(p.contours.size()*sizeof(gpc_vertex_list));
  //std::cout << "num_contours: " << gpc_p->num_contours << std::endl;
  for(size_t j=0; j<p.contours.size(); j++)
  {
    //std::cout << j << std::endl;
    gpc_p->contour[j].num_vertices = p.contours[j].size();
    gpc_p->hole[j] = 0;
    //std::cout << "num_vertices: " << gpc_p->contour[j].num_vertices << std::endl;
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));
    for(size_t k=0; k<p.contours[j].size(); k++)
    {
      //std::cout << p.contours[j][k] << std::endl;
      Eigen::Vector3f point_trans = p.transform_from_world_to_plane*p.contours[j][k];
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);
      // if(fabs(point_trans(2))>0.01) std::cout << "z: " << point_trans(2) << std::endl;
      //std::cout << k << ":" << gpc_p->contour[j].vertex[k].x << "," << gpc_p->contour[j].vertex[k].y <<std::endl;
    }
  }
  //std::cout << "num_contours2: " << gpc_p->num_contours << std::endl;
  //horizontal plane
  /*if(fabs(p.normal.x) < 0.1 && fabs(p.normal.y) < 0.1 && fabs(p.normal.z) > 0.9)
  {
          v_list.vertex[j].x = p.polygons[i].points[j].x;
          v_list.vertex[j].y = p.polygons[i].points[j].y;
  }
  //vertical plane
  else if(fabs(p.normal.z) < 0.15)
  {
                  Eigen::Vector3f n2(p.normal.x, p.normal.y, p.normal.z);
                  n2.normalize();
                  Eigen::Vector3f n_x_world(1,0,0);
                  double angle_x = std::acos(n2.dot(n_x_world));
                  v_list.vertex[j].x = p.polygons[i].points[j].x*cos(angle_x)-p.polygons[i].points[j].y*sin(angle_x);
                  v_list.vertex[j].y = p.polygons[i].points[j].x*sin(angle_x)+p.polygons[i].points[j].y*cos(angle_x);
  }*/
}

void
GeometryMap::getGpcStructureUsingMap(Polygon& p, Eigen::Affine3f& transform_from_world_to_plane, gpc_polygon* gpc_p)
{
  //Eigen::Affine3f transformation_from_plane_to_world;
  //getTransformationFromPlaneToWorld(p.normal, p.contours[0][0], transformation_from_plane_to_world);
  //p.transform_from_world_to_plane = transform_from_world_to_plane;//transformation_from_plane_to_world.inverse();
  //printMapEntry(p);
  gpc_p->num_contours = p.contours.size();
  gpc_p->hole = (int*)malloc(p.contours.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(p.contours.size()*sizeof(gpc_vertex_list));
  //std::cout << "num_contours: " << gpc_p->num_contours << std::endl;
  for(size_t j=0; j<p.contours.size(); j++)
  {
    //std::cout << j << std::endl;
    gpc_p->hole[j] = p.holes[j];
    gpc_p->contour[j].num_vertices = p.contours[j].size();
    //std::cout << "num_vertices: " << gpc_p->contour[j].num_vertices << std::endl;
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));
    for(size_t k=0; k<p.contours[j].size(); k++)
    {
      //std::cout << p.contours[j][k] << std::endl;
      Eigen::Vector3f point_trans = transform_from_world_to_plane*p.contours[j][k];
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);

      //if(point_trans(2)>0.2 || point_trans(2)<-0.2) std::cout << "z: " << point_trans(2) << std::endl;
      //std::cout << k << ":" << gpc_p->contour[j].vertex[k].x << "," << gpc_p->contour[j].vertex[k].y <<std::endl;
    }
  }
}

void
GeometryMap::printMapEntry(Polygon& p)
{
  std::cout << "Polygon:\n";
  for(int i=0; i< p.contours.size(); i++)
  {
    std::cout << i << std::endl;
    for(int j=0; j< p.contours[i].size(); j++)
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

	ss << "/home/goa-hh/map/outputfile_" << counter_output << ".txt";
    std::ofstream outputFile2;
    outputFile2.open(ss.str().c_str());


	  for(int i=0; i< map_.size(); i++)
	  {

		  Polygon& p =*map_[i];

		     outputFile2 <<"ID: " << i << "trafo " << std::endl <<  p.transform_from_world_to_plane.matrix() <<std::endl;
			 outputFile2 << "normal:" << std::endl << p.normal << std::endl << "d: " << p.d << std::endl;
			 outputFile2 << "Polygon:\n";
			  for(int i=0; i< p.contours.size(); i++)
			  {
				  outputFile2 << i << std::endl;
			    for(int j=0; j< p.contours[i].size(); j++)
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
GeometryMap::saveMapEntry(std::string path, int ctr, Polygon& p)
{
  std::stringstream ss;
  ss << path << "polygon_" << ctr << ".pl";
  std::ofstream plane_file;
  plane_file.open (ss.str().c_str());
  plane_file << p.normal(0) << " " << p.normal(1) << " " << p.normal(2) << " " << p.d;
  ss.str("");
  ss.clear();
  plane_file.close();
  for(int i=0; i< p.contours.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> pc;
    ss << path << "polygon_" << ctr << "_" << i << ".pcd";
    for(int j=0; j< p.contours[i].size(); j++)
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
  static int ctr=0;
  std::stringstream ss;
  ss << path << "/" << ctr << "_";
  //std::cout << ctr << " Saving map with " << map_.size() << " entries..." << std::endl;
  for(size_t i=0; i< map_.size(); i++)
  {
      saveMapEntry(ss.str(), i, *map_[i]);
  }
  ctr++;
}

void
GeometryMap::clearMap()
{
  map_.clear();
}

void
GeometryMap::printGpcStructure(gpc_polygon* p)
{
  std::cout << "GPC Structure: " << std::endl;
  std::cout << "Num Contours: " << p->num_contours << std::endl;
  for(int i=0; i< p->num_contours; i++)
  {
    std::cout << i << std::endl;
    std::cout << "isHole: " << p->hole[i] << std::endl;
    std::cout << "Num points: " << p->contour[i].num_vertices << std::endl;
    for(int j=0; j< p->contour[i].num_vertices; j++)
    {
      std::cout << p->contour[i].vertex[j].x << " " << p->contour[i].vertex[j].y << "\n";
    }
  }
}

void
GeometryMap::getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
                                            Eigen::Vector3f &u, Eigen::Vector3f &v)
{
  v = normal.unitOrthogonal ();
  u = normal.cross (v);
}

void
GeometryMap::getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,
                                                   const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
{
  Eigen::Vector3f u, v;


  getCoordinateSystemOnPlane(normal, u, v);

//  std::cout << "u " << u <<  std::endl << " v " << v << std::endl;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal,  origin, transformation);
  transformation = transformation.inverse();
}

void
GeometryMap::getPointOnPlane(const Eigen::Vector3f &normal,double d,Eigen::Vector3f &point)
{
	// outputFile << "in getPointOnPlane" << std::endl;
	// outputFile << "normal 0 " << normal(0) << "normal 1 " << normal(1)<< "normal 2 " << normal(2)<< std::endl;

	float value=fabs(normal(0));
	int direction=0;
//	 outputFile << "abs normal 0 " << fabs(normal(0)) << "normal 1 " << fabs(normal(1))<< "normal 2 " << fabs(normal(2))<< std::endl;

	if(fabs(normal(1))>value)
	{

		direction=1;
		value=fabs(normal(1));
	}


	if(fabs(normal(2))>value)
	{
		direction=2;
		value=fabs(normal(2));
	}
	point << 0,0,0;
	point(direction)=-d/normal(direction);
//	 outputFile << "direction: " << direction << "  point " << std::endl << point << std::endl;

//	Eigen::Vector3f round_normal;
//	round_normal[0]=rounding(normal[0]);
//	round_normal[1]=rounding(normal[1]);
//	round_normal[2]=rounding(normal[2]);
//
//	int counter=0;
//	std::vector<int> no_zero_direction;
//	for(int i=0 ;i<3;i++)
//	{
//		if(round_normal[i]==0){
//			counter++;}
//		else
//		no_zero_direction.push_back(i);
//	}
////	std::cout << " normal " <<std::endl<< normal << std::endl;
////	std::cout << "counter " << counter;
//	if(counter==0)
//	{
//		point << 0,0,d/round_normal(2);
//	}
//	if(counter==1)
//	{
//		point << 0,0,0;
//		point[no_zero_direction[0]]=1;
//		point[no_zero_direction[1]]=-round_normal(no_zero_direction[0])/round_normal(no_zero_direction[1])+d/round_normal(no_zero_direction[1]);
//	}
//	if(counter==2)
//	{
//		point << 0,0,0;
//		point(no_zero_direction[0])=d/round_normal(no_zero_direction[0]);
//	}
}
float
GeometryMap::rounding(float x)

{
x *= 10000;
x += 0.5;
x = floor(x); 5;
x /= 10000;
return x;
}
/*int main (int argc, char** argv)
 {

  GeometryMap fm;
  for(int j=0; j<46; j++)
  {
    GeometryMap::PolygonPtr m_p = GeometryMap::PolygonPtr(new GeometryMap::Polygon());
    std::stringstream ss;
    ss << "/home/goa/pcl_daten/kitchen_kinect/polygons/polygon_" << j << ".txt";
    std::ifstream myfile;
    myfile.open (ss.str().c_str());
    if (myfile.is_open())
    {
        myfile >> m_p->id;
        std::cout << m_p->id << std::endl;
        myfile >> m_p->normal(0);
        myfile >> m_p->normal(1);
        myfile >> m_p->normal(2);
        std::cout << "normal:" << m_p->normal << std::endl;
        std::vector<Eigen::Vector3f> vv;
        int num;
        myfile >> num;
        std::cout << num << std::endl;
        int i=0;
        while ( i<num )
        {
          Eigen::Vector3f v;
          myfile >> v(0);
          myfile >> v(1);
          myfile >> v(2);
          vv.push_back(v);
          i++;
      }
        m_p->contours.push_back(vv);
      myfile.close();
    }
    //fm.printMapEntry(*m_p);
    fm.addMapEntry(m_p);

  }
  fm.saveMap("/home/goa/pcl_daten/kitchen_kinect/map");
}*/

int main (int argc, char** argv)
 {

  GeometryMap gm;
  GeometryMapVisualisation gmv;
  PolygonPtr m_p = PolygonPtr(new Polygon());




  m_p->id = 0;
  m_p->normal << 0,0,1;
  m_p->d = -1;
  std::vector<Eigen::Vector3f> vv;
  Eigen::Vector3f v;
  v << 1,0,1;
  vv.push_back(v);
  v << 1,1,1;
  vv.push_back(v);
  v << 0,1,1;
  vv.push_back(v);
  v << 0,0,1;
  vv.push_back(v);
  m_p->contours.push_back(vv);
//  gm.addMapEntry(m_p);


//  gmv.showPolygon(m_p,0);

  m_p = PolygonPtr(new Polygon());
  m_p->id = 1;
  m_p->normal << 0,0,4;
  m_p->d = 4;
  vv.clear();
  v << 1,0,-1;
  vv.push_back(v);
  v << 1,1,-1;
  vv.push_back(v);
  v << 0,1,-1;
  vv.push_back(v);
  v << 0,0,-1;
  vv.push_back(v);
  m_p->contours.push_back(vv);
  //gm.addMapEntry(m_p);

  /*m_p = PolygonPtr(new Polygon());
  m_p->id = 1;
  m_p->normal << 0,0,-1;
  m_p->d = -1;
  vv.clear();
  v << 2,2,1;
  vv.push_back(v);
  v << 2,3,1;
  vv.push_back(v);
  v << 3,3,1;
  vv.push_back(v);
  v << 3,2,1;
  vv.push_back(v);
  m_p->contours.push_back(vv);
  gm.addMapEntry(m_p);*/
  gm.saveMap("/home/goa/pcl_daten/merge_test/");

/*  Eigen::Vector3f test;
  Eigen::Vector3f result;
  double d=1;
  test << 1,1,1;
  gm.getPointOnPlane(test,d,result);
  std::cout << result(0) <<"," << result(1) <<","<< result(2);*/
 }



