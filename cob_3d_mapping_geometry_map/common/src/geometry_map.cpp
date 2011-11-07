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
//#include <pcl/common/impl/transform.hpp>


#include "cob_3d_mapping_geometry_map/geometry_map.h"
//#include "cob_3d_mapping_geometry_map/vis/geometry_map_visualisation.h"



//void
//GeometryMap::addMapEntry(GeometryMap::MapEntryPtr p_ptr)
//{
//  GeometryMap::MapEntry& p = *p_ptr;
//  //ROS_INFO("polygonCallback");
//  bool merged = false;
//  p.d = p.d/p.normal.norm();
//  p.normal.normalize();
//  //gpc_polygon gpc_p;
//  //getGpcStructure(p, &gpc_p);
//  /*printGpcStructure(&gpc_p);
//  std::stringstream ss;
//  ss << "/home/goa/pcl_daten/kitchen_kinect/outfile_" << entry_ctr_ << ".txt";
//  FILE *out;*/
//  //out= fopen(ss.str().c_str(), "w");
//  //gpc_write_polygon(out, 0, &p.polygon_plane);
//  //fclose(out);
//  // Iterate over map
//  for(size_t i=0; i< map_.size(); i++)
//  {
//    //std::cout << "loop" << std::endl;
//    MapEntry& p_map = *(map_[i]);
//    Eigen::Vector3f n2(p.normal(0), p.normal(1), p.normal(2));
//    //n2.normalize();
//    //double angle = fabs(std::acos(p_map.normal.dot(n2)));
//    /*if(n2(0)>0.9 || n2(0)<-0.9)
//    {
//      std::cout << "x axis" << std::endl;
//      std::cout << "n:" << n2 << std::endl;
//      std::cout << "n_map:" << p_map.normal << std::endl;
//      std::cout << "d, d_map:" << p.d << "," << p_map.d << std::endl;
//    }*/
//    if(fabs(p_map.normal.dot(n2))>0.95 && fabs(fabs(p_map.d)-fabs(p.d))<0.25) //0.97 = 14 degree; 0.95,0.15
//    //if(angle<0.05 && fabs(p_map.d-p.d)<0.08) //planes can be merged
//    {
//      gpc_polygon gpc_result;
//      gpc_polygon gpc_p_merge;
//      gpc_polygon gpc_p_map;
//      double d_p = p.d;
//      if(p_map.normal.dot(n2)<-0.95) {n2 = -n2, d_p=-d_p;}
//      Eigen::Vector3f n_map = (p_map.normal + n2)/2;
//      double d_map = (p_map.d + d_p)/2;
//      d_map = d_map/n_map.norm();
//      n_map.normalize();
//      /*n_map(0) = (p_map.normal(0) + n2(0))/2;
//      n_map(1) = (p_map.normal(1) + n2(1))/2;
//      n_map(2) = (p_map.normal(2) + n2(2))/2;*/
//      Eigen::Vector3f ft_pt;
//      if(fabs(n_map(2))>0.01)
//        ft_pt << 0, 0, -d_map/n_map(2);
//      else if(fabs(n_map(0))>0.01)
//        ft_pt << -d_map/n_map(0), 0, 0;
//      else if(fabs(n_map(1))>0.01)
//        ft_pt << 0, -d_map/n_map(1), 0;
//      Eigen::Affine3f transformation_from_plane_to_world;
//      Eigen::Affine3f transformation_from_world_to_plane;
//      getTransformationFromPlaneToWorld(n_map, ft_pt, transformation_from_plane_to_world);
//      transformation_from_world_to_plane = transformation_from_plane_to_world.inverse();
//      getGpcStructureUsingMap(p, transformation_from_world_to_plane, &gpc_p_merge);
//      //std::cout <<  p.transform_from_world_to_plane.matrix() << std::endl;
//      //printGpcStructure(&gpc_p_merge);
//      //getGpcStructure(p, &gpc_p_merge);
//      //std::cout <<  p.transform_from_world_to_plane.matrix() << std::endl;
//      //printGpcStructure(&gpc_p_merge);
//      getGpcStructureUsingMap(p_map, transformation_from_world_to_plane, &gpc_p_map);
//      //printGpcStructure(&gpc_p_map);
//      gpc_polygon_clip(GPC_UNION, &gpc_p_merge, &gpc_p_map, &gpc_result);
//      if(gpc_result.num_contours == 0) //merge failed
//      {
//        std::cout << "merge failed" << std::endl;
//        continue;
//      }
//      else if(gpc_result.num_contours >= gpc_p_map.num_contours + gpc_p_merge.num_contours)
//      {
//        std::cout << "merge failed" << std::endl;
//        std::cout << "feature id: " << i << std::endl;
//        continue;
//      }
//      else
//      {
//        /*if(fabs(p_map.normal(1))>0.8)
//        {
//          printMapEntry(p_map);
//          std::cout << "Trafo from plane to world: " << std::endl;
//          std::cout <<  p_map.transform_from_world_to_plane.inverse().matrix() << std::endl;
//        }*/
//        //gpc_result = gpc_p_map;
//        p_map.transform_from_world_to_plane = transformation_from_world_to_plane;
//        p_map.normal = n_map;
//        p_map.d = d_map;
//        p_map.polygon_world.resize(gpc_result.num_contours);
//        /*for(int j=0; j<gpc_p_merge.num_contours; j++)
//        {
//          p_map.polygon_world[j].resize(gpc_p_merge.contour[j].num_vertices);
//          for(int k=0; k<gpc_p_merge.contour[j].num_vertices; k++)
//          {
//            //TODO: set z to something else?
//            Eigen::Vector3f point(gpc_p_merge.contour[j].vertex[k].x, gpc_p_merge.contour[j].vertex[k].y, 0);
//            p_map.polygon_world[j][k] = p_map.transform_from_world_to_plane.inverse()*point;
//            //TODO: update normal, d, transformation...?
//          }
//        }*/
//        //printGpcStructure(&gpc_result);
//        for(int j=0; j<gpc_result.num_contours; j++)
//        {
//          p_map.polygon_world[j].resize(gpc_result.contour[j].num_vertices);
//          //std::cout << p_map.polygon_world[j].size() << std::endl;
//          for(int k=0; k<gpc_result.contour[j].num_vertices; k++)
//          {
//            //TODO: set z to something else?
//            Eigen::Vector3f point(gpc_result.contour[j].vertex[k].x, gpc_result.contour[j].vertex[k].y, 0);
//            p_map.polygon_world[j][k] = p_map.transform_from_world_to_plane.inverse()*point;
//            //TODO: update normal, d, transformation...?
//          }
//        }
//        std::cout << i << ": feature merged" << std::endl;
//        /*if(fabs(p_map.normal(1))>0.8)
//        {
//          printGpcStructure(&gpc_result);
//          printMapEntry(p_map);
//        }*/
//        merged = true;
//        p_map.merged++;
//        break;
//      }
//      gpc_free_polygon(&gpc_result);
//      gpc_free_polygon(&gpc_p_merge);
//      gpc_free_polygon(&gpc_p_map);
//    }
//  }
//  //new entry
//  if(!merged)
//  {
//    Eigen::Affine3f transformation_from_plane_to_world;
//    getTransformationFromPlaneToWorld(p.normal, p.polygon_world[0][0], transformation_from_plane_to_world);
//    p.transform_from_world_to_plane = transformation_from_plane_to_world.inverse();
//    p.id = new_id_;
//    map_.push_back(p_ptr);
//    new_id_++;
//    std::cout << "feature added" << std::endl;
//    //ROS_INFO("added new feature");
//  }
//  //printGpcStructure(&gpc_p);
//  //gpc_free_polygon(&gpc_p);
//  //saveMap("/home/goa/pcl_daten/kitchen_kinect2/map");
//
//}

void
GeometryMap::addMapEntry(MapEntryPtr p_ptr)
{
  MapEntry& p = *p_ptr;
  //ROS_INFO("polygonCallback");
  bool merged = false;
  //p.d = p.d/p.normal.norm();
  //p.normal.normalize();
  //gpc_polygon gpc_p;
  //getGpcStructure(p, &gpc_p);
  /*printGpcStructure(&gpc_p);
  std::stringstream ss;
  ss << "/home/goa/pcl_daten/kitchen_kinect/outfile_" << entry_ctr_ << ".txt";
  FILE *out;*/
  //out= fopen(ss.str().c_str(), "w");
  //gpc_write_polygon(out, 0, &p.polygon_plane);
  //fclose(out);
  // Iterate over map

  for(size_t i=0; i< map_.size(); i++)
  {
		//std::cout << "loop" << std::endl;
		  MapEntry& p_map = *(map_[i]);
		//Eigen::Vector3f n2(p.normal(0), p.normal(1), p.normal(2));
		//n2.normalize();
		//double angle = fabs(std::acos(p_map.normal.dot(n2)));
		/*if(n2(0)>0.9 || n2(0)<-0.9)
		{
		  std::cout << "x axis" << std::endl;
		  std::cout << "n:" << n2 << std::endl;
		  std::cout << "n_map:" << p_map.normal << std::endl;
		  std::cout << "d, d_map:" << p.d << "," << p_map.d << std::endl;
		}*/
		//std::cout << p_map_normal_d.dot(p_normal_d) << std::endl;
		if((p_map.normal.dot(p.normal)>0.95 && fabs(p_map.d-p.d)<0.1) ||
			(p_map.normal.dot(p.normal)<-0.95 && fabs(p_map.d+p.d)<0.1))
		//if(fabs(p_map.normal.dot(n2))>0.95 && fabs(fabs(p_map.d)-fabs(p.d))<0.25) //0.97 = 14 degree; 0.95,0.15
		//if(angle<0.05 && fabs(p_map.d-p.d)<0.08) //planes can be merged
		{
		  /*if(p.normal(2)>0.9)
		  {
			std::cout << "map: " << p_map.d << "," << p_map.normal << std::endl;
			std::cout << "new feature: " << p.d << "," << p.normal << std::endl;
		  }*/
		  gpc_polygon gpc_result;
		  gpc_polygon gpc_p_merge;
		  gpc_polygon gpc_p_map;
		  std::cout << "Planes before merging:\n";
		  std::cout << "New f: " << p.normal(0) << "," << p.normal(1) << "," << p.normal(2) << "," << p.d << std::endl;
		  std::cout << "Map f: " << p_map.normal(0) << "," << p_map.normal(1) << "," << p_map.normal(2) << "," << p_map.d << std::endl;
		  double d_p = p.d;
		  Eigen::Vector3f normal_p(p.normal(0), p.normal(1), p.normal(2));
		  if(p_map.normal.dot(normal_p)<-0.95) {normal_p = -normal_p, d_p=-d_p;}
		  std::cout << "dot: " << p_map.normal.dot(normal_p) << std::endl;
		  Eigen::Vector3f n_map = p_map.normal + normal_p;
		  double d_map = p_map.d + d_p;
		  std::cout << "d_map before norm: " << d_map << "," << n_map.norm() << std::endl;
		  d_map = d_map/n_map.norm();
		  n_map.normalize();
		  Eigen::Vector3f ft_pt;
		  double x = -d_map/(n_map(0)+n_map(1)+n_map(2));
		  ft_pt << x,x,x;
		  /*if(fabs(n_map(2))>0.01)
			ft_pt << 0, 0, -d_map/n_map(2);
		  else if(fabs(n_map(0))>0.01)
			ft_pt << -d_map/n_map(0), 0, 0;
		  else if(fabs(n_map(1))>0.01)
			ft_pt << 0, -d_map/n_map(1), 0;*/
		  std::cout << "Merged plane:\n";
		  std::cout << "Merged f: " << n_map(0) << "," << n_map(1) << "," << n_map(2) << "," << d_map << std::endl;
		  std::cout << "FP: " << ft_pt(0) << "," << ft_pt(1) << "," << ft_pt(2) << std::endl;
		  Eigen::Affine3f transformation_from_plane_to_world;
		  Eigen::Affine3f transformation_from_world_to_plane;
		  getTransformationFromPlaneToWorld(n_map, ft_pt, transformation_from_plane_to_world);
		  //transformation_from_world_to_plane = transformation_from_plane_to_world.inverse();
		  transformation_from_world_to_plane = p_map.transform_from_world_to_plane;
		  //std::cout << "Transform new feature" << std::endl;
		  getGpcStructureUsingMap(p, transformation_from_world_to_plane/*p_map.transform_from_world_to_plane*/, &gpc_p_merge);
		  //std::cout <<  p.transform_from_world_to_plane.matrix() << std::endl;
		  //printGpcStructure(&gpc_p_merge);
		  //getGpcStructure(p, &gpc_p_merge);
		  //std::cout <<  p.transform_from_world_to_plane.matrix() << std::endl;
		  //printGpcStructure(&gpc_p_merge);
		  //std::cout << "Transform map" << std::endl;
		  getGpcStructureUsingMap(p_map, transformation_from_world_to_plane/*p_map.transform_from_world_to_plane*/, &gpc_p_map);
		  //printGpcStructure(&gpc_p_map);
		  gpc_polygon_clip(GPC_INT, &gpc_p_merge, &gpc_p_map, &gpc_result);
		  std::cout << "num contours intersect: " << gpc_result.num_contours << std::endl;
		  if(gpc_result.num_contours == 0)
		  {
			std::cout << "no intersection" << std::endl;
			std::cout << p.normal << std::endl;
			continue;
		  }
		  gpc_polygon_clip(GPC_UNION, &gpc_p_merge, &gpc_p_map, &gpc_result);
		  /*else if(gpc_result.num_contours >= gpc_p_map.num_contours + gpc_p_merge.num_contours)
		  {
			std::cout << "merge failed" << std::endl;
			std::cout << "feature id: " << i << std::endl;
			continue;
		  }*/
		  //else
		  {
			/*if(fabs(p_map.normal(1))>0.8)
			{
			  printMapEntry(p_map);
			  std::cout << "Trafo from plane to world: " << std::endl;
			  std::cout <<  p_map.transform_from_world_to_plane.inverse().matrix() << std::endl;
			}*/
			//gpc_result = gpc_p_map;
			/*p_map.transform_from_world_to_plane = transformation_from_world_to_plane;
			p_map.normal = n_map;
			p_map.d = d_map;*/
			p_map.polygon_world.resize(gpc_result.num_contours);
			/*for(int j=0; j<gpc_p_merge.num_contours; j++)
			{
			  p_map.polygon_world[j].resize(gpc_p_merge.contour[j].num_vertices);
			  for(int k=0; k<gpc_p_merge.contour[j].num_vertices; k++)
			  {
				//TODO: set z to something else?
				Eigen::Vector3f point(gpc_p_merge.contour[j].vertex[k].x, gpc_p_merge.contour[j].vertex[k].y, 0);
				p_map.polygon_world[j][k] = p_map.transform_from_world_to_plane.inverse()*point;
				//TODO: update normal, d, transformation...?
			  }
			}*/
			//printGpcStructure(&gpc_result);
			for(int j=0; j<gpc_result.num_contours; j++)
			{
			  p_map.polygon_world[j].resize(gpc_result.contour[j].num_vertices);
			  //std::cout << p_map.polygon_world[j].size() << std::endl;
			  for(int k=0; k<gpc_result.contour[j].num_vertices; k++)
			  {
				//TODO: set z to something else?
				Eigen::Vector3f point(gpc_result.contour[j].vertex[k].x, gpc_result.contour[j].vertex[k].y, 0);
				p_map.polygon_world[j][k] = p_map.transform_from_world_to_plane.inverse()*point;
				//TODO: update normal, d, transformation...?
			  }
			}
			std::cout << i << ": feature merged" << std::endl;
			/*if(fabs(p_map.normal(1))>0.8)
			{
			  printGpcStructure(&gpc_result);
			  printMapEntry(p_map);
			}*/
			merged = true;
			p_map.merged++;

			break;
		  }
		  gpc_free_polygon(&gpc_result);
		  gpc_free_polygon(&gpc_p_merge);
		  gpc_free_polygon(&gpc_p_map);



		}


  }
  //new entry
  if(!merged)
  {
    Eigen::Affine3f transformation_from_plane_to_world;
    getTransformationFromPlaneToWorld(p.normal, p.polygon_world[0][0], transformation_from_plane_to_world);
    p.transform_from_world_to_plane = transformation_from_plane_to_world.inverse();
    p.id = new_id_;
    map_.push_back(p_ptr);
    new_id_++;
    std::cout << "feature added" << std::endl;
    //ROS_INFO("added new feature");

  }
  //printGpcStructure(&gpc_p);
  //gpc_free_polygon(&gpc_p);
  if(save_to_file_) saveMap(file_path_);

}



void
GeometryMap::getGpcStructure(MapEntry& p, gpc_polygon* gpc_p)
{
  //printMapEntry(p);
  gpc_p->num_contours = p.polygon_world.size();
  gpc_p->hole = (int*)malloc(p.polygon_world.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(p.polygon_world.size()*sizeof(gpc_vertex_list));
  //std::cout << "num_contours: " << gpc_p->num_contours << std::endl;
  for(size_t j=0; j<p.polygon_world.size(); j++)
  {
    //std::cout << j << std::endl;
    gpc_p->contour[j].num_vertices = p.polygon_world[j].size();
    gpc_p->hole[j] = 0;
    //std::cout << "num_vertices: " << gpc_p->contour[j].num_vertices << std::endl;
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));
    for(size_t k=0; k<p.polygon_world[j].size(); k++)
    {
      //std::cout << p.polygon_world[j][k] << std::endl;
      Eigen::Vector3f point_trans = p.transform_from_world_to_plane*p.polygon_world[j][k];
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);
      if(fabs(point_trans(2))>0.01) std::cout << "z: " << point_trans(2) << std::endl;
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
GeometryMap::getGpcStructureUsingMap(MapEntry& p, Eigen::Affine3f& transform_from_world_to_plane, gpc_polygon* gpc_p)
{
  //Eigen::Affine3f transformation_from_plane_to_world;
  //getTransformationFromPlaneToWorld(p.normal, p.polygon_world[0][0], transformation_from_plane_to_world);
  //p.transform_from_world_to_plane = transform_from_world_to_plane;//transformation_from_plane_to_world.inverse();
  //printMapEntry(p);
  gpc_p->num_contours = p.polygon_world.size();
  gpc_p->hole = (int*)malloc(p.polygon_world.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(p.polygon_world.size()*sizeof(gpc_vertex_list));
  //std::cout << "num_contours: " << gpc_p->num_contours << std::endl;
  for(size_t j=0; j<p.polygon_world.size(); j++)
  {
    //std::cout << j << std::endl;
    gpc_p->contour[j].num_vertices = p.polygon_world[j].size();
    gpc_p->hole[j] = 0;
    //std::cout << "num_vertices: " << gpc_p->contour[j].num_vertices << std::endl;
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));
    for(size_t k=0; k<p.polygon_world[j].size(); k++)
    {
      //std::cout << p.polygon_world[j][k] << std::endl;
      Eigen::Vector3f point_trans = transform_from_world_to_plane*p.polygon_world[j][k];
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);
      //if(point_trans(2)>0.02 || point_trans(2)<-0.02) std::cout << "z: " << point_trans(2) << std::endl;
      //std::cout << k << ":" << gpc_p->contour[j].vertex[k].x << "," << gpc_p->contour[j].vertex[k].y <<std::endl;
    }
  }
}

void
GeometryMap::printMapEntry(MapEntry& p)
{
  std::cout << "Polygon:\n";
  for(int i=0; i< p.polygon_world.size(); i++)
  {
    std::cout << i << std::endl;
    for(int j=0; j< p.polygon_world[i].size(); j++)
    {
      std::cout << "(" << p.polygon_world[i][j](0) << ", " << p.polygon_world[i][j](1) << ", " << p.polygon_world[i][j](2) << ")\n";
    }
  }
  std::cout << "Normal: (" << p.normal(0) << ", " << p.normal(1) << ", " << p.normal(2) << ")\n";
  std::cout << "d: " << p.d << std::endl;
  std::cout << "Transformation:\n" << p.transform_from_world_to_plane.matrix() << "\n";
}

void
GeometryMap::saveMapEntry(std::string path, int ctr, MapEntry& p)
{
  std::stringstream ss;
  ss << path << "polygon_" << ctr << ".pl";
  std::ofstream plane_file;
  plane_file.open (ss.str().c_str());
  plane_file << p.normal(0) << " " << p.normal(1) << " " << p.normal(2) << " " << p.d;
  ss.str("");
  ss.clear();
  plane_file.close();
  for(int i=0; i< p.polygon_world.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> pc;
    ss << path << "polygon_" << ctr << "_" << i << ".pcd";
    for(int j=0; j< p.polygon_world[i].size(); j++)
    {
      pcl::PointXYZ pt;
      pt.x = p.polygon_world[i][j](0);
      pt.y = p.polygon_world[i][j](1);
      pt.z = p.polygon_world[i][j](2);
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
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal,  origin, transformation);
  transformation = transformation.inverse();
}

/*int main (int argc, char** argv)
 {

  GeometryMap fm;
  for(int j=0; j<46; j++)
  {
    GeometryMap::MapEntryPtr m_p = GeometryMap::MapEntryPtr(new GeometryMap::MapEntry());
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
        m_p->polygon_world.push_back(vv);
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
  MapEntryPtr m_p = MapEntryPtr(new MapEntry());

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
  m_p->polygon_world.push_back(vv);
  gm.addMapEntry(m_p);


  gmv.showPolygon(m_p,0);

  m_p = MapEntryPtr(new MapEntry());
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
  m_p->polygon_world.push_back(vv);
  gm.addMapEntry(m_p);

  /*m_p = MapEntryPtr(new MapEntry());
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
  m_p->polygon_world.push_back(vv);
  gm.addMapEntry(m_p);*/
  gm.saveMap("/home/goa/pcl_daten/merge_test/");
}



