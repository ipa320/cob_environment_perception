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

#include "cob_env_model/map/feature_map.h"



void
FeatureMap::addMapEntry(FeatureMap::MapEntryPtr p_ptr)
{
  FeatureMap::MapEntry& p = *p_ptr;
  //ROS_INFO("polygonCallback");
  bool merged = false;
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
    Eigen::Vector3f n2(p.normal(0), p.normal(1), p.normal(2));
    n2.normalize();
    //double angle = fabs(std::acos(p_map.normal.dot(n2)));
    if(fabs(p_map.normal.dot(n2))>0.97 && fabs(fabs(p_map.d)-fabs(p.d))<0.01) //0.97 = 14 degree
    //if(angle<0.05 && fabs(p_map.d-p.d)<0.08) //planes can be merged
    {
      gpc_polygon gpc_result;
      gpc_polygon gpc_p_merge;
      gpc_polygon gpc_p_map;
      getGpcStructureUsingMap(p, p_map.transform_from_world_to_plane, &gpc_p_merge);
      //std::cout <<  p.transform_from_world_to_plane.matrix() << std::endl;
      //printGpcStructure(&gpc_p_merge);
      //getGpcStructure(p, &gpc_p_merge);
      //std::cout <<  p.transform_from_world_to_plane.matrix() << std::endl;
      //printGpcStructure(&gpc_p_merge);
      getGpcStructure(p_map, &gpc_p_map);
      //printGpcStructure(&gpc_p_map);
      gpc_polygon_clip(GPC_UNION, &gpc_p_merge, &gpc_p_map, &gpc_result);
      if(gpc_result.num_contours == 0) //merge failed
        continue;
      else
      {
        /*if(fabs(p_map.normal(1))>0.8)
        {
          printMapEntry(p_map);
          std::cout << "Trafo from plane to world: " << std::endl;
          std::cout <<  p_map.transform_from_world_to_plane.inverse().matrix() << std::endl;
        }*/
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
        std::cout << "feature merged" << std::endl;
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
  //saveMap("/home/goa/pcl_daten/kitchen_kinect/map");

}


void
FeatureMap::getGpcStructure(FeatureMap::MapEntry& p, gpc_polygon* gpc_p)
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
FeatureMap::getGpcStructureUsingMap(FeatureMap::MapEntry& p, Eigen::Affine3f& transform_from_world_to_plane, gpc_polygon* gpc_p)
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
      //std::cout << k << ":" << gpc_p->contour[j].vertex[k].x << "," << gpc_p->contour[j].vertex[k].y <<std::endl;
    }
  }
}

void
FeatureMap::printMapEntry(FeatureMap::MapEntry& p)
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
FeatureMap::saveMapEntry(std::string path, int ctr, FeatureMap::MapEntry& p)
{
  std::stringstream ss;
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
    pcl::io::savePCDFileASCII (ss.str(), pc);
    ss.str("");
    ss.clear();
  }
}

void
FeatureMap::saveMap(std::string path)
{
  static int ctr=0;
  std::stringstream ss;
  ss << path << "/" << ctr << "_";
  std::cout << "Saving map with " << map_.size() << " entries..." << std::endl;
  for(size_t i=0; i< map_.size(); i++)
  {
    if (map_[i]->merged >0)
      saveMapEntry(ss.str(), i, *map_[i]);
  }
  ctr++;
}

void
FeatureMap::printGpcStructure(gpc_polygon* p)
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
FeatureMap::getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
                                            Eigen::Vector3f &u, Eigen::Vector3f &v)
{
  v = normal.unitOrthogonal ();
  u = normal.cross (v);
}

void
FeatureMap::getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,
                                                   const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
{
  Eigen::Vector3f u, v;
  getCoordinateSystemOnPlane(normal, u, v);
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal,  origin, transformation);
  transformation = transformation.inverse();
}

int main (int argc, char** argv)
 {

  FeatureMap fm;
  for(int j=0; j<46; j++)
  {
    FeatureMap::MapEntryPtr m_p = FeatureMap::MapEntryPtr(new FeatureMap::MapEntry());
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
}



