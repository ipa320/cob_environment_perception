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
#include <queue>
#include <functional>


// external includes
#include <boost/timer.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <Eigen/Geometry>
#include <pcl/win32_macros.h>
//#include <pcl/common/transform.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
//#include <pcl/common/impl/transform.hpp>


#include <cob_3d_mapping_slam/dof/tflink.h>
#include "cob_3d_mapping_geometry_map/geometry_map.h"
using namespace cob_3d_mapping;


void
GeometryMap::addMapEntry(Polygon::Ptr& p_ptr)
{
  Polygon& p = *p_ptr;

  cob_3d_mapping::merge_config  limits;
  limits.d_thresh=d_;
  limits.angle_thresh=cos_angle_;
  //	limits.weighting_method="COMBINED";
  limits.weighting_method="COUNTER";
  p.merge_settings_ = limits;
  p.assignWeight();

  std::vector<int> intersections;
  if (map_polygon_.size()> 0)
  {
    p.getMergeCandidates(map_polygon_,intersections);
    if(intersections.size()>0) // if polygon has to be merged ...
    {
      std::vector<Polygon::Ptr> merge_candidates;
      for(int i=intersections.size()-1; i>=0 ;--i)
      {
        // copies pointer to polygon
        merge_candidates.push_back(map_polygon_[intersections[i]]);
        // delete pointer in map, polygon still available. However there should be a better solution than
        // copying and deleting pointers manually.
        map_polygon_[intersections[i]] = map_polygon_.back();
        map_polygon_.pop_back();
      }
      // merge polygon with merge candidates
      p.merge(merge_candidates); // merge all new candidates into p
      p.id = new_id_++;
      map_polygon_.push_back(p_ptr); // add p to map, candidates were dropped!
    }
    else //if polygon does not have to be merged , add new polygon
    {
      p.computeAttributes(p.normal,p.centroid);
      p.assignWeight();
      p.id = new_id_++;
      p.frame_stamp = frame_counter_;
      map_polygon_.push_back(p_ptr);
    }
  }
  else
  {
    p.computeAttributes(p.normal,p.centroid);
    p.assignWeight();
    p.id = new_id_++;
    p.frame_stamp = frame_counter_;
    map_polygon_.push_back(p_ptr);
  }
  if(save_to_file_) saveMap(file_path_);
}

void
GeometryMap::addMapEntry(Cylinder::Ptr& c_ptr)
{
  std::cout << "add cylinder" << std::endl;
  Cylinder& c = *c_ptr;
//

  cob_3d_mapping::merge_config  limits;
  limits.d_thresh=d_;
  limits.angle_thresh=cos_angle_;
  //limits.weighting_method="COUNTER";
  limits.weighting_method="AREA";

  c.merge_settings_ = limits;

  c.assignWeight();

  // find out polygons, to merge with
  std::vector<int> intersections;
  if (map_cylinder_.size()> 0 )
  {
    c.isMergeCandidate(map_cylinder_,limits,intersections);
    std::cout<<"intersections size = "<<intersections.size()<<std::endl;
    if (intersections.size() > 1) {
       std::cout<<"Intersection Size CYLINDER = "<<intersections.size()<<"\n";
     }

    // if polygon has to be merged ...
    if(intersections.size()>0)
    {
      std::vector<Cylinder::Ptr> merge_candidates;

      for(int i=intersections.size()-1; i>=0 ;--i)
      {

        merge_candidates.push_back(map_cylinder_[intersections[i]]);
        map_cylinder_[intersections[i]] = map_cylinder_.back();
        map_cylinder_.pop_back();


      }
      std::cout << merge_candidates.size() << std::endl;
      // merge polygon with merge candidates
//      c.debug_output("Pre");
      c.merge(merge_candidates);
      c.id = new_id_;
//      c.debug_output("Post");
      map_cylinder_.push_back(c_ptr);
      new_id_ ++;


      //	  std::cout<<"size +- "<< 1 -merge_candidates.size()<<std::endl;
    }
    //	}
    //if polygon does not have to be merged , add new polygon
    else
    {


      c.computeAttributes(c.sym_axis,c.normal,c.origin_);
      c.assignWeight();
      c.id = new_id_;
      c.frame_stamp =frame_counter_; 

      map_cylinder_.push_back(c_ptr);
      new_id_++;
      //	std::cout<<"size +1"<<std::endl;
    }
  }
  else{
    std::cout<<"ADD CYLINDER----\n";
    c.computeAttributes(c.sym_axis,c.normal,c.origin_);
    c.assignWeight();
    c.id = new_id_;
    c.frame_stamp =frame_counter_; 

    map_cylinder_.push_back(c_ptr);

    new_id_++;
  }
  std::cout<<"Map Size CYLINDER="<<map_cylinder_.size()<<std::endl;
  //	if(save_to_file_) saveMap(file_path_);
}

void
GeometryMap::addMapEntry(ShapeCluster::Ptr& sc_ptr)
{
  sc_ptr->computeAttributes();
  if (map_shape_cluster_.size())
  {
    std::vector<int> intersections;
    sc_ptr->getMergeCandidates(map_shape_cluster_, intersections);
    std::cout << intersections.size() << std::endl;
    if(intersections.size())
    {
      std::vector<ShapeCluster::Ptr> do_merge;
      for(int i=intersections.size()-1; i>=0; --i)
      {
        do_merge.push_back(map_shape_cluster_[intersections[i]]);
        map_shape_cluster_[intersections[i]] = map_shape_cluster_.back();
        map_shape_cluster_.pop_back();
      }
      sc_ptr->merge(do_merge);
      sc_ptr->id = new_id_++;
      map_shape_cluster_.push_back(sc_ptr);
    }
    else
    {
      sc_ptr->id = new_id_++;
      sc_ptr->frame_stamp = frame_counter_;
      map_shape_cluster_.push_back(sc_ptr);
    }
  }
  else
  {
    sc_ptr->id = new_id_++;
    sc_ptr->frame_stamp = frame_counter_;
    map_shape_cluster_.push_back(sc_ptr);
  }
}

bool
GeometryMap::computeTfError(const std::vector<Polygon::Ptr>& list_polygon, const Eigen::Affine3f& tf_old, Eigen::Affine3f& adjust_tf)
{
  return false;
  if (map_polygon_.size() < 10)
  {
    adjust_tf = Eigen::Affine3f::Identity();
    last_tf_err_ = Eigen::Affine3f::Identity();
    return false;
  }
  cob_3d_mapping::merge_config  limits;
  limits.d_thresh=d_;
  limits.angle_thresh=cos_angle_;
  limits.weighting_method="COUNTER";
  // min heap to store polygons with max overlap (Landmark elements: overlap, num_vertices, idx_old, idx_new)
  typedef boost::tuple<float,unsigned int,unsigned int> Landmark;
  std::priority_queue<Landmark> landmarks_queue;
  const size_t q_size = 3;
  int sum_overlap = 0;
  for (size_t p=0; p<map_polygon_.size(); ++p) // old polys
  {
    Polygon::Ptr pp = map_polygon_[p];

    for (size_t q=0; q<list_polygon.size(); ++q) // new polys
    {
      Polygon::Ptr pq = list_polygon[q];
      pq->merge_settings_=limits;
      if ( !pp->hasSimilarParametersWith(pq) ) continue;
      //if ( pq->contours[pq->outerContourIndex()].size() < 20 ) continue;
      //int abs_overlap;
      //float rel_overlap;
      //if (!pp->getContourOverlap(pq, rel_overlap, abs_overlap)) continue;
      //if (abs_overlap < 10) continue;
      //if (rel_overlap < 0.3) continue;
      //sum_overlap += abs_overlap;
      float w = pp->computeSimilarity(pq);
      std::cout << "Sim: " << w << std::endl;
      if (w < 0.70) continue;
      landmarks_queue.push( Landmark(w, p, q) );
    }
  }
  if (landmarks_queue.size() < q_size) return false;

  Landmark lm;
  DOF6::TFLinkvf tfe;
  Eigen::Vector3f n,m;
  float d1, d2;
  int i = 0;
  while (landmarks_queue.size() != 0)
  {
    lm = landmarks_queue.top(); landmarks_queue.pop();
    n = map_polygon_[lm.get<1>()]->normal;
    m = list_polygon[lm.get<2>()]->normal;
    d1 = map_polygon_[lm.get<1>()]->d;
    d2 = list_polygon[lm.get<2>()]->d;
    float weight = 2.0f/(fabs(d1)+fabs(d2));//(float)lm.get<0>();
    tfe(DOF6::TFLinkvf::TFLinkObj( d2 * m , true, false, weight),
        DOF6::TFLinkvf::TFLinkObj( d1 * n , true, false, weight));

    std::cout<<"%Overlap: "<<lm.get<0>()<<" Weigth: "<<weight<<std::endl;
    std::cout<<"%Area(old/new): "<<map_polygon_[lm.get<1>()]->computeArea3d()<<", "
             <<list_polygon[lm.get<2>()]->computeArea3d()<<std::endl;
    std::cout<<"vector_a"<<i<<" = ["<<n(0)<<","<<n(1)<<","<<n(2)<<"];"<<std::endl;
    std::cout<<"vector_b"<<i<<" = ["<<m(0)<<","<<m(1)<<","<<m(2)<<"];"<<std::endl;
    std::cout<<"origin_a"<<i<<" = "<<d1<<" * vector_a"<<i<<";"<<std::endl;
    std::cout<<"origin_b"<<i<<" = "<<d2<<" * vector_b"<<i<<";"<<std::endl;
    ++i;
  }

  tfe.finish();
  Eigen::Affine3f tf;
  tf.matrix().topLeftCorner<3,3>() = tfe.getRotation();
  tf.matrix().topRightCorner<3,1>() = tfe.getTranslation();
  tf.matrix().bottomLeftCorner<1,4>() << 0, 0, 0, 1;

  float roll, pitch, yaw;
  pcl::getEulerAngles(tf, roll, pitch, yaw);
  std::cout<<"Angles: r="<<roll*180.0f/M_PI<<" p="<<pitch*180.0f/M_PI<<" y="<<yaw*180.0f/M_PI<<std::endl;

  adjust_tf = tf;
  last_tf_err_ = adjust_tf;

  return true;
}


void
GeometryMap::cleanUp()
{
  int n_dropped = 0, m_dropped = 0, c_dropped=0;
  for(int idx = map_polygon_.size() - 1 ; idx >= 0; --idx)
  {
    //std::cout << map_polygon_[idx]->merged <<", " << (frame_counter_ - 3) <<" > "<<(int)map_polygon_[idx]->frame_stamp<<std::endl;
    if (map_polygon_[idx]->merged <= 1 && (frame_counter_ - 3) > (int)map_polygon_[idx]->frame_stamp)
    {
      map_polygon_[idx] = map_polygon_.back();
      map_polygon_.pop_back();
      ++n_dropped;
    }
  }
  for(int idx = map_cylinder_.size() - 1 ; idx >= 0; --idx)
  {
  bool drop_cyl=false;
      std::cout<<"merged:"<<(int)map_cylinder_[idx]->merged<<" frame ctr:"<<frame_counter_<<" frame st:"<<(int)map_cylinder_[idx]->frame_stamp<<" size:"<<(int)map_cylinder_[idx]->contours[0].size()<<"\n";
    if (map_cylinder_[idx]->merged <= 1 && (frame_counter_ - 2) > (int)map_cylinder_[idx]->frame_stamp)
        {
        drop_cyl=true;
        }
    if ((int)map_cylinder_[idx]->contours[0].size()<20 && (int)map_cylinder_[idx]->merged <= 1)
        {
         drop_cyl=true;
        }
    if ( drop_cyl==true)
    {        
      map_cylinder_[idx] = map_cylinder_.back();
      map_cylinder_.pop_back();
      ++c_dropped;
    }
  }
  for(int idx = map_shape_cluster_.size() - 1 ; idx >= 0; --idx)
  {
    std::cout << map_shape_cluster_[idx]->merged <<", " << (frame_counter_ - 3) <<" > "<<(int)map_shape_cluster_[idx]->frame_stamp<<std::endl;
    if (map_shape_cluster_[idx]->merged <= 1 && (frame_counter_ - 3) > (int)map_shape_cluster_[idx]->frame_stamp)
    {
      map_shape_cluster_[idx] = map_shape_cluster_.back();
      map_shape_cluster_.pop_back();
      ++m_dropped;
    }
  }
  std::cout << "Dropped " << n_dropped << " Polys, "<<c_dropped<<" Cyls, " << m_dropped << " Clusters" << std::endl;
  // TODO: clean up cylinders
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
      map_polygon_[i]->color[0] = 0.75;
      map_polygon_[i]->color[1] = 0.75;
      map_polygon_[i]->color[2] = 0;
      map_polygon_[i]->color[3] = 0.8;
    }
    else if(fabs(map_polygon_[i]->normal[0]) < 0.12 && fabs(map_polygon_[i]->normal[1]) < 0.12 && fabs(map_polygon_[i]->normal[2]) > 0.9) //plane is horizontal
    {
      map_polygon_[i]->color[0] = 0;
      map_polygon_[i]->color[1] = 0.5;
      map_polygon_[i]->color[2] = 0;
      map_polygon_[i]->color[3] = 0.8;
    }
    else
    {
      map_polygon_[i]->color[0] = 0.75;
      map_polygon_[i]->color[1] = 0.75;
      map_polygon_[i]->color[2] = 0.75;
      map_polygon_[i]->color[3] = 0.8;
    }
  }

  //coloring for cylinder
  for(unsigned int i=0; i<map_cylinder_.size(); i++)
  {
    if(fabs(map_cylinder_[i]->normal[0]) < 0.1 && fabs(map_cylinder_[i]->normal[1]) < 0.1) //cylinder is vertical
    {
      map_cylinder_[i]->color[0] = 0.5;
      map_cylinder_[i]->color[1] = 0.5;
      map_cylinder_[i]->color[2] = 0;
      map_cylinder_[i]->color[3] = 1;
    }
    else if(fabs(map_cylinder_[i]->normal[2]) < 0.12) //plane is horizontal
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
  Polygon::Ptr m_p1 = Polygon::Ptr(new Polygon());
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
  Polygon::Ptr m_p2 = Polygon::Ptr(new Polygon());
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
