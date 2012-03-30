/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 10/2011
 * ToDo:
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

#ifndef __IMPL_EXTENDED_SEGMENTATION_H__
#define __IMPL_EXTENDED_SEGMENTATION_H__

#include <deque>
#include <math.h>

#include <pcl/common/eigen.h>

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_mapping_features/extended_segmentation.h"

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::propagateWavefront(
  const LabelCloudPtr& labels,
  ClusterList& cluster_out)
{
  int curr_label = I_EDGE;
  int width = labels->width, height = labels->height;
  //const float angle_threshold = M_PI / 9.0; // max angle difference between mean angle of cluster
  const float angle_threshold = 0.8; // everything above belongs to cluster
  const float curvature_threshold = 20; // everything above belongs to cluster
  const int min_cluster_size = 20;
  cluster_out.clear();
  cluster_out.resize(3);
  cluster_out[0].type = I_NAN;
  cluster_out[1].type = I_BORDER;
  cluster_out[2].type = I_EDGE;

  for(size_t i=0; i<labels->points.size(); ++i)
  {
    switch(labels->points[i].label)
    {
    case I_NAN:
      cluster_out[0].indices.push_back(i);
      break;

    case I_BORDER:
      cluster_out[1].indices.push_back(i);
      break;

    case I_EDGE:
      cluster_out[2].indices.push_back(i);
      break;

    case I_UNDEF:
    {
      ++curr_label;
      cob_3d_mapping_common::Cluster new_cluster;
      std::deque<Coords> coords_todo;
      coords_todo.push_back(Coords(i%width, i/width, 0));
      labels->points[i].label = curr_label;
      while (coords_todo.size() != 0)
      {
	Coords p = coords_todo.front();
	float cos_angle_norm, cos_angle_dist, angle;
	Eigen::Vector3f p_i;
	coords_todo.pop_front();
	int p_idx = p.v * width + p.u;
	new_cluster.updateCluster(
	  p_idx, surface_->points[p_idx].getVector3fMap(),
	  Eigen::Vector3f(normals_->points[p_idx].normal));
	//bool is_highly_curved = (std::abs(new_cluster.getMaxCurvature()) > curvature_threshold);
	
	Eigen::Vector3f c = new_cluster.getCentroid();
	float dist_threshold = 8.0 * 0.003 * c(2) * c(2);
	// Look right
	if (p.u+1 < width && labels->points[p_idx+1].label == I_UNDEF)
	{
	  /* Cluster is still to small to have convincing mean values
	   * --- or ---
	   * Cluster has high curvature and most likely being some sort of more complex object,
	   * so add to cluster and ignore normal comparision
	   * --- or ---
	   * Cluster has low mean curvature and is most likely a plane
	   * so check if the orientation of the new point is similar to the cluster
	   */
	  cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx+1].normal));
	  //p_i = surface_->points[p_idx+1].getVector3fMap() - c;
	  //cos_angle_dist = new_cluster.getOrientation().dot(p_i.normalized());
	  //angle = std::sin( ( cos_angle_dist < 0 ? std::acos(cos_angle_dist) - M_PI_2 : M_PI_2 - std::acos(cos_angle_dist) ) );
	  if ( (int)new_cluster.indices.size() < min_cluster_size || 
	       cos_angle_norm > angle_threshold )// && std::fabs(p_i.norm() * angle) < dist_threshold) )
	  {
	    labels->points[p_idx+1].label = curr_label;
	    coords_todo.push_back(Coords(p.u+1, p.v, 0.0));
	  }
	}
	// Look down
	if (p.v+1 < height  && labels->points[p_idx+width].label == I_UNDEF)
	{
	  cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx+width].normal));
	  //p_i = surface_->points[p_idx+width].getVector3fMap() - c;
	  //cos_angle_dist = new_cluster.getOrientation().dot(p_i.normalized());
	  //angle = std::sin( ( cos_angle_dist < 0 ? std::acos(cos_angle_dist) - M_PI_2 : M_PI_2 - std::acos(cos_angle_dist) ) );
	  if ( (int)new_cluster.indices.size() < min_cluster_size || 
	       cos_angle_norm > angle_threshold )//&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	  {
	    labels->points[p_idx+width].label = curr_label;
	    coords_todo.push_back(Coords(p.u, p.v+1, 0.0));
	  }
	}
	// Look left
	if (p.u > 0 && labels->points[p_idx-1].label == I_UNDEF)
	{
	  cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx-1].normal));
	  //p_i = surface_->points[p_idx-1].getVector3fMap() - c;
	  //cos_angle_dist = new_cluster.getOrientation().dot(p_i.normalized());
	  //angle = std::sin( ( cos_angle_dist < 0 ? std::acos(cos_angle_dist) - M_PI_2 : M_PI_2 - std::acos(cos_angle_dist) ) );
	  if ( (int)new_cluster.indices.size() < min_cluster_size || 
	       cos_angle_norm > angle_threshold )//&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	  {
	    labels->points[p_idx-1].label = curr_label;
	    coords_todo.push_back(Coords(p.u-1, p.v, 0.0));
	  }	 
	}
	// Look up
	if (p.v > 0 && labels->points[p_idx-width].label == I_UNDEF)
	{
	  cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx-width].normal));
	  //p_i = surface_->points[p_idx-width].getVector3fMap() - c;
	  //cos_angle_dist = new_cluster.getOrientation().dot(p_i.normalized());
	  //angle = std::sin( ( cos_angle_dist < 0 ? std::acos(cos_angle_dist) - M_PI_2 : M_PI_2 - std::acos(cos_angle_dist) ) );
	  if ( (int)new_cluster.indices.size() < min_cluster_size || 
	       cos_angle_norm > angle_threshold )//&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	  {
	    labels->points[p_idx-width].label = curr_label;
	    coords_todo.push_back(Coords(p.u, p.v-1, 0.0));
	  }
	}
      }

      cluster_out.push_back(new_cluster);
      break;
    }
    default:
      break;
    }
  }
  return;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::analyseClusters(
  ClusterList& cluster_list)
{
  for (ClusterList::iterator c = cluster_list.begin(); c != cluster_list.end(); ++c)
  {
    if (c->indices.size() < 100) continue;
    if (c->type == I_EDGE || c->type == I_NAN || c->type == I_BORDER) continue;

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    Eigen::Vector3f centroid = c->getCentroid();
    for (std::vector<int>::iterator idx = c->indices.begin(); idx != c->indices.end(); ++idx)
    {
      Eigen::Vector3f demean = surface_->points[*idx].getVector3fMap() - centroid;
      cov += demean * demean.transpose();
    }
    Eigen::Matrix3f eigenvectors;
    pcl::eigen33(cov, eigenvectors, c->eigenvalues);
    c->first_component = eigenvectors.col(2);
    c->second_component = eigenvectors.col(1);
    c->third_component = eigenvectors.col(0);
    c->eigenvalues /= c->indices.size();
    if (c->getSurfaceCurvature() < 0.001 * centroid(2) * centroid(2))
      c->type = I_PLANE;
    std::cout << "Z: " << centroid(2) << "\tSize: " << c->indices.size() << "\t(1)| " 
	      << c->eigenvalues(2) << "\t(2)| " << c->eigenvalues(1) << "\t(3)| " << c->eigenvalues(0)
	      << "\tC: " << c->getSurfaceCurvature() << std::endl;
    
  }
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::getColoredCloud(
  ClusterList& cluster_list,
  pcl::PointCloud<PointXYZRGB>::Ptr& color_cloud)
{
  std::sort(cluster_list.begin()+3, cluster_list.end());
  uint32_t rgb;
  int t = 4;
  for(int c = cluster_list.size()-1; c>2; --c, ++t)
  {
    /*if (cluster_list[c].indices.size() > 10)
      std::cout << "Size: " << cluster_list[c].indices.size() 
		<< " | MaxCurv: " << cluster_list[c].getMaxCurvature() 
		<< " | MinCurv: " << cluster_list[c].getMinCurvature() << std::endl;*/
    rgb = (color_tab_[t])[2] << 16 | (color_tab_[t])[1] << 8 | (color_tab_[t])[0];
    for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
    {
      color_cloud->points[cluster_list[c].indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
    }
  }
  //Edge
  rgb = (color_tab_[3])[2] << 16 | (color_tab_[3])[1] << 8 | (color_tab_[3])[0];
  for(size_t i = 0; i<cluster_list[2].indices.size(); ++i)
    color_cloud->points[cluster_list[2].indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
  //border
  rgb = (color_tab_[2])[2] << 16 | (color_tab_[2])[1] << 8 | (color_tab_[2])[0];
  for(size_t i = 0; i<cluster_list[1].indices.size(); ++i)
    color_cloud->points[cluster_list[1].indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
  rgb = (color_tab_[1])[2] << 16 | (color_tab_[1])[1] << 8 | (color_tab_[1])[0];
  //nan
  for(size_t i = 0; i<cluster_list[0].indices.size(); ++i)
    color_cloud->points[cluster_list[0].indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
}


template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::getColoredCloudByType(
  ClusterList& cluster_list,
  pcl::PointCloud<PointXYZRGB>::Ptr& color_cloud)
{
  for(int c = 0; c<(int)cluster_list.size(); ++c)
  {
    switch(cluster_list[c].type)
    {
    case I_NAN:
      break;
    case I_BORDER:
      for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
      {
	color_cloud->points[cluster_list[c].indices[i]].r = 255;
	color_cloud->points[cluster_list[c].indices[i]].g = 255;
	color_cloud->points[cluster_list[c].indices[i]].b = 255;
      }
      break;
    case I_EDGE:
      for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
      {
	color_cloud->points[cluster_list[c].indices[i]].r = 255;
	color_cloud->points[cluster_list[c].indices[i]].g = 0;
	color_cloud->points[cluster_list[c].indices[i]].b = 0;
      }
      break;
    case I_PLANE:
      for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
      {
	color_cloud->points[cluster_list[c].indices[i]].r = 0;
	color_cloud->points[cluster_list[c].indices[i]].g = 200;
	color_cloud->points[cluster_list[c].indices[i]].b = 255;
      }
      break;
    default:
      if (cluster_list[c].isConvex())
      {
	for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
	{
	  color_cloud->points[cluster_list[c].indices[i]].r = 200;
	  color_cloud->points[cluster_list[c].indices[i]].g = 0;
	  color_cloud->points[cluster_list[c].indices[i]].b = 0;
	}
      }
      else
      {
	for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
	{
	  color_cloud->points[cluster_list[c].indices[i]].r = 50;
	  color_cloud->points[cluster_list[c].indices[i]].g = 50;
	  color_cloud->points[cluster_list[c].indices[i]].b = 255;
	}
      }
      break;
    }
  }
}


#define PCL_INSTANTIATE_ExtendedSegmentation(T,NT,CT,LabelT) template class PCL_EXPORTS cob_3d_mapping_features::ExtendedSegmentation<T,NT,CT,LabelT>;


#endif // __IMPL_EXTENDED_SEGMENTATION_H__
