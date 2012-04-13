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
#include <queue>
#include <list>
#include <math.h>

#include <pcl/common/eigen.h>
#include <Eigen/LU>

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_mapping_features/extended_segmentation.h"
#include "cob_3d_mapping_features/organized_normal_estimation.h"

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::propagateWavefront(
  ClusterList& cluster_out)
{
  int curr_label = I_EDGE;
  int width = labels_->width, height = labels_->height;
  //const float angle_threshold = M_PI / 9.0; // max angle difference between mean angle of cluster
  const float angle_threshold = 0.8; // everything above belongs to cluster
  const float curvature_threshold = 20; // everything above belongs to cluster
  const int min_cluster_size = 20;
  cluster_out.clear();
  cluster_out.resize(3);
  cluster_out[0].type = I_NAN;
  cluster_out[1].type = I_BORDER;
  cluster_out[2].type = I_EDGE;

  for(size_t i=0; i<labels_->points.size(); ++i)
  {
    switch(labels_->points[i].label)
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
      std::list<ClusterElement> coords_todo;
      coords_todo.push_back(ClusterElement(i%width, i/width, 0.0));
      labels_->points[i].label = curr_label;
      while (coords_todo.size() != 0)
      {
	ClusterElement p = coords_todo.front();
	coords_todo.pop_front();
	float cos_angle_norm, cos_angle_dist, angle, plane_distance;
	Eigen::Vector3f p_i;
	int p_idx = p.v * width + p.u;
	new_cluster.updateCluster(
	  p_idx, surface_->points[p_idx].getVector3fMap(),
	  Eigen::Vector3f(normals_->points[p_idx].normal));
	//bool is_highly_curved = (std::abs(new_cluster.getMaxCurvature()) > curvature_threshold);
	
	Eigen::Vector3f c = new_cluster.getCentroid();
	//float dist_threshold = 20.0 * 0.003 * surface_->points[p_idx].z *surface_->points[p_idx].z;
	// Look right
	if (p.u+1 < width && labels_->points[p_idx+1].label == I_UNDEF)
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
	  //plane_distance = surface_->points[p_idx+1].getVector3fMap().dot(normals_->points[p_idx+1].getNormalVector3fMap());
	  if ( (int)new_cluster.indices.size() < min_cluster_size || 
	       //(fabs(plane_distance - plane_distance_cluster) < dist_threshold &&
		cos_angle_norm > angle_threshold)// && std::fabs(p_i.norm() * angle) < dist_threshold) )
	  {
	    labels_->points[p_idx+1].label = curr_label;
	    coords_todo.push_back(ClusterElement(p.u+1, p.v, cos_angle_norm));
	  }
	}
	// Look down
	if (p.v+1 < height  && labels_->points[p_idx+width].label == I_UNDEF)
	{
	  cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx+width].normal));
	  //plane_distance = surface_->points[p_idx+width].getVector3fMap().dot(normals_->points[p_idx+width].getNormalVector3fMap());
	  if ( (int)new_cluster.indices.size() < min_cluster_size || 
	       //(fabs(plane_distance - plane_distance_cluster) < dist_threshold &&
		cos_angle_norm > angle_threshold)//&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	  {
	    labels_->points[p_idx+width].label = curr_label;
	    coords_todo.push_back(ClusterElement(p.u, p.v+1, cos_angle_norm));
	  }
	}
	// Look left
	if (p.u > 0 && labels_->points[p_idx-1].label == I_UNDEF)
	{
	  cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx-1].normal));
	  //plane_distance = surface_->points[p_idx-1].getVector3fMap().dot(normals_->points[p_idx-1].getNormalVector3fMap());
	  if ( (int)new_cluster.indices.size() < min_cluster_size || 
	       //(fabs(plane_distance - plane_distance_cluster) < dist_threshold &&
		cos_angle_norm > angle_threshold) //&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	  {
	    labels_->points[p_idx-1].label = curr_label;
	    coords_todo.push_back(ClusterElement(p.u-1, p.v, cos_angle_norm));
	  }	 
	}
	// Look up
	if (p.v > 0 && labels_->points[p_idx-width].label == I_UNDEF)
	{
	  cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx-width].normal));
	  //plane_distance = surface_->points[p_idx-width].getVector3fMap().dot(normals_->points[p_idx-width].getNormalVector3fMap());
	  if ( (int)new_cluster.indices.size() < min_cluster_size || 
	       //(fabs(plane_distance - plane_distance_cluster) < dist_threshold &&
		cos_angle_norm > angle_threshold)//&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	  {
	    labels_->points[p_idx-width].label = curr_label;
	    coords_todo.push_back(ClusterElement(p.u, p.v-1, cos_angle_norm));
	  }
	}
	//coords_todo.sort();
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
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::propagateWavefront2ndPass(
  ClusterList& cluster_list)
{
  // ---- Find potential planes and drop all other clusters -----
  std::vector<int> indices_todo;
  ClusterList::iterator c = cluster_list.begin();
  int curr_label = I_EDGE;
  while (c != cluster_list.end())
  {
    //std::cout << cluster_list.size() << std::endl;    
    if (c->type == I_EDGE || c->type == I_NAN || c->type == I_BORDER) { ++c; continue; }
    if (c->indices.size() > 100) computeClusterComponents(*c);
    curr_label = std::max(curr_label, labels_->points[c->indices[0]].label);
    // set labels_ of cluster to UNDEF for 2nd-Pass segmentation and delete cluster from cluster_list
    for (std::vector<int>::iterator idx = c->indices.begin(); idx != c->indices.end(); ++idx)
    {
      labels_->points[*idx].label = I_UNDEF;
      indices_todo.push_back(*idx);
    }
    c = cluster_list.erase(c);
  }

  // ----- Try a different segmentation method on all previously dropped clusters -----
  const int min_cluster_size = 20;
  const std::size_t buffer_size = 150;
  const float angle_threshold = 0.9; // everything above belongs to cluster
  float cos_angle_norm = 0.0;
  int width = labels_->width, height = labels_->height;
  for (std::vector<int>::iterator idx = indices_todo.begin(); idx != indices_todo.end(); ++idx)
  {
    //std::cout << (*idx)%width <<"/"<< (*idx)/width << std::endl;
    if (labels_->points[*idx].label != I_UNDEF) continue;

    ++curr_label;
    cob_3d_mapping_common::Cluster new_cluster;
    std::list<ClusterElement> coords_todo;
    coords_todo.push_back(ClusterElement((*idx)%width, (*idx)/width, 0.0));
    std::queue<Eigen::Vector3f> normal_buffer;
    labels_->points[*idx].label = curr_label;
    while (coords_todo.size() != 0)
    {
      ClusterElement p = coords_todo.front();
      coords_todo.pop_front();
      int p_idx = p.v * width + p.u;
      new_cluster.updateCluster(p_idx,surface_->points[p_idx].getVector3fMap(),Eigen::Vector3f(normals_->points[p_idx].normal));
      normal_buffer.push(Eigen::Vector3f(normals_->points[p_idx].normal));
      //if (normal_buffer.size() > min_cluster_size && normal_buffer.size() > 0.3 * new_cluster.indices.size())
      if (normal_buffer.size() > buffer_size)
      {
	new_cluster.sum_orientation -= normal_buffer.front();
	normal_buffer.pop();
      }
 
      // Look right
      if (p.u+1 < width && labels_->points[p_idx+1].label == I_UNDEF)
      {
	cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx+1].normal));
	if (new_cluster.indices.size() < min_cluster_size || cos_angle_norm > angle_threshold)
	{
	  labels_->points[p_idx+1].label = curr_label;
	  coords_todo.push_back(ClusterElement(p.u+1, p.v, cos_angle_norm));
	}
      }
      // Look down
      if (p.v+1 < height  && labels_->points[p_idx+width].label == I_UNDEF)
      {
	cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx+width].normal));
	if (new_cluster.indices.size() < min_cluster_size || cos_angle_norm > angle_threshold)
	{
	  labels_->points[p_idx+width].label = curr_label;
	  coords_todo.push_back(ClusterElement(p.u, p.v+1, cos_angle_norm));
	}
      }
      // Look left
      if (p.u > 0 && labels_->points[p_idx-1].label == I_UNDEF)
      {
	cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx-1].normal));
	if (new_cluster.indices.size() < min_cluster_size || cos_angle_norm > angle_threshold)
	{
	  labels_->points[p_idx-1].label = curr_label;
	  coords_todo.push_back(ClusterElement(p.u-1, p.v, cos_angle_norm));
	}	 
      }
      // Look up
      if (p.v > 0 && labels_->points[p_idx-width].label == I_UNDEF)
      {
	cos_angle_norm = new_cluster.getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx-width].normal));
	if (new_cluster.indices.size() < min_cluster_size || cos_angle_norm > angle_threshold)
	{
	  labels_->points[p_idx-width].label = curr_label;
	  coords_todo.push_back(ClusterElement(p.u, p.v-1, cos_angle_norm));
	}
      }
      //coords_todo.sort();
    }

    cluster_list.push_back(new_cluster);
  }
}


template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> bool
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::hasValidCurvature(int idx)
{
  Eigen::Vector3f n_idx(normals_->points[idx].normal);
  Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - n_idx * n_idx.transpose(); // projection matrix
  std::vector<Eigen::Vector3f> normals_projected;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  int s = 4; // neighborhood size
  for (int y = idx - s * surface_->width; y <= idx + s * surface_->width; y+=surface_->width)
  {
    for (int x = -s; x <= s; ++x)
    {
      PointNT const* n_i = &(normals_->points[y+x]);
      if ( pcl_isnan(n_i->normal[2]) ) continue;
      normals_projected.push_back( M * Eigen::Vector3f(n_i->normal) );
      centroid += normals_projected.back();
    }
  }
  float num_p_inv = 1.0f / normals_projected.size();
  centroid *= num_p_inv;
  //if (centroid.norm() > 0.05) return true; // if normals are asymetric
  
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (std::vector<Eigen::Vector3f>::iterator n_it = normals_projected.begin(); n_it != normals_projected.end(); ++n_it)
  {
    Eigen::Vector3f demean = *n_it - centroid;
    cov += demean * demean.transpose();
  }
  Eigen::Vector3f eigenvalues;
  Eigen::Matrix3f eigenvectors;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  std::cout << "C: " << (1.0 / centroid.norm()) * eigenvalues(2)*num_p_inv << "\tNorm: " << centroid.norm() <<std::endl;
  if ( (1.0 / centroid.norm()) * eigenvalues(2)*num_p_inv < 0.8) return true;
  return false;
}


template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> bool
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeClusterComponents(
  cob_3d_mapping_common::Cluster& c)
{
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  Eigen::Vector3f centroid = c.getCentroid();
  for (std::vector<int>::iterator idx = c.indices.begin(); idx != c.indices.end(); ++idx)
  {
    Eigen::Vector3f demean = surface_->points[*idx].getVector3fMap() - centroid;
    cov += demean * demean.transpose();
  }
  Eigen::Matrix3f eigenvectors;
  Eigen::Vector3f eigenvalues;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  eigenvalues /= c.indices.size();
  c.first_component = eigenvectors.col(2);
  c.second_component = eigenvectors.col(1);
  c.third_component = eigenvectors.col(0);
  c.eigenvalues = eigenvalues;
  if ( eigenvalues(0) / (eigenvalues(0)+eigenvalues(1)+eigenvalues(2)) < 0.001 * centroid(2) * centroid(2) )
  {
    c.type = I_PLANE;
    c.is_save_plane = true;
  }
  return c.is_save_plane;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeClusterCurvature(
  cob_3d_mapping_common::Cluster& c, int search_size)
{
  //float mean_curvature_max=0.0, mean_curvature_min=0.0;
  std::vector<int> geometry(NUM_LABELS, 0);
  int y_offset = search_size * surface_->width;
  int cluster_label = labels_->points[c.indices.front()].label;
  for (std::vector<int>::iterator idx=c.indices.begin(); idx != c.indices.end(); ++idx)
  {
    Eigen::Vector3f n_idx(normals_->points[*idx].normal);
    if (pcl_isnan(n_idx(2))) continue;
    Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - n_idx * n_idx.transpose();
    std::vector<Eigen::Vector3f> normals_projected;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int y = *idx - y_offset; y <= *idx + y_offset; y+=surface_->width)
    {
      for (int x = -search_size; x <= search_size; ++x)
      {
	if( labels_->points[y+x].label != cluster_label ) continue;
	PointNT const* n_i = &(normals_->points[y+x]);
	if ( pcl_isnan(n_i->normal[2]) ) continue;
	normals_projected.push_back( M * Eigen::Vector3f(n_i->normal) );
	centroid += normals_projected.back();
      }
    }
    float num_p_inv = 1.0f / normals_projected.size();
    centroid *= num_p_inv;
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (std::vector<Eigen::Vector3f>::iterator n_it = normals_projected.begin(); n_it != normals_projected.end(); ++n_it)
    {
      Eigen::Vector3f demean = *n_it - centroid;
      cov += demean * demean.transpose();
    }
    Eigen::Vector3f eigenvalues;
    Eigen::Matrix3f eigenvectors;
    pcl::eigen33(cov, eigenvectors, eigenvalues);
    eigenvalues(2) *= num_p_inv;
    eigenvalues(1) *= num_p_inv;
    if (eigenvalues(2) < 0.01) ++geometry[I_PLANE];
    else if(eigenvalues(2) < 0.11)
    {
      if(eigenvalues(2) < 7.0 * eigenvalues(1)) ++geometry[I_SPHERE];
      else ++geometry[I_CYL];
    }
    else ++geometry[I_UNDEF];
  }
  int max = 0; size_t max_idx = 0;
  for (size_t i=0; i<geometry.size(); ++i)
    if ( (max=std::max(max,geometry[i])) == geometry[i] ) max_idx=i;
  c.type = max_idx;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::analyseClusters(
  ClusterList& cluster_list)
{
  OrganizedNormalEstimation<PointT,PointNT,PointOutT> one;
  for (ClusterList::iterator c = cluster_list.begin(); c != cluster_list.end(); ++c)
  {
    if (c->indices.size() < 100) continue;
    
    if (c->type == I_EDGE || c->type == I_NAN || c->type == I_BORDER || computeClusterComponents(*c)) continue;

    // recompute normals
    /*
    int w_size = std::floor(sqrt(c->indices.size() / 16.0f))+8;
    int steps = std::floor(w_size / 3);
    one.setPixelSearchRadius(w_size,2,steps);
    one.computeMaskManually(surface_->width);
    c->sum_orientation = Eigen::Vector3f::Zero();
    for (std::vector<int>::iterator idx = c->indices.begin(); idx != c->indices.end(); ++ idx)
    {
      one.recomputeSegmentNormal(surface_, labels_, *idx,
				 normals_->points[*idx].normal[0],
				 normals_->points[*idx].normal[1],
				 normals_->points[*idx].normal[2]);
      if (!pcl_isnan(normals_->points[*idx].normal[2]))
	c->sum_orientation += Eigen::Vector3f(normals_->points[*idx].normal);
    }
    */
    // compute principal curvature of the segment in its entirety
    Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - c->getOrientation() * c->getOrientation().transpose();
    std::vector<Eigen::Vector3f> normals_projected;
    Eigen::Vector3f n_centroid = Eigen::Vector3f::Zero();
    for (std::vector<int>::iterator idx = c->indices.begin(); idx != c->indices.end(); ++ idx)
    {
      if (pcl_isnan(normals_->points[*idx].normal[2])) continue;
      normals_projected.push_back( M * Eigen::Vector3f(normals_->points[*idx].normal) );
      n_centroid += normals_projected.back();
    }
    float num_p_inv = 1.0f / normals_projected.size();
    n_centroid *= num_p_inv;
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (std::vector<Eigen::Vector3f>::iterator n_it = normals_projected.begin(); n_it != normals_projected.end(); ++n_it)
    {
      Eigen::Vector3f demean = *n_it - n_centroid;
      cov += demean * demean.transpose();
    }
    Eigen::Vector3f eigenvalues;
    Eigen::Matrix3f eigenvectors;
    pcl::eigen33(cov, eigenvectors, eigenvalues);
    eigenvalues(2) *= num_p_inv;
    eigenvalues(1) *= num_p_inv;
    //std::cout <<"2ndPass: size = "<< c->indices.size() <<"\tc_max = "<< eigenvalues(2) <<"\tc_min = "<< eigenvalues(1) << std::endl;

    if (eigenvalues(2) < 0.02) c->type = I_PLANE;
    else if(eigenvalues(2) < 7.0 * eigenvalues(1)) c->type = I_SPHERE;
    else c->type = I_CYL;
  }
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::calcNormalIntersections(
  ClusterList& cluster_list,
  pcl::PointCloud<PointXYZ>::Ptr& intersection_points)
{
  const std::size_t idx_steps = 1;
  const float rand_max_inv = 1.0f / RAND_MAX;

  for (ClusterList::iterator c = cluster_list.begin(); c != cluster_list.end(); ++c)
  {
    if (c->indices.size() < 100) continue;
    if (c->type == I_EDGE || c->type == I_NAN || c->type == I_BORDER || c->is_save_plane) continue;
    std::size_t front = 0, back = c->indices.size()-1;
    Eigen::Vector3f p_mean = Eigen::Vector3f::Zero();
    std::vector<Eigen::Vector3f> p_list;
    while(front < c->indices.size()) //back)
    {
      back = (std::size_t)( (float)(c->indices.size()-1) * (float)rand() * (float)rand_max_inv );
      while (back == front) back = (std::size_t)( (float)(c->indices.size()-1) * (float)rand() * (float)rand_max_inv );

      Eigen::Vector3f x1(surface_->points[c->indices[front]].getVector3fMap());
      Eigen::Vector3f x2(surface_->points[c->indices[back]].getVector3fMap());
      Eigen::Vector3f n1(normals_->points[c->indices[front]].getNormalVector3fMap());
      Eigen::Vector3f n2(normals_->points[c->indices[back]].getNormalVector3fMap());
      if (!pcl_isnan(n1(0)) && !pcl_isnan(n2(0)))
      {
	Eigen::Vector3f cross_n = n2.cross(n1);
	Eigen::Vector3f dist_x = x1 - x2;
	float cross_n_denom = 1.0f / (cross_n(0)*cross_n(0) + cross_n(1)*cross_n(1) + cross_n(2)*cross_n(2));
	Eigen::Matrix3f m_s, m_t;
	m_s << dist_x, n2, cross_n;
	m_t << dist_x, n1, cross_n;
	float s = m_s.determinant() * cross_n_denom;
	float t = m_t.determinant() * cross_n_denom;
      
	Eigen::Vector3f p1 = x1 + n1 * s;
	Eigen::Vector3f p2 = x2 + n2 * t;
	p_mean += p1 + p2;
	p_list.push_back(p1);
	p_list.push_back(p2);
	intersection_points->points.push_back( pcl::PointXYZ(p1(0),p1(1),p1(2)) );
	intersection_points->points.push_back( pcl::PointXYZ(p2(0),p2(1),p2(2)) );
      }
      front += idx_steps;
      //back -= idx_steps;
    }

    float num_p_inv = 1.0f / (float)p_list.size();
    c->ints_centroid = p_mean * num_p_inv;
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (std::vector<Eigen::Vector3f>::iterator p_it = p_list.begin(); p_it != p_list.end(); ++p_it)
    {
      Eigen::Vector3f demean = *p_it - c->ints_centroid;
      cov += demean * demean.transpose();
    }
    Eigen::Vector3f eigenvalues;
    Eigen::Matrix3f eigenvectors;
    pcl::eigen33(cov, eigenvectors, eigenvalues);
    eigenvalues *= num_p_inv;

    c->ints_comp_1 = eigenvectors.col(2);
    c->ints_comp_2 = eigenvectors.col(1);
    c->ints_comp_3 = eigenvectors.col(0);
    c->ints_values = eigenvalues;
    //if ( eigenvalues(1) / (eigenvalues(0)+eigenvalues(1)+eigenvalues(2)) > 0.05)
    if ( eigenvalues(1) / (eigenvalues(0)) > 2.0)
      c->type = I_CYL;

    std::cout<<"Z = "<< c->getCentroid()(2) <<"\t2nd = "<< eigenvalues(1) / (eigenvalues(0)) <<std::endl;
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
    case I_CYL:
      for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
      {
	color_cloud->points[cluster_list[c].indices[i]].r = 0;
	color_cloud->points[cluster_list[c].indices[i]].g = 200;
	color_cloud->points[cluster_list[c].indices[i]].b = 0;
      }
      break;
    case I_SPHERE:
      for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
      {
	color_cloud->points[cluster_list[c].indices[i]].r = 255;
	color_cloud->points[cluster_list[c].indices[i]].g = 255;
	color_cloud->points[cluster_list[c].indices[i]].b = 0;
      }
      break;
    default:
      for(size_t i = 0; i<cluster_list[c].indices.size(); ++i)
      {
	color_cloud->points[cluster_list[c].indices[i]].r = 255;
	color_cloud->points[cluster_list[c].indices[i]].g = 0;
	color_cloud->points[cluster_list[c].indices[i]].b = 255;
      }
      break;
    }
  }
}


#define PCL_INSTANTIATE_ExtendedSegmentation(T,NT,CT,LabelT) template class PCL_EXPORTS cob_3d_mapping_features::ExtendedSegmentation<T,NT,CT,LabelT>;


#endif // __IMPL_EXTENDED_SEGMENTATION_H__
