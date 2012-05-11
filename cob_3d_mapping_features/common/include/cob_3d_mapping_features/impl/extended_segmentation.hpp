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
#include <set>
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
  int width = labels_->width, height = labels_->height;
  const float angle_threshold = 35.0 / 180.0 * M_PI; // max angle difference between mean angle of cluster
  //const float angle_threshold = 0.8; // everything above belongs to cluster
  const float curvature_threshold = 20; // everything above belongs to cluster
  const int min_cluster_size = 100;
  cluster_out.clear();
  cluster_out.addNewCluster(I_NAN)->type = I_NAN;
  cluster_out.addNewCluster(I_BORDER)->type = I_BORDER;
  cluster_out.addNewCluster(I_EDGE)->type = I_EDGE;

  for(size_t i=0; i<labels_->points.size(); ++i)
  {
    if (labels_->points[i].label == I_UNDEF)
    {
      ClusterPtr new_cluster = cluster_out.addNewCluster();
      //std::list<ClusterElement> coords_todo;
      //coords_todo.push_back(ClusterElement(i%width, i/width, 0.0));
      std::multiset<ClusterElement> coords_todo;
      coords_todo.insert(ClusterElement(i%width, i/width, 0.0));
      labels_->points[i].label = new_cluster->id;
      while (coords_todo.size() != 0)
      {
	//ClusterElement p = coords_todo.front();
	ClusterElement p = *coords_todo.begin();
	coords_todo.erase(coords_todo.begin());
	//coords_todo.pop_front();
	float cos_angle_norm, cos_angle_dist, angle, plane_distance;
	Eigen::Vector3f p_i;
	int p_idx = p.v * width + p.u;
	int p_new;
	new_cluster->addNewPoint(
	  p_idx, surface_->points[p_idx].getVector3fMap(),
	  Eigen::Vector3f(normals_->points[p_idx].normal));
	//bool is_highly_curved = (std::abs(new_cluster.getMaxCurvature()) > curvature_threshold);
	
	Eigen::Vector3f c = new_cluster->getCentroid();
	//float dist_threshold = 20.0 * 0.003 * surface_->points[p_idx].z *surface_->points[p_idx].z;
	// Look right
	if (p.u+1 < width)
	{
	  /* Cluster is still to small to have convincing mean values
	   * --- or ---
	   * Cluster has high curvature and most likely being some sort of more complex object,
	   * so add to cluster and ignore normal comparision
	   * --- or ---
	   * Cluster has low mean curvature and is most likely a plane
	   * so check if the orientation of the new point is similar to the cluster
	   */
	  if (labels_->points[(p_new = p_idx+1)].label == I_UNDEF)
	  {
	    //cos_angle_norm = new_cluster->getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx+1].normal));
	    cos_angle_norm = 
	      fabs(atan2((new_cluster->getOrientation().cross(normals_->points[p_new].getNormalVector3fMap())).norm(),
			new_cluster->getOrientation().dot(normals_->points[p_new].getNormalVector3fMap())));
	    if ( (int)new_cluster->indices.size() < min_cluster_size || 
		 //(fabs(plane_distance - plane_distance_cluster) < dist_threshold &&
		 cos_angle_norm < angle_threshold)// && std::fabs(p_i.norm() * angle) < dist_threshold) )
	    {
	      labels_->points[p_new].label = new_cluster->id;
	      //coords_todo.push_back(ClusterElement(p.u+1, p.v, cos_angle_norm));
	      coords_todo.insert(ClusterElement(p.u+1, p.v, cos_angle_norm));
	    }
	  }
	  else if (labels_->points[p_new].label > I_EDGE)
	  { 
	    cluster_out.connect(new_cluster->id, labels_->points[p_new].label, p_idx, p_new);
	  }
	}

	// Look down
	if (p.v+1 < height)
	{
	  if (labels_->points[(p_new = p_idx+width)].label == I_UNDEF)
	  {
	    //cos_angle_norm = new_cluster->getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx+width].normal));
	    cos_angle_norm = 
	      fabs(atan2((new_cluster->getOrientation().cross(normals_->points[p_new].getNormalVector3fMap())).norm(),
			new_cluster->getOrientation().dot(normals_->points[p_new].getNormalVector3fMap())));
	    if ( (int)new_cluster->indices.size() < min_cluster_size || 
		 //(fabs(plane_distance - plane_distance_cluster) < dist_threshold &&
		 cos_angle_norm < angle_threshold)//&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	    {
	      labels_->points[p_new].label = new_cluster->id;
	      //coords_todo.push_back(ClusterElement(p.u, p.v+1, cos_angle_norm));
	      coords_todo.insert(ClusterElement(p.u, p.v+1, cos_angle_norm));
	    }
	  }
	  else if (labels_->points[p_new].label > I_EDGE)
	  {
	    cluster_out.connect(new_cluster->id, labels_->points[p_new].label, p_idx, p_new);
	  }
	}

	// Look left
	if (p.u > 0)
	{
	  if (labels_->points[(p_new = p_idx-1)].label == I_UNDEF)
	  {
	    //cos_angle_norm = new_cluster->getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx-1].normal));
	    cos_angle_norm = 
	      fabs(atan2(new_cluster->getOrientation().cross(normals_->points[p_new].getNormalVector3fMap()).norm(),
			new_cluster->getOrientation().dot(normals_->points[p_new].getNormalVector3fMap())));
	    //plane_distance = surface_->points[p_idx-1].getVector3fMap().dot(normals_->points[p_idx-1].getNormalVector3fMap());
	    if ( (int)new_cluster->indices.size() < min_cluster_size || 
		 //(fabs(plane_distance - plane_distance_cluster) < dist_threshold &&
		 cos_angle_norm < angle_threshold) //&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	    {
	      labels_->points[p_new].label = new_cluster->id;
	      //coords_todo.push_back(ClusterElement(p.u-1, p.v, cos_angle_norm));
	      coords_todo.insert(ClusterElement(p.u-1, p.v, cos_angle_norm));

	    }
	  }
	  else if (labels_->points[p_idx-1].label > I_EDGE)
	  { 
	    cluster_out.connect(new_cluster->id, labels_->points[p_new].label, p_idx, p_new);
	  }
	}

	// Look up
	if (p.v > 0)
	{
	  if (labels_->points[(p_new = p_idx-width)].label == I_UNDEF)
	  {
	    //cos_angle_norm = new_cluster->getOrientation().dot(Eigen::Vector3f(normals_->points[p_idx-width].normal));
	    cos_angle_norm = 
	      fabs(atan2((new_cluster->getOrientation().cross(normals_->points[p_new].getNormalVector3fMap())).norm(),
			new_cluster->getOrientation().dot(normals_->points[p_new].getNormalVector3fMap())));
	    if ( (int)new_cluster->indices.size() < min_cluster_size || 
		 //(fabs(plane_distance - plane_distance_cluster) < dist_threshold &&
		 cos_angle_norm < angle_threshold)//&& std::fabs(p_i.norm() * angle) < dist_threshold) )
	    {
	      labels_->points[p_new].label = new_cluster->id;
	      //coords_todo.push_back(ClusterElement(p.u, p.v-1, cos_angle_norm));
	      coords_todo.insert(ClusterElement(p.u, p.v-1, cos_angle_norm));
	    }
	  }
	  else if (labels_->points[p_new].label > I_EDGE)
	  { 
	    cluster_out.connect(new_cluster->id, labels_->points[p_new].label, p_idx, p_new);
	  }
	}
	//coords_todo.sort();
      } // end while
    }
    else if(labels_->points[i].label <= I_EDGE)
      cluster_out[labels_->points[i].label]->indices.push_back(i);
    else
      continue;
  }
  return;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::propagateWavefront2ndPass(
  ClusterList& cluster_list)
{
  std::cout << "[propagateWavefront2ndPass] currently not implemented" << std::endl;
  return;
/*
  // ---- Find potential planes and drop all other clusters -----
  std::vector<int> indices_todo;
  ClusterPtr c = cluster_list.begin();
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
    ++c;
    //c = cluster_list.erase(c);
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
    cob_3d_mapping_features::Cluster new_cluster;
    std::list<ClusterElement> coords_todo;
    coords_todo.push_back(ClusterElement((*idx)%width, (*idx)/width, 0.0));
    std::queue<Eigen::Vector3f> normal_buffer;
    labels_->points[*idx].label = curr_label;
    while (coords_todo.size() != 0)
    {
      ClusterElement p = coords_todo.front();
      coords_todo.pop_front();
      int p_idx = p.v * width + p.u;
      new_cluster.addNewPoint(p_idx,surface_->points[p_idx].getVector3fMap(),Eigen::Vector3f(normals_->points[p_idx].normal));
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
*/
}


template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> bool
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::hasValidCurvature(int idx)
{
  std::cout << "[hasValidCurvature] currently not implemented" << std::endl;
  return false;
/*
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
*/
}


template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> bool
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeClusterComponents(
  ClusterPtr c)
{
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  Eigen::Vector3f centroid = c->getCentroid();
  for (std::vector<int>::iterator idx = c->indices.begin(); idx != c->indices.end(); ++idx)
  {
    Eigen::Vector3f demean = surface_->points[*idx].getVector3fMap() - centroid;
    cov += demean * demean.transpose();
  }

  Eigen::Matrix3f eigenvectors;
  Eigen::Vector3f eigenvalues;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  eigenvalues /= c->indices.size();
  c->pca_point_comp1 = eigenvectors.col(2);
  c->pca_point_comp2 = eigenvectors.col(1);
  c->pca_point_comp3 = eigenvectors.col(0);
  c->pca_point_values = eigenvalues;
  if ( eigenvalues(0) / (eigenvalues(0)+eigenvalues(1)+eigenvalues(2)) < 0.001 * centroid(2) * centroid(2) )
  {
    c->type = I_PLANE;
    c->is_save_plane = true;
  }
  return c->is_save_plane;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeClusterCurvature(
  ClusterPtr c)
{
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
  c->max_curvature = eigenvalues(2) * num_p_inv;
  c->min_curvature = eigenvalues(1) * num_p_inv;
  c->min_curvature_direction = eigenvectors.col(1);
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeClusterPointCurvature(
  ClusterPtr c, int search_size)
{
  //float mean_curvature_max=0.0, mean_curvature_min=0.0;
  std::vector<int> geometry(NUM_LABELS, 0);
  int y_offset = search_size * surface_->width;
  int cluster_label = labels_->points[c->indices.front()].label;
  for (std::vector<int>::iterator idx=c->indices.begin(); idx != c->indices.end(); ++idx)
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
  c->type = max_idx;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeClusterNormalIntersections(
  ClusterPtr c)
{
  const std::size_t idx_steps = 1;
  const float rand_max_inv = 1.0f / RAND_MAX;
  std::size_t front = 0, back = c->indices.size()-1;
  Eigen::Vector3f p_mean = Eigen::Vector3f::Zero();
  std::vector<Eigen::Vector3f> p_list;
  //std::cout << "Inter: " << c->id << std::endl;
  while(front < c->indices.size()) //back)
  {
    // find random normal pair on cluster
    back = (std::size_t)( (float)(c->indices.size()-1) * (float)rand() * (float)rand_max_inv );
    while (back == front) back = (std::size_t)( (float)(c->indices.size()-1) * (float)rand() * (float)rand_max_inv );

    Eigen::Vector3f x1(surface_->points[c->indices[front]].getVector3fMap());
    Eigen::Vector3f x2(surface_->points[c->indices[back]].getVector3fMap());
    Eigen::Vector3f n1(normals_->points[c->indices[front]].getNormalVector3fMap());
    Eigen::Vector3f n2(normals_->points[c->indices[back]].getNormalVector3fMap());
    //if (pcl_isnan(n1(0)) || pcl_isnan(n2(0)) || pcl_isnan(x1(0)) || pcl_isnan(x2(0)))
    //std::cout<<"n1:"<<n1(0)<<" n2:"<<n2(0)<<" x1:"<<x1(0)<<" x2:"<<x2(0)<<std::endl;
    //if (!pcl_isnan(n1(0)) && !pcl_isnan(n2(0)))
    //{
    Eigen::Vector3f cross_n = n2.cross(n1);
    Eigen::Vector3f dist_x = x1 - x2;
    /*if (cross_n(0) == 0)
    {
      std::cout << cross_n(0) <<" "<< cross_n(1)<<" "<<cross_n(2) << std::endl;
      std::cout<<"n1:"<<n1 << std::endl;
      std::cout<<"n2:"<<n2 << std::endl;
      std::cout<<" x1:"<<x1<< std::endl;
      std::cout<<" x2:"<<x2<<std::endl;
    }*/
    if ( !((cross_n(0) == cross_n(1)) && (cross_n(1) == cross_n(2)) && (cross_n(2) == 0)) )
    {
      float cross_n_denom = 1.0f / (cross_n(0)*cross_n(0) + cross_n(1)*cross_n(1) + cross_n(2)*cross_n(2));
      //if (pcl_isnan(cross_n_denom)) std::cout << "Denom:"<< front << std::endl;
      Eigen::Matrix3f m_s, m_t;
      m_s << dist_x, n2, cross_n;
      m_t << dist_x, n1, cross_n;
      float s = m_s.determinant() * cross_n_denom;
      float t = m_t.determinant() * cross_n_denom;
      //std::cout<<"s:" << front << " | "<< cross_n_denom<<  std::endl;
      //std::cout<<"t:" << front << " | "<< cross_n_denom<<  std::endl;
      Eigen::Vector3f p1 = x1 + n1 * s;
      Eigen::Vector3f p2 = x2 + n2 * t;
      p_mean += p1 + p2;
      p_list.push_back(p1);
      p_list.push_back(p2);
    }
    front += idx_steps;
  }
  
  float num_p_inv = 1.0f / (float)p_list.size();
  c->pca_inter_centroid = p_mean * num_p_inv;
  //std::cout << "" << c->pca_inter_centroid(0) <<","<<c->pca_inter_centroid(1)<<","<<c->pca_inter_centroid(1) << std::endl;
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (std::vector<Eigen::Vector3f>::iterator p_it = p_list.begin(); p_it != p_list.end(); ++p_it)
  {
    Eigen::Vector3f demean = *p_it - c->pca_inter_centroid;
    cov += demean * demean.transpose();
  }
  Eigen::Vector3f eigenvalues;
  Eigen::Matrix3f eigenvectors;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  eigenvalues *= num_p_inv;

  c->pca_inter_comp1 = eigenvectors.col(2);
  c->pca_inter_comp2 = eigenvectors.col(1);
  c->pca_inter_comp3 = eigenvectors.col(0);
  c->pca_inter_values = eigenvalues;
  //std::cout << "Inter: " << c->id << " done" << std::endl;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::mergeClusterProperties(
  ClusterPtr c_src, ClusterPtr c_trg)
{
  // TODO: boundary points will not be merged yet!
  for (std::vector<int>::iterator it = c_src->indices.begin(); it != c_src->indices.end(); ++it)
  {
    c_trg->addNewPoint(*it, surface_->points[*it].getVector3fMap(), normals_->points[*it].getNormalVector3fMap());
    labels_->points[*it].label = c_trg->id;
  }
  //std::cout<<"components"<<std::endl;
  //computeClusterComponents(c_trg);
  //std::cout<<"intersections"<<std::endl;
  //computeClusterNormalIntersections(c_trg);
  //c_trg->type = I_UNDEF;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::joinAdjacentRotationalClusters(
  ClusterPtr c,
  ClusterList& cluster_list)
{
  const float max_angle = 45.0 / 180.0 * M_PI;
  const float min_smoothness = 0.80;
  std::vector<ClusterPtr> adjacent_clusters;
  //cluster_list.getMinAngleAdjacentClusters(c->id, max_angle, adjacent_clusters);
  cluster_list.getAdjacentClustersWithSmoothBoundaries(c->id, min_smoothness, adjacent_clusters);
  //std::cout << "Adjacent Clusters: " << adjacent_clusters.size() << std::endl;
  for (std::vector<ClusterPtr>::iterator a_it = adjacent_clusters.begin(); a_it != adjacent_clusters.end(); ++a_it)
  {
    mergeClusterProperties(*a_it, c);
    //std::cout << "Done properties" << std::endl;
    cluster_list.mergeClusterDataStructure( (*a_it)->id, c->id);
    //std::cout << "Done data structure" << std::endl;
  }
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::joinAdjacentRotationalClustersOld(
  ClusterPtr c,
  ClusterList& cluster_list)
{
  const float max_angle = 45.0 / 180.0 * M_PI;
  const float max_dist = 0.5;
  bool still_something_todo = !c->is_save_plane;
  while (still_something_todo)
  {
    //std::cout << "1" << std::endl;
    std::vector<ClusterPtr> adjacent_clusters;
    if ( pcl_isnan(c->pca_inter_centroid(0)) ) std::cout << "Gotcha... -1 | " << c->id << std::endl;
    if ( pcl_isnan(c->getCentroid()(0)) ) std::cout << "Gotcha... 0 | " << c->id << std::endl;
    Eigen::Vector3f c_int = c->pca_inter_centroid - c->getCentroid();
    //std::cout << "2" << std::endl;
    cluster_list.getAdjacentClusters(c->id, adjacent_clusters);
    //std::cout << cluster_list.size() << std::endl;
    //std::cout << "3" << std::endl;
    for (std::vector<ClusterPtr>::iterator a_it = adjacent_clusters.begin(); a_it != adjacent_clusters.end(); ++a_it)
    {
      if ( (*a_it)->is_save_plane ) continue;

      //if ( pcl_isnan((*a_it)->pca_inter_centroid(0)) ) std::cout << "Gotcha... 1" << std::endl;
      //if ( pcl_isnan(c->pca_inter_centroid(0)) ) std::cout << "Gotcha... 2" << std::endl;
      //if ( pcl_isnan((*a_it)->getCentroid()(0)) ) std::cout << "Gotcha... 3" << std::endl;
      //std::cout << "calc angles" << std::endl;
      if ( ((*a_it)->pca_inter_centroid - c->pca_inter_centroid).norm() > max_dist ) continue;
      Eigen::Vector3f a_int = (*a_it)->pca_inter_centroid;
      Eigen::Vector3f a = (*a_it)->getCentroid();
      Eigen::Vector3f a_cross_c = (a_int - a).cross(c_int);
      float a_dot_c = (a_int - a).dot(c_int);
      float angle = atan2( a_cross_c.norm(), a_dot_c );
      //Eigen::Vector3f cross = (*a_it)->getOrientation().cross(c->getOrientation());
      //float dot = (*a_it)->getOrientation().dot(c->getOrientation());
      //float angle = atan2( cross.norm(), dot);
      //std::cout << "c:"<<c_int(0)<<","<<c_int(1)<<","<<c_int(2)<<std::endl;
      //std::cout << "a:"<<a(0)<<","<<a(1)<<","<<a(2)<<" | "<<a_int(0)<<","<<a_int(1)<<","<<a_int(2)<<std::endl;
      //std::cout << "cross|dot:"<<a_cross_c(0)<<","<<a_cross_c(1)<<","<<a_cross_c(2)<<" | "<<a_dot_c<<std::endl;
      //std::cout << angle << " " << fabs(angle) << std::endl;
      if ( fabs(angle) > max_angle ) continue;
      
      //std::cout << "merge properties" << std::endl;
      mergeClusterProperties(*a_it, c);
      //std::cout << "merge data structure" << std::endl;
      cluster_list.mergeClusterDataStructure( (*a_it)->id, c->id);
      
      still_something_todo = false;
      break;
    }
    still_something_todo = !still_something_todo; // if we merged before, we still have something todo!
    //std::cout << (still_something_todo ? "true" : "false") << std::endl;
  }
}

/*
template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeBoundarySmoothness(
  ClusterList& cl)
{
  boost::graph_traits<ClusterList::GraphT>::edge_iterator e_it, e_end;
  ClusterPtr c1;//, c2;
  const float max_angle = cos(35.0 * M_PI / 180);
  for (boost::tie(e_it,e_end)=boost::edges(cl.g_); e_it != e_end; ++e_it)
  {
    //c2 = cl.g_[boost::source(*e_it, cl.g_)].c_it;
    c1 = cl.g_[boost::target(*e_it, cl.g_)].c_it;
    int smooth_points = 0;
    std::map<int,int>::iterator it = (cl.g_[*e_it].boundary_points[c1->id]).begin();
    for ( ; it != (cl.g_[*e_it].boundary_points[c1->id]).end(); ++it)
    {
      if ( max_angle < normals_->points[(*it).first].getNormalVector3fMap().dot(
	     normals_->points[(*it).second].getNormalVector3fMap()) )
	++smooth_points;
    }
    cl.g_[*e_it].smoothness = (float)smooth_points / (cl.g_[*e_it].boundary_points[c1->id]).size();
  }
}
*/

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::recomputeClusterNormals(ClusterPtr c)
{
  OrganizedNormalEstimation<PointT,PointNT,PointOutT> one;
  int w_size = std::min( std::floor(sqrt(c->indices.size() / 16.0f))+ 8, 30.0);
  //int w_size = std::floor(4.0 * log10(c->indices.size()));
  //std::cout << "Size="<<w_size<< std::endl;
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
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeBoundaryProperties(
  ClusterPtr c,
  ClusterList& cluster_list)
{
  std::vector<ClusterPtr> adj_c;
  std::vector<std::map<int,int>* > boundary_lists;
  cluster_list.getAdjacentClusters(c->id, adj_c);
  int perimeter = 0;
  for (std::vector<ClusterPtr>::iterator a_it = adj_c.begin(); a_it != adj_c.end(); ++a_it)
  {
    Edge e = cluster_list.getConnection(c->id, (*a_it)->id);
    //perimeter = round(static_cast<float>(e.boundary_points[c->id].size()) / 10.0f + pow(c->getCentroid()(2),2));
    //std::cout << perimeter <<" = "<<e.boundary_points[c->id].size()<<" / 10.0f + "<<c->getCentroid()(2)<<"^2" << std::endl;
    perimeter = ceil( std::min(
			sqrt((float)c->size()) / (float)e.boundary_points[c->id].size() + pow(c->getCentroid()(2), 2)+5.0, 30.0) );
    //std::cout << perimeter <<" = "<<c->size()<<"/"<<e.boundary_points[c->id].size()<<" + "<<c->getCentroid()(2)<<"^2" << std::endl;
    for (std::map<int,int>::iterator b_it = e.boundary_points[c->id].begin(); b_it != e.boundary_points[c->id].end(); ++b_it)
    {
      computeBoundaryPointProperties(perimeter, b_it->first, cluster_list.getBoundaryPoint(b_it->first));
    }
  }
/*
  for (std::vector<ClusterPtr>::iterator a_it = adj_c.begin(); a_it != adj_c.end(); ++a_it)
  {
    boundary_lists.push_back( &( (cluster_list.getConnection(c->id, (*a_it)->id)).boundary_points[c->id] ) );
    perimeter += boundary_lists.back()->size();
  }
  for (std::vector<std::map<int,int>* >::iterator l_it = boundary_lists.begin(); l_it != boundary_lists.end(); ++l_it)
  {
    for (std::map<int,int>::iterator b_it = (*l_it)->begin(); b_it != (*l_it)->end(); ++b_it)
    {
      computeBoundaryPointProperties(5, b_it->first, cluster_list.getBoundaryPoint(b_it->first));
    }
  }
*/
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::computeBoundaryPointProperties(
  const int r,
  const int index,
  BoundaryPoint& bp)
{
  const int w = surface_->width, h = surface_->height, s = surface_->height * surface_->width;
  const int l_idx = labels_->points[index].label;

  // compute mask boundary constrains first, 
  const int w_rem = index%w;
  const int x_max = std::min(2*r, r + w - w_rem - 1); // max offset at each line
  const int y_min = std::max(index - r*w - r, w_rem - r);
  const int y_max = std::min(index + r*w - r, s - (w - w_rem) - r);
  Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero();
  int point_count = 0;
  for (int y = y_min; y <= y_max; y += w) // y: beginning of each line
  {
    for (int idx = y; idx < y + x_max; ++idx) 
    {
      if (labels_->points[idx].label != l_idx) continue;
      PointT const* p_idx = &(surface_->points[idx]);
      if (pcl_isnan(p_idx->x)) continue;
      accu[0] += p_idx->x * p_idx->x;
      accu[1] += p_idx->x * p_idx->y;
      accu[2] += p_idx->x * p_idx->z;
      accu[3] += p_idx->y * p_idx->y;
      accu[4] += p_idx->y * p_idx->z;
      accu[5] += p_idx->z * p_idx->z;
      accu[6] += p_idx->x;
      accu[7] += p_idx->y;
      accu[8] += p_idx->z;
      ++point_count;
    }
  }
  accu /= static_cast<float>(point_count);
    
  Eigen::Matrix3f cov;
  cov.coeffRef(0) =                   accu(0) - accu(6) * accu(6);
  cov.coeffRef(1) = cov.coeffRef(3) = accu(1) - accu(6) * accu(7);
  cov.coeffRef(2) = cov.coeffRef(6) = accu(2) - accu(6) * accu(8);
  cov.coeffRef(4) =                   accu(3) - accu(7) * accu(7);
  cov.coeffRef(5) = cov.coeffRef(7) = accu(4) - accu(7) * accu(8);
  cov.coeffRef(8) =                   accu(5) - accu(8) * accu(8);
  Eigen::Vector3f eigenvalues;
  Eigen::Matrix3f eigenvectors;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  if ( surface_->points[index].getVector3fMap().dot(eigenvectors.col(0)) > 0)
    bp.normal = eigenvectors.col(0) * (-1);
  else
    bp.normal = eigenvectors.col(0);
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::analyseClusters(
  ClusterList& cluster_list)
{
  cluster_list.sort();
  for (ClusterPtr c = cluster_list.begin(); c != cluster_list.end(); ++c)
  {
    if (c->indices.size() < 20) continue;
    if (c->type == I_EDGE || c->type == I_NAN || c->type == I_BORDER) continue;
    //computeClusterComponents(c);
    computeBoundaryProperties(c, cluster_list);
    //computeClusterNormalIntersections(c);
    //cluster_list.computeEdgeAngles(c->id);
  }
  //std::cout << "Second.. " << std::endl;
  //computeBoundarySmoothness(cluster_list);
  cluster_list.computeEdgeSmoothness(cos(45.0/180.0*M_PI));
  //cluster_list.removeSmallClusters();

  for (ClusterPtr c = --cluster_list.end(); c != cluster_list.begin(); --c)
  {
    //std::cout << "ID: " << c->id << std::endl;
    if (c->indices.size() < 5) continue;
    if (c->type == I_EDGE || c->type == I_NAN || c->type == I_BORDER) continue;
    joinAdjacentRotationalClusters(c, cluster_list);
    computeClusterComponents(c);
    if (!c->is_save_plane) 
    {
      recomputeClusterNormals(c);
      computeClusterCurvature(c);
      if (c->max_curvature < 0.02) c->type = I_PLANE;
      else if(c->max_curvature < 9.0 * c->min_curvature) c->type = I_SPHERE;
      else c->type = I_CYL;
    }
    else c->type = I_PLANE;
    //std::cout << "ID: " << c->id << " done"<< std::endl;
  }

  //std::cout << "done" << std::endl;
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::getBoundaryCloud(
  ClusterList& cluster_list,
  pcl::PointCloud<PointXYZRGB>::Ptr& boundary_points,
  pcl::PointCloud<Normal>::Ptr& boundary_normals)
{
  boundary_points->clear();
  boundary_normals->clear();
  boundary_points->resize(cluster_list.bp_size());
  boundary_normals->resize(cluster_list.bp_size());
  boundary_points->width = boundary_normals->width = 1;
  boundary_points->height = boundary_normals->height = cluster_list.bp_size();
  typename pcl::PointCloud<PointT>::iterator p_it = boundary_points->begin();
  typename pcl::PointCloud<PointNT>::iterator n_it = boundary_normals->begin();
  for(std::map<int,BoundaryPoint>::iterator b_it = cluster_list.bp_begin(); b_it != cluster_list.bp_end(); ++b_it, ++p_it, ++n_it)
  {
    *p_it = surface_->points[b_it->first];
    n_it->getNormalVector3fMap() = b_it->second.normal;
  }
}

template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::getColoredCloud(
  ClusterList& cluster_list,
  pcl::PointCloud<PointXYZRGB>::Ptr& color_cloud)
{
  cluster_list.sort();
  uint32_t rgb;
  int t = 1;
  for(std::list<Cluster>::reverse_iterator c = cluster_list.rbegin(); c != cluster_list.rend(); ++c, ++t)
  {
    if (c->type == I_NAN || c->type == I_BORDER)
    {
      rgb = (color_tab_[0])[2] << 16 | (color_tab_[0])[1] << 8 | (color_tab_[0])[0];
      --t;
    }
    else
    {
      rgb = (color_tab_[t])[2] << 16 | (color_tab_[t])[1] << 8 | (color_tab_[t])[0];
    }
    for(size_t i = 0; i<c->indices.size(); ++i)
    {
      color_cloud->points[c->indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
    }
  }
}


template <typename PointT, typename PointNT, typename PointCT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointT,PointNT,PointCT,PointOutT>::getColoredCloudByType(
  ClusterList& cluster_list,
  pcl::PointCloud<PointXYZRGB>::Ptr& color_cloud)
{
  for(ClusterPtr c = cluster_list.begin(); c != cluster_list.end(); ++c)
  {
    switch(c->type)
    {
    case I_NAN:
      break;
    case I_BORDER:
      for(size_t i = 0; i<c->indices.size(); ++i)
      {
	color_cloud->points[c->indices[i]].r = 255;
	color_cloud->points[c->indices[i]].g = 255;
	color_cloud->points[c->indices[i]].b = 255;
      }
      break;
    case I_EDGE:
      for(size_t i = 0; i<c->indices.size(); ++i)
      {
	color_cloud->points[c->indices[i]].r = 255;
	color_cloud->points[c->indices[i]].g = 0;
	color_cloud->points[c->indices[i]].b = 0;
      }
      break;
    case I_PLANE:
      for(size_t i = 0; i<c->indices.size(); ++i)
      {
	color_cloud->points[c->indices[i]].r = 0;
	color_cloud->points[c->indices[i]].g = 200;
	color_cloud->points[c->indices[i]].b = 255;
      }
      break;
    case I_CYL:
      for(size_t i = 0; i<c->indices.size(); ++i)
      {
	color_cloud->points[c->indices[i]].r = 0;
	color_cloud->points[c->indices[i]].g = 200;
	color_cloud->points[c->indices[i]].b = 0;
      }
      break;
    case I_SPHERE:
      for(size_t i = 0; i<c->indices.size(); ++i)
      {
	color_cloud->points[c->indices[i]].r = 255;
	color_cloud->points[c->indices[i]].g = 255;
	color_cloud->points[c->indices[i]].b = 0;
      }
      break;
    default:
      for(size_t i = 0; i<c->indices.size(); ++i)
      {
	color_cloud->points[c->indices[i]].r = 255;
	color_cloud->points[c->indices[i]].g = 0;
	color_cloud->points[c->indices[i]].b = 255;
      }
      break;
    }
  }
}


#define PCL_INSTANTIATE_ExtendedSegmentation(T,NT,CT,LabelT) template class PCL_EXPORTS cob_3d_mapping_features::ExtendedSegmentation<T,NT,CT,LabelT>;


#endif // __IMPL_EXTENDED_SEGMENTATION_H__
