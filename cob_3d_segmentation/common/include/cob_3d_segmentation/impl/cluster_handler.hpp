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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 05/2012
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#ifndef __IMPL_CLUSTER_HANDLER_HPP__
#define __IMPL_CLUSTER_HANDLER_HPP__

#include "cob_3d_segmentation/cluster_handler.h"


template<typename LabelT, typename PointT, typename PointNT> void
cob_3d_segmentation::DepthClusterHandler<LabelT,PointT,PointNT>::computeClusterComponents(ClusterPtr c)
{
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  Eigen::Vector3f centroid = c->getCentroid();
  for (ClusterType::iterator idx = c->begin(); idx != c->end(); ++idx)
  {
    Eigen::Vector3f demean = surface_->points[*idx].getVector3fMap() - centroid;
    cov += demean * demean.transpose();
  }

  Eigen::Matrix3f eigenvectors;
  Eigen::Vector3f eigenvalues;
  pcl::eigen33(cov, eigenvectors, eigenvalues);
  eigenvalues /= c->size();
  c->pca_point_comp1 = eigenvectors.col(2);
  c->pca_point_comp2 = eigenvectors.col(1);
  c->pca_point_comp3 = eigenvectors.col(0);
  c->pca_point_values = eigenvalues;
  
  //std::cout << c->id() <<"("<< colorHumanReadable(c->label_color) <<") "<<c->size()<<" | "<< eigenvalues(0) <<", "<< eigenvalues(1) <<", "<< eigenvalues(2)<<" | ";
  if ( eigenvalues(0) / (eigenvalues(0)+eigenvalues(1)+eigenvalues(2)) < 0.001 * centroid(2) * centroid(2) )
  {
    c->type = I_PLANE;
    c->is_planar_ = true;
  }
  //std::cout << c->is_save_plane << std::endl;
}

template<typename LabelT, typename PointT, typename PointNT> void
cob_3d_segmentation::DepthClusterHandler<LabelT,PointT,PointNT>::computeCurvature(ClusterPtr c)
{
  // compute principal curvature of the segment in its entirety
  Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - c->getOrientation() * c->getOrientation().transpose();
  std::vector<Eigen::Vector3f> normals_projected;
  Eigen::Vector3f n_centroid = Eigen::Vector3f::Zero();
  for (ClusterType::iterator idx = c->begin(); idx != c->end(); ++ idx)
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

template<typename LabelT, typename PointT, typename PointNT> void
cob_3d_segmentation::DepthClusterHandler<LabelT,PointT,PointNT>::computeNormalIntersections(ClusterPtr c)
{
  const std::size_t idx_steps = 1;
  const float rand_max_inv = 1.0f / RAND_MAX;
  std::size_t front = 0, back = c->size()-1;
  Eigen::Vector3f p_mean = Eigen::Vector3f::Zero();
  std::vector<Eigen::Vector3f> p_list;
  //std::cout << "Inter: " << c->id << std::endl;
  while(front < c->size()) //back)
  {
    // find random normal pair on cluster
    back = (std::size_t)( (float)(c->size()-1) * (float)rand() * (float)rand_max_inv );
    while (back == front) back = (std::size_t)( (float)(c->size()-1) * (float)rand() * (float)rand_max_inv );

    Eigen::Vector3f x1(surface_->points[(*c)[front]].getVector3fMap());
    Eigen::Vector3f x2(surface_->points[(*c)[back]].getVector3fMap());
    Eigen::Vector3f n1(normals_->points[(*c)[front]].getNormalVector3fMap());
    Eigen::Vector3f n2(normals_->points[(*c)[back]].getNormalVector3fMap());
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
    Eigen::Vector3f demean = *p_it;// - c->pca_inter_centroid;
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

#endif
