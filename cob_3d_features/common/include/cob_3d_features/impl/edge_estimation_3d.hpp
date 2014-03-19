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
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 10/2011
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

#ifndef __IMPL_EDGE_ESTIMATION_3D_H__
#define __IMPL_EDGE_ESTIMATION_3D_H__

#include "cob_3d_features/edge_estimation_3d.h"
#include <cfloat>
#include "cob_3d_mapping_common/stop_watch.h"

//////////////////////////////////////////////////////////////////////////////////////////////
/*template <typename PointInT, typename PointNT, typename PointOutT> bool
ipa_features::EdgeEstimation<PointInT, PointNT, PointOutT>::isBoundaryPoint (
    const pcl::PointCloud<PointInT> &cloud, int q_idx,
    const std::vector<int> &indices,
    const Eigen::Vector3f &u, const Eigen::Vector3f &v,
    float angle_threshold)
    {
  return (isBoundaryPoint (cloud, cloud.points[q_idx], indices, u, v, angle_threshold));
    }

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
ipa_features::EdgeEstimation<PointInT, PointNT, PointOutT>::isBoundaryPoint (
    const pcl::PointCloud<PointInT> &cloud, const PointInT &q_point,
    const std::vector<int> &indices,
    const Eigen::Vector3f &u, const Eigen::Vector3f &v,
    float angle_threshold)
    {
  if (indices.size () < 3)
    return (false);
  float uvn_nn[2];
  Eigen::Vector3f delta;
  delta.setZero ();
  // Compute the angles between each neighboring point and the query point itself
  std::vector<float> angles;
  angles.reserve (indices.size ());
  for (size_t i = 0; i < indices.size (); ++i)
  {

    delta[0] = cloud.points[indices[i]].x - q_point.x;
    delta[1] = cloud.points[indices[i]].y - q_point.y;
    delta[2] = cloud.points[indices[i]].z - q_point.z;

    uvn_nn[0] = u.dot (delta);
    uvn_nn[1] = v.dot (delta);

    if (uvn_nn[0] == 0 && uvn_nn[1] == 0)
      continue;
    angles.push_back (atan2 (uvn_nn[1], uvn_nn[0])); // the angles are fine between -PI and PI too
  }
  std::sort (angles.begin (), angles.end ());

  // Compute the maximal angle difference between two consecutive angles
  float max_dif = FLT_MIN, dif;
  for (size_t i = 0; i < angles.size () - 1; ++i)
  {
    dif = angles[i + 1] - angles[i];
    if (max_dif < dif)
      max_dif = dif;
  }
  // Get the angle difference between the last and the first
  dif = 2 * M_PI - angles[angles.size () - 1] + angles[0];
  if (max_dif < dif)
    max_dif = dif;

  // Check results
  if (max_dif > angle_threshold)
    return (true);
  else
    return (false);
    }*/


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> double
cob_3d_features::EdgeEstimation3D<PointInT, PointNT, PointOutT>::isEdgePoint (
    const pcl::PointCloud<PointInT> &cloud, const PointInT &q_point,
    const std::vector<int> &indices,
    const Eigen::Vector3f &n, const float &threshold)
    {
  if (indices.size () < 3)
  {
    return 0.0;
  }
  float nd_dot;
  //float c_ang_thresh = cos(ang_threshold);
  //Eigen::Vector3f n_norm = n.normalized();
  Eigen::Vector3f delta;
  //delta.setZero ();
  // Compute the angles between each neighboring point and the query point itself
  //std::vector<float> angles;
  //angles.reserve (indices.size ());
  int nn_ctr=0;//, nan_ctr=0;
  int b_ctr=0;
  //std::cout << "d: ";
  for (size_t i = 0; i < indices.size (); ++i)
  {
    /*delta[0] = cloud.points[indices[i]].x - q_point.x;
    delta[1] = cloud.points[indices[i]].y - q_point.y;
    delta[2] = cloud.points[indices[i]].z - q_point.z;*/
    delta = cloud.points[indices[i]].getVector3fMap() - q_point.getVector3fMap();
    //delta = delta.normalized();

    nd_dot = n.dot (delta);

    //double nd_dot_sum;

    if(nd_dot==nd_dot)
    {
      nn_ctr++;
      if(fabs(nd_dot) > threshold) //is border point
        b_ctr++;
      //std::cout << nd_dot << ",";
      //angles.push_back (fabs(nd_dot)); // the angles are fine between -PI and PI too
      //nd_dot_sum += fabs(nd_dot);
    }
    else
    {
      //nan_ctr++;
    }
  }
  //float edge_prob = (float)b_ctr/nn_ctr;
  //float boundary_prob = (float)nan_ctr/indices.size();
  return (float)b_ctr/nn_ctr;
  /*std::cout << border_prob << std::endl;
  if(border_prob >0.00001)
	  return true;
  else
	  return false;*/
  //double nd_dot_av = nd_dot_sum/nn_ctr;
  //std::sort (angles.begin (), angles.end ());

  //float max_angle = angles.back();
  //std::cout << max_angle << "," << acos(max_angle) << std::endl;

  //TODO: mark NaNs
  if(b_ctr>0)
  //if(/*max_angle>threshold*/edge_prob>0.1 || boundary_prob>0.2)
  {
    //std::cout << "true";
    return (true);
  }
  else
  {
    //std::cout << "false";
    return (false);
  }

    }


template <typename PointInT, typename PointNT, typename PointOutT> int
cob_3d_features::EdgeEstimation3D<PointInT, PointNT, PointOutT>::searchForNeighbors (
    int index,
    double radius,
    std::vector<int>& indices,
    std::vector<float>& distances)
{
  //NaN test
  //TODO: use PCL function for that
  if(input_->points[index].x != input_->points[index].x) return 0;

  indices.clear();
  distances.clear();
  //const PointInT& p = input_->points[index];
  int idx_x = index%input_->width;
  int idx_y = index/input_->width;
  double v=2;
  double r_x=0, r_y=0;

  //TODO: choose search radius according to viewpoint distance, NaN test for neighbors
  if(idx_x<v) //border points
    r_x = fabs((input_->points[idx_y*input_->width+idx_x+2*v].x-input_->points[idx_y*input_->width+idx_x].x)/(2*v+1));
  else if(idx_x>=input_->width-v)
    r_x = fabs((input_->points[idx_y*input_->width+idx_x].x-input_->points[idx_y*input_->width+idx_x-2*v].x)/(2*v+1));
  else
    r_x = fabs((input_->points[idx_y*input_->width+idx_x+v].x-input_->points[idx_y*input_->width+idx_x-v].x)/(2*v+1));

  if(idx_y<v)
    r_y = fabs((input_->points[(idx_y+2*v)*input_->width+idx_x].y-input_->points[(idx_y)*input_->width+idx_x].y)/(2*v+1));
  else if(idx_y>=input_->height-v)
    r_y = fabs((input_->points[(idx_y)*input_->width+idx_x].y-input_->points[(idx_y-2*v)*input_->width+idx_x].y)/(2*v+1));
  else
    r_y = fabs((input_->points[(idx_y+v)*input_->width+idx_x].y-input_->points[(idx_y-v)*input_->width+idx_x].y)/(2*v+1));


  int num_nn = 3; //num_nn increments desired in each direction (-x, +x, -y, +y)
  int radius_x_pix = search_radius_;//radius/r_x;
  int radius_y_pix = search_radius_;//radius/r_y;
  //std::cout << radius_x_pix << "," << radius_y_pix << std::endl;
  int incr_x = 1;//std::max(radius_x_pix/(2*num_nn+1),1);
  int incr_y = 1;//std::max(radius_y_pix/(2*num_nn+1),1);
  //std::cout << idx_x << "," << idx_y <<  " step: " << step << std::endl;
  //std::cout << idx_x << "," << idx_y << ": ";
  for (int i=idx_x-radius_x_pix; i<=idx_x+radius_x_pix; i+=incr_x)
  {
    if(i>=input_->width) break;
    if(i<0) continue;
    for (int j=idx_y-radius_y_pix; j<=idx_y+radius_y_pix; j+=incr_y)
    {
      if(j<0) continue;
      if(i==idx_x && j==idx_y) continue; //skip p itself
      if(j>=input_->height) break;
      indices.push_back(i+j*input_->width);
      //std::cout << "(" << i << "," << j << ")";
    }
  }
  /*if (indices.size () < 3)
  {
    std::cout << idx_x << "," << idx_y << ": not enough neighbours" << std::endl;
    std::cout << r_x << "," << r_y << "," << radius_x_pix << "," << radius_y_pix << std::endl;
  }*/
  return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
cob_3d_features::EdgeEstimation3D<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  Eigen::Vector3f u, v;

  double time_nn=0, time_edge=0;
  PrecisionStopWatch sw_nn;
  PrecisionStopWatch sw_edge;
  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    //dist_threshold_=0.05;//calc_dist_threshold(*surface_ ,(*indices_)[idx]);
    Eigen::Vector3f p = input_->points[(*indices_)[idx]].getVector3fMap();
    double range = p.norm();
    dist_threshold_ = 0.0015544*search_radius_*range;
    //  ROS_INFO_STREAM("threshold" << dist_threshold_);

   if (isnan(normals_->points[(*indices_)[idx]].normal[0]))
     output.points[idx].strength=2;

    //TODO: test nn search
   else if(this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists))
    {

      /*std::cout << idx << ": ";
		for (int i=0; i<nn_indices.size(); i++)
		{
			std::cout << nn_indices[i] << ",";
		}*/


      // Obtain a coordinate system on the least-squares plane
      getCoordinateSystemOnPlane (normals_->points[(*indices_)[idx]], u, v);

      Eigen::Vector3f normal;
      normal[0] = normals_->points[(*indices_)[idx]].normal_x;
      normal[1] = normals_->points[(*indices_)[idx]].normal_y;
      normal[2] = normals_->points[(*indices_)[idx]].normal_z;
      // Estimate whether the point is lying on a boundary surface or not
      //std::cout << idx << "," << idx%input_->width << ":";
      //TODO: test edge detection
      sw_edge.precisionStart();
      output.points[idx].strength = isEdgePoint (*surface_, input_->points[(*indices_)[idx]], nn_indices, normal, dist_threshold_);
      time_edge += sw_edge.precisionStop();
    }
    else
    {
      output.points[idx].strength = 3;
      //std::cout << "point is NAN" << std::endl;
    }
    //std::cout << std::endl;
  }
  /*for(unsigned int i = 0; i < input_->height; i++)
  {
    for(unsigned int j = 0; j < input_->width; j++)
    {
      if(j==100)
      {
        std::cout << input_->points[i*input_->width+j].x << "," << input_->points[i*input_->width+j].y << ","<< input_->points[i*input_->width+j].z << std::endl;
        std::cout << normals_->points[i*input_->width+j].normal[0] << "," << normals_->points[i*input_->width+j].normal[1] << ","<< normals_->points[i*input_->width+j].normal[2] << std::endl << std::endl;
      }
    }
  }*/
  std::cout << "time for edges: " << time_edge << std::endl;
}


template <typename PointInT, typename PointNT, typename PointOutT> float
cob_3d_features::EdgeEstimation3D<PointInT, PointNT, PointOutT>::calc_dist_threshold (const pcl::PointCloud<PointInT> &cloud , int idx)
{
  //  if(input_->points[idx].z != input_->points[idx].z) return 0;


  /*double z_points_;
	int counter=0;
	for (int i=0;i<cloud.size();i++)
	{	if(cloud.points[i].z != cloud.points[i].z){
	}
	else {
		z_points_ += cloud.points[i].z;
		counter++;}
	}


		float avg_z_point_=static_cast< float >(z_points_)/counter;*/
  float threshold=static_cast< float >(-3)/static_cast< float >(430)*cloud.points[idx].z+static_cast< float >(1361)/43000;
  //	ROS_INFO_STREAM("avg z distance is  " << avg_z_point_ << "   threshold is " <<threshold );

  return threshold;
}


#define PCL_INSTANTIATE_EdgeEstimation3D(PointInT,PointNT,PointOutT) template class PCL_EXPORTS cob_3d_features::EdgeEstimation3D<PointInT, PointNT, PointOutT>;

#endif    // __IMPL_EDGE_ESTIMATION_3D_H__

