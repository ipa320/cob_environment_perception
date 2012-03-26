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

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_mapping_features/extended_segmentation.h"

template <typename PointNT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointNT,PointOutT>::propagateWavefront(
  const LabelCloudPtr& labels,
  std::vector<cob_3d_mapping_common::Cluster>& cluster_out)
{
  int curr_label = I_EDGE;
  int width = labels->width, height = labels->height;
  float angle_threshold = 0.7;
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
      new_cluster.orientation = Eigen::Vector3f(normals_->points[i].normal);
      std::vector<Coords> coords_todo;
      coords_todo.push_back(Coords(i%width, i/width, false));
      while (coords_todo.size() != 0)
      {
	Coords p = coords_todo.back();
	coords_todo.pop_back();
	int p_idx = p.v * width + p.u;
	labels->points[p_idx].label = curr_label;
	new_cluster.updateCluster(p_idx, Eigen::Vector3f(normals_->points[p_idx].normal));

	// Look right
	if (p.u+1 < width && labels->points[p_idx+1].label == I_UNDEF)
	{
	  if (new_cluster.orientation.dot(Eigen::Vector3f(normals_->points[p_idx+1].normal)) > angle_threshold)
	    coords_todo.push_back(Coords(p.u+1, p.v, false));
	}
	// Look left
	if (p.u > 0 && labels->points[p_idx-1].label == I_UNDEF)
	{
	  if (new_cluster.orientation.dot(Eigen::Vector3f(normals_->points[p_idx-1].normal)) > angle_threshold)
	    coords_todo.push_back(Coords(p.u-1, p.v, false));
	}
	// Look up
	if (p.v > 0 && labels->points[p_idx-width].label == I_UNDEF)
	{
	  if (new_cluster.orientation.dot(Eigen::Vector3f(normals_->points[p_idx-width].normal)) > angle_threshold)
	    coords_todo.push_back(Coords(p.u, p.v-1, false));
	}
	// Look down
	if (p.v+1 < height  && labels->points[p_idx+width].label == I_UNDEF)
	{
	  if (new_cluster.orientation.dot(Eigen::Vector3f(normals_->points[p_idx+width].normal)) > angle_threshold)
	    coords_todo.push_back(Coords(p.u, p.v+1, false));
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

template <typename PointNT, typename PointOutT> void
cob_3d_mapping_features::ExtendedSegmentation<PointNT,PointOutT>::getColoredCloud(
  std::vector<cob_3d_mapping_common::Cluster>& cluster_list,
  pcl::PointCloud<PointXYZRGB>::Ptr& color_cloud)
{
  std::sort(cluster_list.begin()+3, cluster_list.end());
  int t = 4;
  uint32_t rgb;
  for(int c = cluster_list.size()-1; c>2; --c, ++t)
  {
    rgb = (color_tab_[t])[2] << 16 | (color_tab_[t])[1] << 8 | (color_tab_[t])[0];
    for(int i = 0; i<cluster_list[c].indices.size(); ++i)
    {
      color_cloud->points[cluster_list[c].indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
    }
  }
  //Edge
  rgb = (color_tab_[3])[2] << 16 | (color_tab_[3])[1] << 8 | (color_tab_[3])[0];
  for(int i = 0; i<cluster_list[2].indices.size(); ++i)
    color_cloud->points[cluster_list[2].indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
  //border
  rgb = (color_tab_[2])[2] << 16 | (color_tab_[2])[1] << 8 | (color_tab_[2])[0];
  for(int i = 0; i<cluster_list[1].indices.size(); ++i)
    color_cloud->points[cluster_list[1].indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
  rgb = (color_tab_[1])[2] << 16 | (color_tab_[1])[1] << 8 | (color_tab_[1])[0];
  //nan
  for(int i = 0; i<cluster_list[0].indices.size(); ++i)
    color_cloud->points[cluster_list[0].indices[i]].rgb = *reinterpret_cast<float*>(&rgb);
}

#define PCL_INSTANTIATE_ExtendedSegmentation(NT,LabelT) template class PCL_EXPORTS cob_3d_mapping_features::ExtendedSegmentation<NT,LabelT>;


#endif // __IMPL_EXTENDED_SEGMENTATION_H__
