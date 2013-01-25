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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
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
#include "cob_3d_mapping_features/segmentation.h"
#include <pcl/io/pcd_io.h>
#include <highgui.h>

int main()
{
  cob_3d_mapping_features::Segmentation seg;
  pcl::PointCloud<PointLabel>::Ptr cloud_in = pcl::PointCloud<PointLabel>::Ptr (new pcl::PointCloud<PointLabel>);
  cv::Mat seg_img;
  std::vector<pcl::PointIndices> cluster_indices;
  /*cloud_in->points.resize(900);
  cloud_in->width = 30;
  cloud_in->height = 30;
  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(i<14 && j==14) cloud_in->points[i*30+j].label = 1;
      if(i>16 && j==14) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg",seg_img);

  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(j<14 && i==14) cloud_in->points[i*30+j].label = 1;
      if(j>16 && i==14) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg2",seg_img);

  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(i<14 && j==14) cloud_in->points[i*30+j].label = 1;
      if(i>16 && j==15) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg3",seg_img);

  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(i<14 && j==14) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg4",seg_img);*/

  pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  pcl::PointCloud<PointLabel>::Ptr cloud_in_c = pcl::PointCloud<PointLabel>::Ptr (new pcl::PointCloud<PointLabel>);
  for(unsigned int i=0; i<cloud_in->points.size(); i++)
  {
    if(i>=64000)
    {
      cloud_in_c->points.push_back(cloud_in->points[i]);
    }
  }
  cloud_in_c->width = 640;
  cloud_in_c->height = 380;
  seg.propagateWavefront2(cloud_in_c);
  seg.getClusterIndices(cloud_in_c, cluster_indices, seg_img);
  cv::imshow("seg_real",seg_img);

  cv::waitKey();
  return 0;
}
