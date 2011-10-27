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

#include <boost/timer.hpp>
#include "cob_3d_mapping_features/segmentation.h"

 int
  Segmentation::searchForNeighbors (
      pcl::PointCloud<PointLabel>::Ptr& cloud_in,
      int col, int row,
      double radius,
      std::vector<int>& indices_ul,
      std::vector<int>& indices_ur,
      std::vector<int>& indices_lr,
      std::vector<int>& indices_ll,
      bool gap_l, bool gap_r, bool gap_a, bool gap_d)
  {
    //NaN test
    //TODO: use PCL function for that
    //if(cloud_in->points[index].x != cloud_in->points[index].x) return 0;

    gap_l=gap_r=gap_a=gap_d=false;
    indices_ul.clear();
    indices_ur.clear();
    indices_lr.clear();
    indices_ll.clear();

    int idx_x = col;
    int idx_y = row;

    int gap_l_ctr = 0;
    int gap_a_ctr = 0;
    for (int i=idx_x-radius+1; i<=idx_x; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y-radius+1; j<=idx_y; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ul.push_back(i+j*cloud_in->width);
        if(j==idx_y && cloud_in->points[i+j*cloud_in->width].label!=1) gap_l_ctr++;
        if(i==idx_x && cloud_in->points[i+j*cloud_in->width].label!=1) gap_a_ctr++;
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    if(gap_l_ctr==0) gap_l = true;
    if(gap_a_ctr==0) gap_a = true;

    int gap_r_ctr = 0;
    int gap_d_ctr = 0;
    for (int i=idx_x; i<idx_x+radius; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y-radius+1; j<=idx_y; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ur.push_back(i+j*cloud_in->width);
        if(j==idx_y && cloud_in->points[i+j*cloud_in->width].label!=1) gap_r_ctr++;
        if(i==idx_x && cloud_in->points[i+j*cloud_in->width].label!=1) gap_d_ctr++;
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    if(gap_r_ctr==0) gap_r = true;
    if(gap_d_ctr==0) gap_d = true;
    for (int i=idx_x; i<idx_x+radius; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y; j<idx_y+radius; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_lr.push_back(i+j*cloud_in->width);
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    for (int i=idx_x-radius+1; i<=idx_x; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y; j<idx_y+radius; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ll.push_back(i+j*cloud_in->width);
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    return 1;
  }

  bool Segmentation::isStopperInNeighbors(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<int>& indices)
  {
    for(unsigned int i=0; i< indices.size(); i++)
    {
      if(cloud_in->points[indices[i]].label == 1)
        return true;
    }
    return false;
  }


  void Segmentation::propagateWavefront2(pcl::PointCloud<PointLabel>::Ptr& cloud_in)
  {
    //TODO: unlabel small cluster, change pixel step?
    boost::timer t;
    int width = cloud_in->width, height = cloud_in->height;
    std::vector<pcl::Boundary*> wave;
    std::vector<Coords> wave_coords;

    int cur_label = 3;
    int px_range = 2;
    std::vector<int> indices_ul;
    std::vector<int> indices_ur;
    std::vector<int> indices_lr;
    std::vector<int> indices_ll;

    for(int i = 0; i < height; i++ )
    {
      for(int j = 0; j < width; j++ )
      {
        //std::cout << i << "," << j << ":" << cur_label << std::endl;
        if(cloud_in->points[i*width+j].label == 0)
        {
          cur_label++;
          PointLabel* p = &cloud_in->points[i*width+j];
          p->label = cur_label;
          Coords c(j,i);

          //int count = 0;
          bool gap_l=false, gap_r=false, gap_a=false, gap_d=false;
          bool is_wave = true;
          while(is_wave)
          {
            //count++;
            int pt_ctr = c.u+c.v*width;
            if( c.u < width-1 && cloud_in->points[pt_ctr+1].label==0)
            {
              searchForNeighbors (cloud_in,c.u+1,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);
                std::cout << c.u << "," << c.v << " --> " << pt_ctr << std::endl;
                std::cout << "UL: ";
                for(unsigned int i=0; i<indices_ul.size(); i++)
                  std::cout << indices_ul[i] << ", ";
                std::cout << std::endl;
                std::cout << "UR: ";
                for(unsigned int i=0; i<indices_ur.size(); i++)
                  std::cout << indices_ur[i] << ", ";
                std::cout << std::endl;
                std::cout << "LR: ";
                for(unsigned int i=0; i<indices_lr.size(); i++)
                  std::cout << indices_lr[i] << ", ";
                std::cout << std::endl;
                std::cout << "LL: ";
                for(unsigned int i=0; i<indices_ll.size(); i++)
                  std::cout << indices_ll[i] << ", ";
                std::cout << std::endl;
              cloud_in->points[pt_ctr+1].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ul) && isStopperInNeighbors(cloud_in, indices_ll) && gap_l))
              ///if(!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_lr))
                wave_coords.push_back(Coords(c.u+1,c.v));
            }
            if( c.u > 0 && cloud_in->points[pt_ctr-1].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v+1,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr-1].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_lr) && gap_r))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_ul)) &&
              //    (isStopperInNeighbors(cloud_in, indices_lr) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_ul) || !isStopperInNeighbors(cloud_in, indices_ll))
                wave_coords.push_back(Coords(c.u-1,c.v));
            }
            //std::cout << pt_ctr+width << ": "<< (int)(cloud_in->points[pt_ctr+width].boundary_point) << std::endl;
            if(c.v < height-1 && cloud_in->points[pt_ctr+width].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v+1,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr+width].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_ul) && gap_a))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_lr)) &&
              //    (isStopperInNeighbors(cloud_in, indices_ul) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_lr) || !isStopperInNeighbors(cloud_in, indices_ll))
                wave_coords.push_back(Coords(c.u,c.v+1));
            }
            if(c.v > 0 && cloud_in->points[pt_ctr-width].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v-1,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr-width].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_lr) && isStopperInNeighbors(cloud_in, indices_ll) && gap_d))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_lr)) &&
              //    (isStopperInNeighbors(cloud_in, indices_ul) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_ul))
                wave_coords.push_back(Coords(c.u,c.v-1));
            }
            //p = wave.back();
            if(wave_coords.size()>0)
            {
              //wave.pop_back();
              c = wave_coords.back();
              wave_coords.pop_back();
              //std::cout << wave_coords.size() << std::endl;
            }
            else
              is_wave = false;
          }
          /*if(count <= maxSize)
                                {
                                        region_size[cloud_in->points[i*width+j].boundary_point]=1;
                                }
                                else
                                {
                                        region_size[cloud_in->points[i*width+j].boundary_point]=2;
                                }*/
        }
        /*else
                        {
                                if(region_size[cloud_in->points[i*width+j].boundary_point]==1)
                                        cloud_in->points[i*width+j].boundary_point = 0;
                        }*/
      }

    }
    std::cout << "Time elapsed for wavefront propagation: " << t.elapsed() << std::endl;
    return;
  }

  void Segmentation::getClusterIndices(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<pcl::PointIndices>& cluster_indices, cv::Mat& seg_img)
  {
    int max_idx=0;
    int i=0;
    for(i = 0; i < cloud_in->size(); i++ )
    {
      if(cloud_in->points[i].label>max_idx) max_idx=cloud_in->points[i].label;
    }
    for(int k=0; k<=max_idx; k++)
    {
      pcl::PointIndices cluster;
      for(i = 0; i < cloud_in->size(); i++ )
      {
        if(cloud_in->points[i].label==k)
          cluster.indices.push_back(i);
      }
      cluster_indices.push_back(cluster);
    }
    seg_img = cv::Mat(cloud_in->height,cloud_in->width, CV_8UC3);
    std::vector<cv::Vec3b> colorTab;
    for(int i = 0; i < 256; i++ )
    {
      int b = cv::theRNG().uniform(0, 255);
      int g = cv::theRNG().uniform(0, 255);
      int r = cv::theRNG().uniform(0, 255);

      colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
    }
    for(int i = 0; i < seg_img.rows; i++ )
    {
      for(int j = 0; j < seg_img.cols; j++ )
      {
        int label = cloud_in->points[i*cloud_in->width+j].label;
        seg_img.at<cv::Vec3b>(i,j) = colorTab[label];
      }
    }

  }

#include <pcl/io/pcd_io.h>
#include <highgui.h>

int main()
{
  Segmentation seg;
  pcl::PointCloud<PointLabel>::Ptr cloud_in = pcl::PointCloud<PointLabel>::Ptr (new pcl::PointCloud<PointLabel>);
  cloud_in->points.resize(900);
  cloud_in->width = 30;
  cloud_in->height = 30;
  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(i<14 && j==14) cloud_in->points[i*30+j].label = 1;
      if(i>17 && j==14) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  std::vector<pcl::PointIndices> cluster_indices;
  cv::Mat seg_img;
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg",seg_img);
  cv::waitKey();
  return 0;
}

