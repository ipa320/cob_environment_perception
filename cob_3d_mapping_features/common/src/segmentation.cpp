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
#include <cob_3d_mapping_common/label_defines.h>
#include "cob_3d_mapping_features/segmentation.h"

using namespace cob_3d_mapping_features;

int Segmentation::searchForNeighbors (
  pcl::PointCloud<PointLabel>::Ptr& cloud_in,
  int col, int row,
  double radius,
  std::vector<int>& indices_ul,
  std::vector<int>& indices_ur,
  std::vector<int>& indices_lr,
  std::vector<int>& indices_ll,
  bool& gap_l, bool& gap_r, bool& gap_a, bool& gap_d)
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
  for (int i=idx_x-radius+1; i<idx_x; i++)
  {
    if(i>=cloud_in->width) break;
    if(i<0) continue;
    for (int j=idx_y-radius+1; j<idx_y; j++)
    {
      if(j<0) continue;
      if(i==idx_x && j==idx_y) continue; //skip p itself
      if(j>=cloud_in->height) break;
      indices_ul.push_back(i+j*cloud_in->width);
      //if(j==idx_y && cloud_in->points[i+j*cloud_in->width].label==1) gap_l_ctr++;
      //if(i==idx_x && cloud_in->points[i+j*cloud_in->width].label>=1) gap_a_ctr++;
      //std::cout << i << "," << j<<  ":" << gap_a_ctr << std::endl;
      //std::cout << "(" << i << "," << j << ")";
    }
  }
  //if(gap_l_ctr==0) gap_l = true;
  //if(gap_a_ctr==0) gap_a = true;

  int gap_r_ctr = 0;
  int gap_d_ctr = 0;
  for (int i=idx_x+1; i<idx_x+radius; i++)
  {
    if(i>=cloud_in->width) break;
    if(i<0) continue;
    for (int j=idx_y-radius+1; j<idx_y; j++)
    {
      if(j<0) continue;
      if(i==idx_x && j==idx_y) continue; //skip p itself
      if(j>=cloud_in->height) break;
      indices_ur.push_back(i+j*cloud_in->width);
      //if(j==idx_y && cloud_in->points[i+j*cloud_in->width].label==1) gap_r_ctr++;
      //if(i==idx_x && cloud_in->points[i+j*cloud_in->width].label>=1) gap_d_ctr++;
      //std::cout << "(" << i << "," << j << ")";
    }
  }
  //if(gap_r_ctr==0) gap_r = true;
  //if(gap_d_ctr==0) gap_d = true;
  for (int i=idx_x+1; i<idx_x+radius; i++)
  {
    if(i>=cloud_in->width) break;
    if(i<0) continue;
    for (int j=idx_y+1; j<idx_y+radius; j++)
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
    for (int j=idx_y+1; j<idx_y+radius; j++)
    {
      if(j<0) continue;
      if(i==idx_x && j==idx_y) continue; //skip p itself
      if(j>=cloud_in->height) break;
      indices_ll.push_back(i+j*cloud_in->width);
      //std::cout << "(" << i << "," << j << ")";
    }
  }
  for (int i=idx_x-radius+1; i<idx_x+radius; i++)
  {
    if(i<0) continue;
    if(i>=cloud_in->width) break;
    if(i<idx_x && cloud_in->points[i+idx_y*cloud_in->width].label>=1) gap_l_ctr++;
    if(i>idx_x && cloud_in->points[i+idx_y*cloud_in->width].label>=1) gap_r_ctr++;
  }
  for (int j=idx_y-radius+1; j<idx_y+radius; j++)
  {
    if(j<0) continue;
    if(j>=cloud_in->height) break;
    if(j<idx_y && cloud_in->points[idx_x+j*cloud_in->width].label>=1) gap_a_ctr++;
    if(j>idx_y && cloud_in->points[idx_x+j*cloud_in->width].label>=1) gap_d_ctr++;
  }
  if(gap_r_ctr==0) gap_r = true;
  if(gap_d_ctr==0) gap_d = true;
  if(gap_l_ctr==0) gap_l = true;
  if(gap_a_ctr==0) gap_a = true;
  return 1;
}

bool Segmentation::isStopperInNeighbors(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<int>& indices)
{
  for(unsigned int i=0; i< indices.size(); i++)
  {
    if(cloud_in->points[indices[i]].label > 0 && cloud_in->points[indices[i]].label <= 3)
      return true;
  }
  return false;
}

/*bool Segmentation::isEdgeInNeighbors(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<int>& indices)
{
  int edge_ctr=0;
  for(unsigned int i=0; i< indices.size(); i++)
  {
    if(cloud_in->points[indices[i]].label == 1)
      edge_ctr++;
  }
  if(edge_ctr==indices.size())
    return true;
  return false;
}*/


void Segmentation::propagateWavefront2(pcl::PointCloud<PointLabel>::Ptr& cloud_in)
{
  //TODO: unlabel small cluster, change pixel step?
  int cancel=0;
  boost::timer t;
  int width = cloud_in->width, height = cloud_in->height;
  std::vector<pcl::Boundary*> wave;
  std::vector<Coords> wave_coords;

  int cur_label = 4;
  int px_range = 8;
  std::vector<int> indices_ul;
  std::vector<int> indices_ur;
  std::vector<int> indices_lr;
  std::vector<int> indices_ll;


  for(int i = 0; i < height; i++ )
  {
    for(int j = 0; j < width; j++ )
    {
      //std::cout << i << "," << j << ":" << cloud_in->points[i*width+j].label << std::endl;
      if(cloud_in->points[i*width+j].label==I_UNDEF)
      {
	cur_label++;
	PointLabel* p = &cloud_in->points[i*width+j];
	p->label = cur_label;
	Coords c(j,i,false);
	//int count = 0;
	bool gap_l=false, gap_r=false, gap_a=false, gap_d=false;
	bool is_wave = true;
	bool narrow_wave = false;
	int wave_ctr=0;
	while(is_wave)
	{
	  std::vector<Coords> wave_coords_cands;
	  bool stop = false;
	  //count++;
	  int pt_ctr = c.u+c.v*width;
	  // check the point right of current label:
	  if( c.u < width-1 && cloud_in->points[pt_ctr+1].label==I_UNDEF)
	  {
	    searchForNeighbors (cloud_in,c.u+1,c.v,px_range,
				indices_ul, indices_ur,
				indices_lr, indices_ll,
				gap_l, gap_r, gap_a, gap_d);
	    cloud_in->points[pt_ctr+1].label = cur_label;
	    /*if(cancel==3)
              {
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
	      std::cout << "Stopper ul:" << isStopperInNeighbors(cloud_in, indices_ul) << std::endl;
	      std::cout << "Stopper ll:" << isStopperInNeighbors(cloud_in, indices_ll) << std::endl;
	      std::cout << "Gap l:" << gap_l << std::endl;
              }*/
	    if(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_lr)) c.c_gap = true;
	    //if(!(isStopperInNeighbors(cloud_in, indices_ul) && isStopperInNeighbors(cloud_in, indices_ll) && (!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_lr))/*&& gap_l*/))
	    // has no gap behind or is in/ close before gap
	    if(!(isStopperInNeighbors(cloud_in, indices_ul) && isStopperInNeighbors(cloud_in, indices_ll)) || (isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_lr)))
              ///if(!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_lr))
	        wave_coords_cands.push_back(Coords(c.u+1,c.v,c.c_gap));
	    else
	    {
	      if(!c.c_gap)
	        wave_coords_cands.push_back(Coords(c.u+1,c.v,c.c_gap));
	      else
	      {
                stop=true;
                //if(cancel==3) std::cout << "R Stopper: " << c.u+1 << "," << c.v << std::endl;
	      }
	    }
	    c.c_gap=false;
	  }
	  if( c.u > 0 && cloud_in->points[pt_ctr-1].label==I_UNDEF)
	  {
	    searchForNeighbors (cloud_in,c.u-1,c.v,px_range,
				indices_ul, indices_ur,
				indices_lr, indices_ll,
				gap_l, gap_r, gap_a, gap_d);

	    cloud_in->points[pt_ctr-1].label = cur_label;
	    if(isStopperInNeighbors(cloud_in, indices_ul) && isStopperInNeighbors(cloud_in, indices_ll)) c.c_gap = true;
	    if(!(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_lr)) || (isStopperInNeighbors(cloud_in, indices_ul) && isStopperInNeighbors(cloud_in, indices_ll)))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_ul)) &&
              //    (isStopperInNeighbors(cloud_in, indices_lr) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_ul) || !isStopperInNeighbors(cloud_in, indices_ll))
	      wave_coords_cands.push_back(Coords(c.u-1,c.v, c.c_gap));
	    else
	    {
	      if(!c.c_gap)
	        wave_coords_cands.push_back(Coords(c.u-1,c.v,c.c_gap));
	      else
	      {
	        stop=true;
	        //if(cancel==3) std::cout << "U Stopper: " << c.u+1 << "," << c.v << std::endl;
	      }
	    }
	    c.c_gap=false;
	  }
	  //std::cout << pt_ctr+width << ": "<< (int)(cloud_in->points[pt_ctr+width].boundary_point) << std::endl;
	  if(c.v < height-1 && cloud_in->points[pt_ctr+width].label==I_UNDEF)
	  {
	    searchForNeighbors (cloud_in,c.u,c.v+1,px_range,
				indices_ul, indices_ur,
				indices_lr, indices_ll,
				gap_l, gap_r, gap_a, gap_d);

	    cloud_in->points[pt_ctr+width].label = cur_label;
	    if(isStopperInNeighbors(cloud_in, indices_lr) && isStopperInNeighbors(cloud_in, indices_ll)) c.c_gap = true;
	    if(!(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_ul)) || (isStopperInNeighbors(cloud_in, indices_lr) && isStopperInNeighbors(cloud_in, indices_ll)))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_lr)) &&
              //    (isStopperInNeighbors(cloud_in, indices_ul) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_lr) || !isStopperInNeighbors(cloud_in, indices_ll))
	      wave_coords_cands.push_back(Coords(c.u,c.v+1,c.c_gap));
	    else
	    {
	      if(!c.c_gap)
	        wave_coords_cands.push_back(Coords(c.u,c.v+1,c.c_gap));
	      else
	      {
	        stop=true;
		/*
	        if(cancel==3)
	        {
	          std::cout << "Stopper ur:" << isStopperInNeighbors(cloud_in, indices_ul) << std::endl;
	          std::cout << "Stopper ul:" << isStopperInNeighbors(cloud_in, indices_ll) << std::endl;
	          std::cout << "Gap a:" << gap_l << std::endl;
	        }
	        if(cancel==3) std::cout << "L Stopper: " << c.u+1 << "," << c.v << std::endl;
		*/
	      }
	    }
	  }
	  if(c.v > 0 && cloud_in->points[pt_ctr-width].label==I_UNDEF)
	  {
	    searchForNeighbors (cloud_in,c.u,c.v-1,px_range,
				indices_ul, indices_ur,
				indices_lr, indices_ll,
				gap_l, gap_r, gap_a, gap_d);

	    cloud_in->points[pt_ctr-width].label = cur_label;
	    if(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_ul)) c.c_gap = true;
	    if(!(isStopperInNeighbors(cloud_in, indices_lr) && isStopperInNeighbors(cloud_in, indices_ll)) || (isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_ul)))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_lr)) &&
              //    (isStopperInNeighbors(cloud_in, indices_ul) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_ul))
	      wave_coords_cands.push_back(Coords(c.u,c.v-1,c.c_gap));
	    else
	    {
	      if(!c.c_gap)
	        wave_coords_cands.push_back(Coords(c.u,c.v-1,c.c_gap));
	      else
	      {
	        stop=true;
	        //if(cancel==3) std::cout << "U Stopper: " << c.u+1 << "," << c.v << std::endl;
	      }
	    }
	  }
	  if(!stop)
	  {
	    for (unsigned int c=0; c<wave_coords_cands.size(); c++)
	    {
	      wave_coords.push_back(wave_coords_cands[c]);
	      wave_ctr++;
	    }
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
	  {
	    is_wave = false;
	    cancel++;
	    //if(cancel>3) return;
	  }
	}
	//Better: merge with neighbouring segment
	/*if(wave_ctr <10)
	{
	  cur_label--;
	}*/
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
  //std::cout << "Time elapsed for wavefront propagation: " << t.elapsed() << std::endl;
  return;
}

void Segmentation::getClusterIndices(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<pcl::PointIndices>& cluster_indices, cv::Mat& seg_img)
{
  int max_idx=0;
  int i=0;
  cluster_indices.clear();
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

  for(int i = 0; i < seg_img.rows; i++ )
  {
    for(int j = 0; j < seg_img.cols; j++ )
    {
      int label = cloud_in->points[i*cloud_in->width+j].label;
      seg_img.at<cv::Vec3b>(i,j) = color_tab_[label];
    }
  }
}

void Segmentation::getClusterIndices(pcl::PointCloud<PointLabel>::Ptr& cloud_in, 
				     std::vector<pcl::PointIndices>& cluster_indices, 
				     pcl::PointCloud<PointXYZRGB>::Ptr& colored_cloud)
{
  int max_idx=0;
  int i=0;
  cluster_indices.clear();
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
  for(int i = 0; i < cloud_in->size(); i++ )
  {
    int label = cloud_in->points[i].label;
    uint32_t rgb = (color_tab_[label])[2] << 16 | (color_tab_[label])[1] << 8 | (color_tab_[label])[0];
    colored_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }
}
