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
 * ROS package name: cob_3d_segmentation
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 07/2012
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

// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <pcl/filters/voxel_grid.h>

// stack includes:
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_segmentation/plane_extraction.h"


typedef pcl::PointXYZRGB PointT;

class PerformanceTester
{
public:
  PerformanceTester()
    : p_vox(new pcl::PointCloud<PointT>)
  {
    voxel.setLeafSize(0.03,0.03,0.03);
    pe.setSaveToFile(false);
    pe.setClusterTolerance(0.06);
    pe.setMinPlaneSize(50);
    pe.setAlpha(0.2);
  }

  void
  computeForOneCloud(pcl::PointCloud<PointT>::ConstPtr cloud)
  {
    PrecisionStopWatch t;
    t.precisionStart();
    voxel.setInputCloud(cloud);
    voxel.filter(*p_vox);
    std::cout << t.precisionStop() << " | ";
    t.precisionStart();
    pe.extractPlanes(p_vox, v_cloud_hull, v_hull_polygons, v_coefficients_plane);
    std::cout << t.precisionStop() << std::endl;
  }

private:
  pcl::PointCloud<PointT>::Ptr p_vox;
  std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<pcl::PointCloud<PointT> > > v_cloud_hull;
  std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
  std::vector<pcl::ModelCoefficients> v_coefficients_plane;

  pcl::VoxelGrid<PointT> voxel;
  PlaneExtraction pe;

};

int main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Please provide a folder containing multiple pcd files" << std::endl;
    return 0;
  }

  PerformanceTester pt;
  pcl::PCDReader r;
  std::vector<std::string> files;
  pcl::getAllPcdFilesInDirectory(argv[1], files);
  std::vector<pcl::PointCloud<PointT>::Ptr> clouds;
  for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); ++it)
  {
    clouds.push_back(pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>));
    r.read( argv[1] + *it, *(clouds.back()) );
  }

  int i = 0;
  while (i < 20)
  {
    std::vector<pcl::PointCloud<PointT>::Ptr>::iterator p = clouds.begin();
    std::cout << "Run #"<< i << std::endl;
    while(p != clouds.end())
    {
      pt.computeForOneCloud(*p);
      ++p;
    }
    ++i;
  }

  return 0;
}
