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
 * Date of creation: 05/2012
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

// stack includes:
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_features/organized_normal_estimation_omp.h"
#include "cob_3d_segmentation/depth_segmentation.h"
#include "cob_3d_segmentation/cluster_classifier.h"

using namespace pcl;

typedef cob_3d_segmentation::PredefinedSegmentationTypes ST;

class PerformanceTester
{
public:
  PerformanceTester()
    : l(new PointCloud<PointLabel>)
    , n(new PointCloud<Normal>)
    , g(new ST::Graph)
  {
    one.setOutputLabels(l);
    one.setPixelSearchRadius(8,2,2); //radius,pixel,circle
    one.setSkipDistantPointThreshold(6.0);    
    seg.setNormalCloudIn(n);
    seg.setLabelCloudInOut(l);
    seg.setClusterGraphOut(g);
  }
  
  void 
  computeForOneCloud(PointCloud<PointXYZRGB>::ConstPtr cloud)
  {
    PrecisionStopWatch t;
    t.precisionStart();
    one.setInputCloud(cloud);
    one.compute(*n);
    std::cout << t.precisionStop() << " | ";
    t.precisionStart();
    seg.setPointCloudIn(cloud);
    seg.performInitialSegmentation();
    seg.refineSegmentation();    
    std::cout << t.precisionStop() << std::endl;
  }

private:
  PointCloud<PointLabel>::Ptr l;
  PointCloud<Normal>::Ptr n;
  ST::Graph::Ptr g;
  cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> seg;
  cob_3d_mapping_features::OrganizedNormalEstimationOMP<PointXYZRGB, Normal, PointLabel> one;

};

int main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Please provide a folder containing multiple pcd files" << std::endl;
    return 0;
  }

  PerformanceTester pt;
  PCDReader r;
  std::vector<std::string> files;
  getAllPcdFilesInDirectory(argv[1], files);
  std::vector<PointCloud<PointXYZRGB>::Ptr> clouds;
  for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); ++it)
  {
    clouds.push_back(PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>));
    r.read( argv[1] + *it, *(clouds.back()) );
  }

  int i = 0;
  while (i < 20)
  {
    std::vector<PointCloud<PointXYZRGB>::Ptr>::iterator p = clouds.begin();
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
