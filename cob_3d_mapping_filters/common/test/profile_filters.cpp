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
/*
 * amp_conf_filter_test.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: goa-wq
 */

//Filter
//#include <cob_3d_mapping_filters/amplitude_filter.h>
//#include <cob_3d_mapping_filters/impl/amplitude_filter.hpp>
#include <cob_3d_mapping_filters/jump_edge_filter.h>
//#include <cob_3d_mapping_filters/impl/jump_edge_filter.hpp>
#include <cob_3d_mapping_filters/speckle_filter.h>
//#include <cob_3d_mapping_filters/impl/speckle_filter.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/impl/statistical_outlier_removal.hpp>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cob_3d_mapping_common/point_types.h>
//#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <iostream>
#include <fstream>

/* Methods for testing filters */

/*double
TestProcessingTimeAmp (unsigned int cloud_size, unsigned int iterations)
{
  cob_3d_mapping_filters::AmplitudeFilter<PointXYZA> filter;
  pcl::PointCloud<PointXYZA>::Ptr cloud (new pcl::PointCloud<PointXYZA> ());
  pcl::PointCloud<PointXYZA>::Ptr cloud_out (new pcl::PointCloud<PointXYZA> ());

  cloud->points.resize (cloud_size);
  cloud->width = cloud_size;
  cloud->height = 1;
  for (unsigned int i = 0; i < cloud_size; i++)
  {
    PointXYZA pt;
    pt.x = pt.y = pt.z = pt.amplitude = 1;
    cloud->points[i] = pt;
  }
  filter.setInputCloud (cloud);
  //boost::timer t;
  double time = 0;
  for (unsigned int i = 0; i < iterations; i++)
  {
    PrecisionStopWatch sw;
    sw.precisionStart ();
    filter.filter (*cloud_out);
    time += sw.precisionStop ();
  }
  time /= iterations;
  std::cout << "Cloud size " << cloud_size << ": " << time << " s" << std::endl;
  return time;
}*/

double
TestProcessingTimeJe (unsigned int cloud_size, unsigned int iterations)
{
  cob_3d_mapping_filters::JumpEdgeFilter<PointXYZ> filter;
  pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ> ());
  pcl::PointCloud<PointXYZ>::Ptr cloud_out (new pcl::PointCloud<PointXYZ> ());

  cloud->points.resize (cloud_size);
  cloud->width = std::sqrt (cloud_size);
  cloud->height = std::sqrt (cloud_size);
  for (unsigned int i = 0; i < cloud_size; i++)
  {
    PointXYZ pt;
    pt.x = pt.y = pt.z = 1;
    cloud->points[i] = pt;
  }
  filter.setInputCloud (cloud);
  //boost::timer t;
  double time = 0;
  for (unsigned int i = 0; i < iterations; i++)
  {
    PrecisionStopWatch sw;
    sw.precisionStart ();
    filter.filter (*cloud_out);
    time += sw.precisionStop ();
  }
  time /= iterations;
  std::cout << "Cloud size " << cloud_size << ": " << time << " s" << std::endl;
  return time;
}

double
TestProcessingTimeSpk (unsigned int cloud_size, unsigned int iterations)
{
  cob_3d_mapping_filters::SpeckleFilter<PointXYZ> filter;
  pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ> ());
  pcl::PointCloud<PointXYZ>::Ptr cloud_out (new pcl::PointCloud<PointXYZ> ());

  cloud->points.resize (cloud_size);
  cloud->width = std::sqrt (cloud_size);
  cloud->height = std::sqrt (cloud_size);
  for (unsigned int i = 0; i < cloud_size; i++)
  {
    PointXYZ pt;
    pt.x = pt.y = pt.z = 1;
    cloud->points[i] = pt;
  }
  filter.setInputCloud (cloud);
  //boost::timer t;
  double time = 0;
  for (unsigned int i = 0; i < iterations; i++)
  {
    PrecisionStopWatch sw;
    sw.precisionStart ();
    filter.filter (*cloud_out);
    time += sw.precisionStop ();
  }
  time /= iterations;
  std::cout << "Cloud size " << cloud_size << ": " << time << " s" << std::endl;
  return time;
}

double
TestProcessingTimeSOR (unsigned int cloud_size, unsigned int iterations)
{
  pcl::StatisticalOutlierRemoval<PointXYZ> filter;
  pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ> ());
  pcl::PointCloud<PointXYZ>::Ptr cloud_out (new pcl::PointCloud<PointXYZ> ());
  filter.setStddevMulThresh (0.5);
  filter.setMeanK (10);

  //cloud->points.resize(cloud_size);
  cloud->width = std::sqrt (cloud_size);
  cloud->height = std::sqrt (cloud_size);
  double x = 0, y = 0;
  for (unsigned int i = 0; i < cloud->width; i++, y += 0.01)
  {
    x = 0;
    for (unsigned int j = 0; j < cloud->height; j++, x += 0.01)
    {
      PointXYZ pt;
      pt.x = x;
      pt.y = y;
      pt.z = 1;
      cloud->points.push_back (pt);
    }
  }
  //pcl::io::savePCDFileASCII("/home/goa/tmp/filter_pc.pcd", *cloud);
  filter.setInputCloud (cloud);
  //boost::timer t;
  double time = 0;
  for (unsigned int i = 0; i < iterations; i++)
  {
    PrecisionStopWatch sw;
    sw.precisionStart ();
    filter.filter (*cloud_out);
    time += sw.precisionStop ();
  }
  time /= iterations;
  std::cout << "Cloud size " << cloud->size () << ": " << time << " s" << std::endl;
  return time;
}

void
TestProcessingTime ()
{
  std::ofstream file;
  file.open ("/tmp/filter_timings.txt");
  file << "pts\tamp\tje\tspk\tsor\n";
  for (unsigned int cloud_size = 40000; cloud_size <= 400000; cloud_size += 40000)
  {
    //double time = TestProcessingTimeAmp (cloud_size, 1000);
    //file << cloud_size << "\t" << time;
    double time = TestProcessingTimeJe (cloud_size, 1000);
    file << "\t" << time;
    time = TestProcessingTimeSpk (cloud_size, 1000);
    file << "\t" << time;
    time = TestProcessingTimeSOR (cloud_size, 100);
    file << "\t" << time;
    file << "\n";
  }
  file.close ();
}

int
main ()
{
  TestProcessingTime ();
}

