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

//AmplitudeFilter
#include <cob_3d_mapping_filters/intensity_filter.h>
//#include <cob_3d_mapping_filters/impl/intensity_filter.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cob_3d_mapping_common/point_types.h>
//#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <iostream>
#include <fstream>

/* Methods for testing filters */

double TestProcessingTimeOnce(unsigned int cloud_size, unsigned int iterations)
{
  cob_3d_mapping_filters::IntensityFilter<PointXYZI> filter;
    pcl::PointCloud<PointXYZI>::Ptr cloud(new pcl::PointCloud<PointXYZI> ());
    pcl::PointCloud<PointXYZI>::Ptr cloud_out(new pcl::PointCloud<PointXYZI> ());

    cloud->points.resize(cloud_size);
    cloud->width = cloud_size;
    cloud->height = 1;
    for(unsigned int i=0; i<cloud_size; i++)
    {
      PointXYZI pt;
      pt.x = pt.y = pt.z = pt.intensity = 1;
      cloud->points[i] = pt;
    }
    filter.setInputCloud(cloud);
    //boost::timer t;
    double time=0;
    for(unsigned int i=0; i<iterations; i++)
    {
      PrecisionStopWatch sw;
      sw.precisionStart();
      filter.filter(*cloud_out);
      time += sw.precisionStop();
    }
    time /= iterations;
    std::cout << "Cloud size " << cloud_size << ": " << time << " s" << std::endl;
    return time;
}

void TestProcessingTime()
{
  std::ofstream file;
  file.open("/home/goa/tmp/intensity_filter_timing.dat");
  file << "#No. of points\ttime (s)\n";
  for(unsigned int cloud_size = 40000; cloud_size <= 400000; cloud_size += 40000)
  {
    double time = TestProcessingTimeOnce(cloud_size,1000);
    file << cloud_size << "\t" << time << "\n";
  }
  file.close();
}


int main()
{
  TestProcessingTimeOnce(10000,1);
}





