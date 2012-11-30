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
#include <cob_3d_mapping_filters/speckle_filter.h>
#include <cob_3d_mapping_filters/impl/speckle_filter.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cob_3d_mapping_common/point_types.h>
//#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <iostream>
#include <fstream>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

/* Methods for testing filters */

double TestProcessingTimeOnce(unsigned int cloud_size, unsigned int iterations)
{
  cob_3d_mapping_filters::SpeckleFilter<PointXYZ> filter;
    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ> ());
    pcl::PointCloud<PointXYZ>::Ptr cloud_out(new pcl::PointCloud<PointXYZ> ());

    cloud->points.resize(cloud_size);
    cloud->width = sqrt(cloud_size);
    cloud->height = sqrt(cloud_size);
    for(unsigned int i=0; i<cloud_size; i++)
    {
      PointXYZ pt;
      pt.x = pt.y = pt.z = 1;
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
  file.open("/home/goa/tmp/speckle_filter_timing.dat");
  file << "#No. of points\ttime (s)\n";
  for(unsigned int cloud_size = 40000; cloud_size <= 400000; cloud_size += 40000)
  {
    double time = TestProcessingTimeOnce(cloud_size, 1000);
    file << cloud_size << "\t" << time << "\n";
  }
  file.close();

}

void DoSampleRun()
{
  cob_3d_mapping_filters::SpeckleFilter<PointXYZA> filter;
  pcl::PointCloud<PointXYZA>::Ptr cloud(new pcl::PointCloud<PointXYZA> ());
  pcl::PointCloud<PointXYZA>::Ptr cloud_out(new pcl::PointCloud<PointXYZA> ());
  pcl::io::loadPCDFile("/home/goa/Ubuntu One/diss/images/raw/filter_sequence_amplitude2.pcd", *cloud);
  filter.setInputCloud(cloud);
  filter.filter(*cloud_out);
  pcl::io::savePCDFileASCII("/home/goa/Ubuntu One/diss/images/raw/filter_sequence_speckle2.pcd", *cloud_out);
}

void DoSampleRun2()
{
  cob_3d_mapping_filters::SpeckleFilter<PointXYZ> filter;
  pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ> ());
  pcl::PointCloud<PointXYZ>::Ptr cloud_out(new pcl::PointCloud<PointXYZ> ());
  cloud->width = 640;
  cloud->height = 480;
  double x=0, y=0;
  for(unsigned int i=0; i<cloud->width; i++, y+=0.001)
  {
    x=0;
    for(unsigned int j=0; j<cloud->height; j++, x+=0.001)
    {
      PointXYZ pt;
      pt.x = x;
      pt.y = y;
      pt.z = 1;
      cloud->points.push_back(pt);
    }
  }
  boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)
  boost::normal_distribution<> nd(0.0, 0.05);
  boost::variate_generator<boost::mt19937&, 
                           boost::normal_distribution<> > var_nor(rng, nd);

  for(unsigned int i=0; i<3000; i++)
  	cloud->points[i*100].z += var_nor();
  
  //pcl::io::savePCDFileASCII("/tmp/spk_cloud.pcd", *cloud);
  filter.setInputCloud(cloud);
  for(unsigned int s=10; s<=100; s+=10)
  {
    std::stringstream ss;
  	ss << "/tmp/spk_acc_" << s << ".txt";
    std::ofstream file;
  	file.open(ss.str().c_str());
  	file << "thr\ttp\tfn\tfp\n";
		for(double c=0.01; c<=0.1; c+=0.01)
		{
			filter.setFilterParam (s, c);
			filter.filter(*cloud_out);
			pcl::PointIndices::Ptr ind = filter.getRemovedIndices ();
			std::cout << "Cloud size " << cloud_out->size() << ", ind: " << ind->indices.size() << std::endl;
			int fn_ctr=0, tp_ctr=0;
			for(unsigned int i=0; i<3000; i++)
			{
				bool found=false;
				for(unsigned int j=0; j<ind->indices.size(); j++)
				{
					if(ind->indices[j] == i*100) {tp_ctr++;found=true;break;}
				}
				if(!found) fn_ctr++;
			}
			int fp_ctr = ind->indices.size()-tp_ctr;
			double fn_ratio = (double)fn_ctr/3000;
			double fp_ratio = (double)fp_ctr/3000;
			double tp_ratio = (double)tp_ctr/3000;
			file << c <<"\t"<< tp_ratio << "\t" << fn_ratio << "\t" << fp_ratio << "\n";
			std::cout << "c: "<< c << " fn: " << fn_ratio << ", tp: " << tp_ratio << " fp: " << fp_ratio << std::endl;
		}
		file.close();
  }
}

int main()
{
  DoSampleRun2();
  //TestProcessingTimeOnce(10000, 1);
}



