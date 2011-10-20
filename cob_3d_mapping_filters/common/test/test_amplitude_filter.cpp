/*
 * amp_conf_filter_test.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: goa-wq
 */

//AmplitudeFilter
#include <cob_3d_mapping_filters/amplitude_filter.h>
#include <cob_3d_mapping_filters/impl/amplitude_filter.hpp>

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
  cob_3d_mapping_filters::AmplitudeFilter<PointXYZA> filter;
    pcl::PointCloud<PointXYZA>::Ptr cloud(new pcl::PointCloud<PointXYZA> ());
    pcl::PointCloud<PointXYZA>::Ptr cloud_out(new pcl::PointCloud<PointXYZA> ());

    cloud->points.resize(cloud_size);
    cloud->width = cloud_size;
    cloud->height = 1;
    for(unsigned int i=0; i<cloud_size; i++)
    {
      PointXYZA pt;
      pt.x = pt.y = pt.z = pt.amplitude = 1;
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
  file.open("/home/goa/tmp/amplitude_filter_timing.dat");
  file << "#No. of points\ttime (s)\n";
  for(unsigned int cloud_size = 40000; cloud_size <= 400000; cloud_size += 40000)
  {
    double time = TestProcessingTimeOnce(cloud_size,1000);
    file << cloud_size << "\t" << time << "\n";
  }
  file.close();
}

void DoSampleRun()
{
  cob_3d_mapping_filters::AmplitudeFilter<PointXYZA> filter;
  pcl::PointCloud<PointXYZA>::Ptr cloud(new pcl::PointCloud<PointXYZA> ());
  pcl::PointCloud<PointXYZA>::Ptr cloud_out(new pcl::PointCloud<PointXYZA> ());
  pcl::io::loadPCDFile("/home/goa/smbhome/Studenten/Hampp, Joshua/results/CamCube/jump_edge/cc_je_input_1318310715.049279022.pcd", *cloud);
  filter.setFilterLimits(0.05,1);
  filter.setInputCloud(cloud);
  filter.filter(*cloud_out);
  pcl::io::savePCDFileASCII("/home/goa/smbhome/Studenten/Hampp, Joshua/results/CamCube/amplitude/amplitude_filtered.pcd", *cloud_out);
}


int main()
{
  TestProcessingTimeOnce(10000,1);
}





