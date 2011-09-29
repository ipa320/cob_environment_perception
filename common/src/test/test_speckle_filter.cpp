/*
 * amp_conf_filter_test.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: goa-wq
 */

//AmplitudeFilter
#include <cob_env_model/filters/speckle_filter.h>
#include <cob_env_model/filters/impl/speckle_filter.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cob_env_model/point_types.h>
//#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/timer.hpp>
#include "cob_env_model/stop_watch.h"
#include <iostream>
#include <fstream>

/* Methods for testing filters */

double TestProcessingTime(unsigned int cloud_size)
{
    cob_env_model::SpeckleFilter<PointXYZ> filter;
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
    for(unsigned int i=0; i<1000; i++)
    {
      PrecisionStopWatch sw;
      sw.precisionStart();
      filter.applyFilter(*cloud_out);
      time += sw.precisionStop();
    }
    time /= 1000;
    std::cout << "Cloud size " << cloud_size << ": " << time << " s" << std::endl;
    return time;
}


int main()
{
  std::ofstream file;
  file.open("/home/goa/tmp/speckle_filter_timing.dat");
  file << "#No. of points\ttime (s)\n";
  for(unsigned int cloud_size = 40000; cloud_size <= 400000; cloud_size += 40000)
  {
    double time = TestProcessingTime(cloud_size);
    file << cloud_size << "\t" << time << "\n";
  }
  file.close();

}





