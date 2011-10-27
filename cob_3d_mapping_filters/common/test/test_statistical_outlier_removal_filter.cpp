/*
 * amp_conf_filter_test.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: goa-wq
 */

//AmplitudeFilter
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cob_3d_mapping_common/point_types.h>
//#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <iostream>
#include <fstream>

/* Methods for testing filters */

double TestProcessingTimeOnce(unsigned int width, unsigned int height)
{
    pcl::StatisticalOutlierRemoval<PointXYZ> filter;
    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ> ());
    pcl::PointCloud<PointXYZ>::Ptr cloud_out(new pcl::PointCloud<PointXYZ> ());
    filter.setStddevMulThresh(0.5);
    filter.setMeanK (50);

    cloud->width = width;
    cloud->height = height;
    double x=0, y=0;
    for(unsigned int i=0; i<cloud->width; i++, y+=0.01)
    {
      x=0;
      for(unsigned int j=0; j<cloud->height; j++, x+=0.01)
      {
        PointXYZ pt;
        pt.x = x;
        pt.y = y;
        pt.z = 1;
        cloud->points.push_back(pt);
      }
    }
    //pcl::io::savePCDFileASCII("/home/goa/tmp/filter_pc.pcd", *cloud);
    filter.setInputCloud(cloud);
    //boost::timer t;
    double time=0;
    for(unsigned int i=0; i<100; i++)
    {
      PrecisionStopWatch sw;
      sw.precisionStart();
      filter.filter(*cloud_out);
      time += sw.precisionStop();
    }
    time /= 100;
    std::cout << "Cloud size " << cloud->size() << ": " << time << " s" << std::endl;
    return time;
}

void TestProcessingTime()
{
  std::ofstream file;
  file.open("/home/goa/tmp/statistical_outlier_removal_filter_timing.dat");
  file << "#No. of points\ttime (s)\n";
  file << "0\t0\n";
  unsigned int height = 200;
  for(unsigned int width = 200; width <= 2000; width += 200)
  {
    double time = TestProcessingTimeOnce(width, height);
    file << width*height << "\t" << time << "\n";
  }
  file.close();
}

void DoSampleRun()
{
  pcl::StatisticalOutlierRemoval<PointXYZA> filter;
  pcl::PointCloud<PointXYZA>::Ptr cloud(new pcl::PointCloud<PointXYZA> ());
  pcl::PointCloud<PointXYZA>::Ptr cloud_out(new pcl::PointCloud<PointXYZA> ());
  pcl::io::loadPCDFile("/home/goa/Ubuntu One/diss/images/raw/filter_sequence_amplitude2.pcd", *cloud);
  filter.setInputCloud(cloud);
  filter.setStddevMulThresh(0.5);
  filter.setMeanK (50);
  filter.filter(*cloud_out);
  pcl::io::savePCDFileASCII("/home/goa/Ubuntu One/diss/images/raw/filter_sequence_sor2.pcd", *cloud_out);
}


int main()
{
  DoSampleRun();

}





