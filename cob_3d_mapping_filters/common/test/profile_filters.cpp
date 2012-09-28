/*
 * amp_conf_filter_test.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: goa-wq
 */

//Filter
#include <cob_3d_mapping_filters/amplitude_filter.h>
#include <cob_3d_mapping_filters/impl/amplitude_filter.hpp>
#include <cob_3d_mapping_filters/jump_edge_filter.h>
#include <cob_3d_mapping_filters/impl/jump_edge_filter.hpp>
#include <cob_3d_mapping_filters/speckle_filter.h>
#include <cob_3d_mapping_filters/impl/speckle_filter.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cob_3d_mapping_common/point_types.h>
//#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <iostream>
#include <fstream>

/* Methods for testing filters */

double TestProcessingTimeAmp(unsigned int cloud_size, unsigned int iterations)
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

double TestProcessingTimeJe(unsigned int cloud_size, unsigned int iterations)
{
  cob_3d_mapping_filters::JumpEdgeFilter<PointXYZ> filter;
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

double TestProcessingTimeSpk(unsigned int cloud_size, unsigned int iterations)
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

double TestProcessingTimeSOR(unsigned int cloud_size, unsigned int iterations)
{
    pcl::StatisticalOutlierRemoval<PointXYZ> filter;
    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ> ());
    pcl::PointCloud<PointXYZ>::Ptr cloud_out(new pcl::PointCloud<PointXYZ> ());
    filter.setStddevMulThresh(0.5);
    filter.setMeanK (10);

    //cloud->points.resize(cloud_size);
    cloud->width = sqrt(cloud_size);
    cloud->height = sqrt(cloud_size);
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
    for(unsigned int i=0; i<iterations; i++)
    {
      PrecisionStopWatch sw;
      sw.precisionStart();
      filter.filter(*cloud_out);
      time += sw.precisionStop();
    }
    time /= iterations;
    std::cout << "Cloud size " << cloud->size() << ": " << time << " s" << std::endl;
    return time;
}


void TestProcessingTime()
{
  std::ofstream file;
  file.open("/tmp/filter_timings.txt");
  file << "pts\tamp\tje\tspk\tsor\n";
  for(unsigned int cloud_size = 40000; cloud_size <= 400000; cloud_size += 40000)
  {
    double time = TestProcessingTimeAmp(cloud_size,1000);
    file << cloud_size << "\t" << time; 
    time = TestProcessingTimeJe(cloud_size,1000);
    file << "\t" << time;
    time = TestProcessingTimeSpk(cloud_size,1000);
    file << "\t" << time;
    time = TestProcessingTimeSOR(cloud_size,100);
    file << "\t" << time;
    file << "\n";
  }
  file.close();
}



int main()
{
  TestProcessingTime();
}





