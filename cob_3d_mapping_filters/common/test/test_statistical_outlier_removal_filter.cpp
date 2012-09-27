/*
 * amp_conf_filter_test.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: goa-wq
 */

//AmplitudeFilter
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <cob_3d_mapping_common/point_types.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/search/search.h>
//#include <pcl/search/organized.h>
//#include <pcl/search/impl/organized.hpp>
//#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <iostream>
#include <fstream>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


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
  pcl::StatisticalOutlierRemoval<PointXYZ> filter;
  pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ> ());
  pcl::PointCloud<PointXYZ>::Ptr cloud_out(new pcl::PointCloud<PointXYZ> ());
  pcl::io::loadPCDFile("/home/goa/Ubuntu One/diss/images/raw/filter_sequence_amplitude2.pcd", *cloud);
  filter.setInputCloud(cloud);
  filter.setStddevMulThresh(0.5);
  filter.setMeanK (50);
  filter.filter(*cloud_out);
  pcl::io::savePCDFileASCII("/home/goa/Ubuntu One/diss/images/raw/filter_sequence_sor2.pcd", *cloud_out);
}

void DoSampleRun2()
{
  pcl::StatisticalOutlierRemoval<PointXYZ> filter(true);
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
  
  //pcl::io::savePCDFileBinary("/tmp/sor_cloud.pcd", *cloud);
  filter.setInputCloud(cloud);
  for(unsigned int k=10; k<=50; k+=10)
  {
  	std::stringstream ss;
  	ss << "/tmp/sor_acc_" << k << ".txt";
    std::ofstream file;
  	file.open(ss.str().c_str());
  	file << "sigma\ttp\tfn\tfp\n";
		for(double c=0.1; c<=2; c+=0.1)
		{
			filter.setStddevMulThresh(c);
			filter.setMeanK (k);
			//if(c==0.5) pcl::io::savePCDFileBinary("/tmp/sor_cloud_filtered.pcd", *cloud_out);
			pcl::IndicesConstPtr ind = filter.getRemovedIndices ();
			std::cout << "Cloud size " << cloud_out->size() << ", ind: " << ind->size() << std::endl;
			int fn_ctr=0, tp_ctr=0;
			for(unsigned int i=0; i<3000; i++)
			{
				bool found=false;
				for(unsigned int j=0; j<ind->size(); j++)
				{
					if(ind->at(j) == i*100) {tp_ctr++;found=true;break;}
				}
				if(!found) fn_ctr++;
			}
			int fp_ctr = ind->size()-tp_ctr;
			double fn_ratio = (double)fn_ctr/3000;
			double fp_ratio = (double)fp_ctr/3000;
			double tp_ratio = (double)tp_ctr/3000;
			file << c <<"\t"<< tp_ratio << "\t" << fn_ratio << "\t" << fp_ratio << "\n";
			std::cout << "c: "<< c << " fn: " << fn_ctr << ", tp: " << tp_ctr << " fp: " << fp_ctr << std::endl;
		}
		file.close();
  }
}


int main()
{
  DoSampleRun2();

}





