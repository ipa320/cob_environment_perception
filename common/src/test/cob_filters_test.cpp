/*
 * amp_conf_filter_test.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: goa-wq
 */

//AmplitudeFilter
#include <cob_env_model/filters/amplitude_filter.h>
#include <cob_env_model/filters/impl/amplitude_filter.hpp>

//IntensityFilter
#include <cob_env_model/filters/intensity_filter.h>
#include <cob_env_model/filters/impl/intensity_filter.hpp>

//JumpEdgeFilter
#include <cob_env_model/filters/jump_edge_filter.h>
#include <cob_env_model/filters/impl/jump_edge_filter.hpp>

//SpeckleFilter
#include <cob_env_model/filters/speckle_filter.h>
#include <cob_env_model/filters/impl/speckle_filter.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cob_env_model/point_types.h>
//#include <sensor_msgs/point_cloud_conversion.h>

/* Methods for testing filters */
void AmplitudeIntensityFilter();
void AmplitudeFilter();
void IntensityFilter();
void JumpEdgeFilter();
void SpeckleFilter();


int main()
{
	//Amplitude Test
	AmplitudeFilter();

	//IntensityFilter Test
	//IntensityFilter();

	//JumpEdgeFilter Test
	//JumpEdgeFilter();

}

void AmplitudeFilter()
{
    cob_env_model::AmplitudeFilter<PointXYZA> filter;
    pcl::PointCloud<PointXYZA>::Ptr cloud(new pcl::PointCloud<PointXYZA> ());
    pcl::PointCloud<PointXYZA>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZA> ());
    //pcl::PointCloud<PointXYZCI>::Ptr cloud_filtered1(new pcl::PointCloud<PointXYZCI> ());
    pcl::PCDReader reader;
	reader.read ("/home/goa-wq/no_move_table.pcd", *cloud);
	std::cout << "size: " << cloud->points.size() << std::endl;
	filter.setInputCloud(cloud);
	std::cout << "  preparing to enter apply_filter method "  << std::endl;
	filter.applyFilter(*cloud_filtered);
	std::cout << "size: " << cloud_filtered->points.size() << std::endl;
	pcl::io::savePCDFile("/home/goa-wq/no_move_table_filtered.pcd",*cloud_filtered);
	std::cout << "  preparing to enter negative_apply_filter method "  << std::endl;
	filter.negativeApplyFilter(*cloud_filtered);
	std::cout << "size: " << cloud_filtered->points.size() << std::endl;
	pcl::io::savePCDFile("/home/goa-wq/no_move_table_negativefiltered.pcd",*cloud_filtered);
}

void IntensityFilter()
{
	cob_env_model::IntensityFilter<PointXYZCI> filter;
	pcl::PointCloud<PointXYZCI>::Ptr cloud(new pcl::PointCloud<PointXYZCI> ());
	pcl::PointCloud<PointXYZCI>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZCI> ());
	pcl::PCDReader reader;
	reader.read ("/home/goa-wq/no_move_table.pcd", *cloud);
	std::cout << "size: " << cloud->points.size() << std::endl;
	filter.setInputCloud(cloud);
	std::cout << "  preparing to enter apply_filter method "  << std::endl;
	filter.applyFilter(*cloud_filtered);
	std::cout << "size: " << cloud_filtered->points.size() << std::endl;
	pcl::io::savePCDFile("/home/goa-wq/no_move_table_filtered.pcd",*cloud_filtered);
	std::cout << "  preparing to enter negative_apply_filter method "  << std::endl;
	filter.negativeApplyFilter(*cloud_filtered);
	std::cout << "size: " << cloud_filtered->points.size() << std::endl;
	pcl::io::savePCDFile("/home/goa-wq/no_move_table_negativefiltered.pcd",*cloud_filtered);
}

void JumpEdgeFilter()
{
	cob_env_model::JumpEdgeFilter<PointXYZCI> filter;
	pcl::PointCloud<PointXYZCI>::Ptr cloud(new pcl::PointCloud<PointXYZCI> ());
	pcl::PointCloud<PointXYZCI>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZCI> ());
	pcl::PCDReader reader;
	reader.read ("/home/goa-wq/no_move_table.pcd", *cloud);
	std::cout << "size: " << cloud->points.size() << std::endl;
	filter.setInputCloud(cloud);
	std::cout << "  preparing to enter apply_filter method "  << std::endl;
	filter.applyFilter(*cloud_filtered);
	std::cout << "size: " << cloud_filtered->points.size() << std::endl;
	pcl::io::savePCDFile("/home/goa-wq/no_move_table_filtered.pcd",*cloud_filtered);

}


