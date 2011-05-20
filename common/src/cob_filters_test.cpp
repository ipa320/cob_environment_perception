/*
 * amp_conf_filter_test.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: goa-wq
 */

//AmplitudeFilter
#include <cob_env_model/filters/amplitude_filter.h>
#include <cob_env_model/filters/impl/amplitude_filter.hpp>

//ConfidenceFilter
#include <cob_env_model/filters/confidence_filter.h>
#include <cob_env_model/filters/impl/confidence_filter.hpp>

//JumpEdgeFilter
#include <cob_env_model/filters/jump_edge_filter.h>
#include <cob_env_model/filters/impl/jump_edge_filter.hpp>

//SpeckleFilter
#include <cob_env_model/filters/speckle_filter.h>
#include <cob_env_model/filters/impl/speckle_filter.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cob_env_model/cpc_point.h>
//#include <sensor_msgs/point_cloud_conversion.h>

/* Methods for testing filters */
void AmplitudeConfidenceFilter();
void AmplitudeFilter();
void ConfidenceFilter();
void JumpEdgeFilter();
void SpeckleFilter();


int main()
{
	//Amplitude Test
	AmplitudeFilter();

	//ConfidenceFilter Test
	//ConfidenceFilter();

	//JumpEdgeFilter Test
	//JumpEdgeFilter();

}

void AmplitudeFilter()
{
    cob_env_model::AmplitudeFilter<CPCPoint> filter;
    pcl::PointCloud<CPCPoint>::Ptr cloud(new pcl::PointCloud<CPCPoint> ());
    pcl::PointCloud<CPCPoint>::Ptr cloud_filtered(new pcl::PointCloud<CPCPoint> ());
    //pcl::PointCloud<CPCPoint>::Ptr cloud_filtered1(new pcl::PointCloud<CPCPoint> ());
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

void ConfidenceFilter()
{
	cob_env_model::ConfidenceFilter<CPCPoint> filter;
	pcl::PointCloud<CPCPoint>::Ptr cloud(new pcl::PointCloud<CPCPoint> ());
	pcl::PointCloud<CPCPoint>::Ptr cloud_filtered(new pcl::PointCloud<CPCPoint> ());
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
	cob_env_model::JumpEdgeFilter<CPCPoint> filter;
	pcl::PointCloud<CPCPoint>::Ptr cloud(new pcl::PointCloud<CPCPoint> ());
	pcl::PointCloud<CPCPoint>::Ptr cloud_filtered(new pcl::PointCloud<CPCPoint> ());
	pcl::PCDReader reader;
	reader.read ("/home/goa-wq/no_move_table.pcd", *cloud);
	std::cout << "size: " << cloud->points.size() << std::endl;
	filter.setInputCloud(cloud);
	std::cout << "  preparing to enter apply_filter method "  << std::endl;
	filter.applyFilter(*cloud_filtered);
	std::cout << "size: " << cloud_filtered->points.size() << std::endl;
	pcl::io::savePCDFile("/home/goa-wq/no_move_table_filtered.pcd",*cloud_filtered);

}


