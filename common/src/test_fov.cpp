/*
 * test_fov.cpp
 *
 *  Created on: 28.02.2011
 *      Author: goa
 */

#include <cob_env_model/field_of_view_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <cob_env_model/cpc_point.h>
#include <tf/transform_listener.h>

#include <boost/timer.hpp>

int main()
{
	ipa_env_model::FieldOfViewSegmentation<pcl::PointXYZ> seg;
	Eigen::Vector3d n_up;
	Eigen::Vector3d n_down;
	Eigen::Vector3d n_right;
	Eigen::Vector3d n_left;
	double maxRange = 6;
	double fovHor = 0.3;
	double fovVer = 0.7;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ()), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PCDReader reader;
	reader.read ("tof.pcd", *cloud);
	pcl::PointIndices indices;
	seg.setInputCloud(cloud);
	boost::timer t;
	seg.computeFieldOfView(fovHor,fovVer,maxRange,n_up,n_down,n_right,n_left);
	std::cout << "compute FOV: " << t.elapsed() << std::endl;
	t.restart();
	seg.segment(indices,n_up,n_down,n_right,n_left,maxRange);
	std::cout << "segment: " << t.elapsed() << std::endl;

	pcl::PointCloud<pcl::PointXYZ> frustum;
	pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
	//extractIndices.setInputCloud(cloud);
	//extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	//extractIndices.filter(frustum);
	pcl::io::savePCDFileASCII ("frustum.pcd", frustum);
	return 0;
}
