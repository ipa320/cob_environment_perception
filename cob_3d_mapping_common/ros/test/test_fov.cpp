/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 02/2011
*
* \brief
* Tests for field of view node, deprecated, to be deleted
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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
	seg.computeFieldOfView();
	std::cout << "compute FOV: " << t.elapsed() << std::endl;
	t.restart();
	//seg.segment(indices,n_up,n_down,n_right,n_left,maxRange);
	std::cout << "segment: " << t.elapsed() << std::endl;

	pcl::PointCloud<pcl::PointXYZ> frustum;
	pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
	//extractIndices.setInputCloud(cloud);
	//extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	//extractIndices.filter(frustum);
	pcl::io::savePCDFileASCII ("frustum.pcd", frustum);
	return 0;
}
