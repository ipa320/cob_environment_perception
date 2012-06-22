/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2012
 * ToDo:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <boost/make_shared.hpp>

#include <pcl/io/io.h>
#include <pcl/ros/for_each_type.h>




// Package includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_segmentation/segmentation_all_in_one_nodelet.h"



void
cob_3d_segmentation::SegmentationAllInOneNodelet::onInit()
{
	PCLNodelet::onInit();

	one_.setOutputLabels(labels_);
	one_.setPixelSearchRadius(8,2,2);
	one_.setSkipDistantPointThreshold(8);

	seg_.setNormalCloudIn(normals_);
	seg_.setLabelCloudInOut(labels_);
	seg_.setClusterGraphOut(graph_);

	cc_.setClusterHandler(graph_->clusters());
	cc_.setNormalCloudInOut(normals_);
	cc_.setLabelCloudIn(labels_);

	chull_.setAlpha (0.2);

	nh_ = getNodeHandle();
	sub_points_ = nh_.subscribe<PointCloud>
	("cloud_in", 1, boost::bind(&cob_3d_segmentation::SegmentationAllInOneNodelet::received_cloud_cb, this, _1));
	pub_segmented_ = nh_.advertise<PointCloud>("segmentation_cloud", 1);
	pub_classified_ = nh_.advertise<PointCloud>("classified_cloud", 1);
	pub_shape_array_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray>("shape_array",1);
	pub_chull_ = nh_.advertise<PointCloud>("concave_hull", 1);
	std::cout << "Loaded" << std::endl;

}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::received_cloud_cb(PointCloud::ConstPtr cloud)
{
	PrecisionStopWatch t;
	t.precisionStart();
	NODELET_INFO("Start .... ");

	one_.setInputCloud(cloud);
	one_.compute(*normals_);
	*classified_ = *segmented_ = *cloud;

	seg_.setInputCloud(cloud);
	seg_.performInitialSegmentation();
	seg_.refineSegmentation();
	graph_->clusters()->mapClusterColor(segmented_);
	cc_.setPointCloudIn(cloud);
	cc_.classify();
	graph_->clusters()->mapTypeColor(classified_);
	pub_segmented_.publish(segmented_);
	pub_classified_.publish(classified_);

	publishShapeArray(graph_->clusters(), cloud);
	NODELET_INFO("Done .... ");
}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::publishShapeArray(ST::CH::Ptr cluster_handler, PointCloud::ConstPtr cloud)
{
	cob_3d_mapping_msgs::ShapeArray sa;
	sa.header = cloud->header;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (ST::CH::ClusterPtr c = cluster_handler->begin(); c != cluster_handler->end(); ++c)
	{
		switch(c->type)
		{
		case I_PLANE:
		{
			sa.shapes.push_back(cob_3d_mapping_msgs::Shape());
			cob_3d_mapping_msgs::Shape* s = &sa.shapes.back();
			s->type = cob_3d_mapping_msgs::Shape::POLYGON;
			s->params.resize(4);
			Eigen::Vector3f n = c->getOrientation();
			s->params[0] = n(0); // n_x
			s->params[1] = n(1); // n_y
			s->params[2] = n(2); // n_z
			s->params[3] = c->getCentroid().norm(); // d
			s->points.resize(1);
			s->holes.push_back(false);
			/* // Fill in all border points
      sensor_msgs::PointCloud2* blob = &s->points.back();
      for_each_type<traits::fieldList<pcl::PointXYZ>::type> (detail::FieldAdder<pcl::PointXYZ>(blob->fields));
      blob->header = cloud->header;
      blob->width = c->border_indices.size();
      blob->height = 1;
      blob->point_step = sizeof(pcl::PointXYZ);
      blob->row_step = blob->point_step * c->border_indices.size();
      blob->data.resize(blob->row_step);
      blob->is_dense = true;

      uint8_t* msg_data = &blob->data[0];
      for (size_t i = 0; i < c->border_indices.size(); ++i, msg_data += blob->point_step)
      {
	const uint8_t* cloud_data = reinterpret_cast<const uint8_t*>(&(cloud->points[c->border_indices[i]]));
	memcpy(msg_data, cloud_data, blob->point_step);
      }
			 */
			// use concave hull algorithm
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr v_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
			chull_.setInputCloud(cloud);
			chull_.setIndices(boost::make_shared<std::vector<int> >(c->border_indices));

			std::vector< pcl::Vertices > hull_polygons;

			chull_.reconstruct(*hull,hull_polygons);

//			if (hull_polygons.size()>1) {
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_ptr=hull->makeShared();
//				pcl::ExtractIndices< pcl::PointXYZRGB > extract_2;
//				extract_2.setInputCloud(hull_ptr);
//				for( unsigned int z=0; z<hull_polygons.size(); z++)
//				{
//					pcl::PointCloud<PointXYZRGB> cloud_hull_2;
//					std::vector< pcl::Vertices > hull_polygons_2;
//					pcl::PointIndices hull_poly_indices;
//					for (unsigned int x=0; x<hull_polygons[z].vertices.size(); x++)
//						hull_poly_indices.indices.push_back(hull_polygons[z].vertices[x]);
//					//ROS_INFO("Size indices: %d", hull_poly_indices.indices.size());
//					extract_2.setIndices(boost::make_shared<const pcl::PointIndices> (hull_poly_indices));
//					extract_2.filter(cloud_hull_2);
//					//ROS_INFO("Hull 2 size: %d", cloud_hull_2.size());
//					pcl::Vertices verts;
//					for(unsigned int y=0; y<cloud_hull_2.size(); y++)
//						verts.vertices.push_back(y);
//					verts.vertices.push_back(0);
//					//ROS_INFO("Verts size: %d", verts.vertices.size());
////					hull_polygons_2.push_back(verts);
//					v_hull->push_back(cloud_hull_2);
////					v_hull_polygons.push_back(hull_polygons_2);
////					v_coefficients_plane.push_back(coefficients_plane);
//				}
//			}
//			else
//			{
//				v_hull->push_back(*hull);
////				v_hull_polygons.push_back(hull_polygons);
////				v_coefficients_plane.push_back(coefficients_plane);
//			}
//			//hull_cloud->header = hull->header;
//			//*hull_cloud += *hull;
			pcl::toROSMsg(*hull, s->points.back());
			break;
		}
		case I_CYL:
		{
			break;
		}
		default:
		{
			break;
		}
		}
	}
	//pub_chull_.publish(hull_cloud);
	pub_shape_array_.publish(sa);
}

PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, SegmentationAllInOneNodelet, cob_3d_segmentation::SegmentationAllInOneNodelet, nodelet::Nodelet);
