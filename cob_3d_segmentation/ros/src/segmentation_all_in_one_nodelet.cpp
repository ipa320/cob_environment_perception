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

#include <cob_3d_mapping_common/cylinder.h>



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
	pub_shape_array_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray>("/plane_extraction/shape_array",1);
	pub_chull_ = nh_.advertise<PointCloud>("concave_hull", 1);
	std::cout << "Loaded segmentation nodelet" << std::endl;

}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::received_cloud_cb(PointCloud::ConstPtr cloud)
{
	PrecisionStopWatch t;
	t.precisionStart();
	NODELET_INFO("Start with segmentation .... ");

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
	NODELET_INFO("Done with segmentation .... ");
}

void
cob_3d_segmentation::SegmentationAllInOneNodelet::publishShapeArray(ST::CH::Ptr cluster_handler, PointCloud::ConstPtr cloud)
{
	cob_3d_mapping_msgs::ShapeArray sa;
	sa.header = cloud->header;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
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
			s->centroid.x = c->getCentroid()[0];
			s->centroid.y = c->getCentroid()[1];
			s->centroid.z = c->getCentroid()[2];

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
			/*
      // use concave hull algorithm
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);
      std::vector< pcl::Vertices > hull_polygons;
      chull_.setInputCloud(cloud);
      chull_.setIndices(boost::make_shared<std::vector<int> >(c->border_indices));
      chull_.reconstruct(*hull, hull_polygons);
      hull_cloud->header = hull->header;
			 *hull_cloud += *hull;
      pcl::toROSMsg(*hull, s->points.back());
			 */
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);
			PolygonContours<PolygonPoint> poly;
			pe_.outline(cloud->width, cloud->height, c->border_points, poly);
			//std::cout << "#Polygons: " << poly.polys_.size() << std::endl;
			s->points.resize(poly.polys_.size());
//			s->holes.push_back(false);
			for (int i = 0; i < poly.polys_.size(); ++i)
			{
				s->holes.push_back(false);
				for (std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin(); it != poly.polys_[i].end(); ++it)
				{
					hull_cloud->points.push_back(cloud->points[PolygonPoint::getInd(it->x, it->y)]);
					hull->points.push_back(cloud->points[PolygonPoint::getInd(it->x, it->y)]);
				}
				hull->height = 1;
				hull->width = hull->size();
				pcl::toROSMsg(*hull, s->points[i]);
				hull->clear();
			}
			break;
		}
		case I_CYL:
		{

			//    	cob_3d_mapping::CylinderPtr  cyl  =cob_3d_mapping::CylinderPtr(new cob_3d_mapping::Cylinder());
			//
			//    	Eigen::Vector3f centroid3f  = c->getCentroid();
			//    	cyl->centroid << centroid3f[0] , centroid3f[1] , centroid3f[2] , 0;
			//
			//    	cyl->axes_.resize(3);
			//    	cyl->axes_[1] = c->pca_inter_comp1;
			//
			//
			//    	cyl->ParamsFromCloud(cloud,c->indices_);

			break;
		}
		default:
		{
			break;
		}
		}
	}
	hull_cloud->header = cloud->header;
	hull_cloud->height = 1;
	hull_cloud->width = hull_cloud->size();
	pub_chull_.publish(hull_cloud);
	pub_shape_array_.publish(sa);
}

PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, SegmentationAllInOneNodelet, cob_3d_segmentation::SegmentationAllInOneNodelet, nodelet::Nodelet);
