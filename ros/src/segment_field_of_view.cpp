/****************************************************************
 *
 * Copyright (c) 2010
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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 02/2011
 * ToDo:
 * Publish frustum for visualization
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


//##################
//#### includes ####

// standard includes
//--
#include <sstream>

// ROS includes
#include <ros/ros.h>
#include <cob_env_model/field_of_view_segmentation.h>
#include <pcl/io/io.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cob_env_model/cpc_point.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/filters/extract_indices.h>


// external includes
#include <boost/timer.hpp>


using namespace tf;

//####################
//#### node class ####
class SegmentFieldOfView : public pcl_ros::PCLNodelet
{
	public:
		// Constructor
		SegmentFieldOfView()
		{
		}


		// Destructor
		~SegmentFieldOfView()
		{
			/// void
		}

		void onInit()
		{
			PCLNodelet::onInit();
			n_ = getNodeHandle();

			point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &SegmentFieldOfView::pointCloudSubCallback, this);
			point_cloud_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("point_cloud2_fov",1);
			sensor_fov_hor_ = 40/180*M_PI;
			sensor_fov_ver_ = 40/180*M_PI;
			sensor_max_range_ = 5;
			seg_.computeFieldOfView(sensor_fov_hor_,sensor_fov_ver_,sensor_max_range_,n_up_,n_down_,n_right_,n_left_);
		}

		void pointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
		{
			seg_.setInputCloud(pc);
			transformNormals(pc->header.frame_id);
			pcl::PointIndices indices;
			seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, sensor_max_range_);
			pcl::PointCloud<pcl::PointXYZ> frustum;
			pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
			extractIndices.setInputCloud(pc);
			extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
			extractIndices.filter(frustum);
			point_cloud_pub_.publish(frustum);
		}

		void transformNormals(std::string& target_frame)
		{
			tf::Point n_up(n_up_(0),n_up_(1),n_up_(2));
			tf::Stamped<tf::Point> stamped_n_up(n_up,ros::Time(0),"/head_tof_link");
			tf::Stamped<tf::Point> stamped_n_up_t;
			try{
				tf_listener_.transformPoint(target_frame,stamped_n_up, stamped_n_up_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			n_up_t_(0) = stamped_n_up_t.x();
			n_up_t_(1) = stamped_n_up_t.y();
			n_up_t_(2) = stamped_n_up_t.z();

			tf::Point n_down(n_down_(0),n_down_(1),n_down_(2));
			tf::Stamped<tf::Point> stamped_n_down(n_down,ros::Time(0),"/head_tof_link");
			tf::Stamped<tf::Point> stamped_n_down_t;
			try{
				tf_listener_.transformPoint("/map",stamped_n_down, stamped_n_down_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			n_down_t_(0) = stamped_n_down_t.x();
			n_down_t_(1) = stamped_n_down_t.y();
			n_down_t_(2) = stamped_n_down_t.z();

			tf::Point n_right(n_right_(0),n_right_(1),n_right_(2));
			tf::Stamped<tf::Point> stamped_n_right(n_right,ros::Time(0),"/head_tof_link");
			tf::Stamped<tf::Point> stamped_n_right_t;
			try{
				tf_listener_.transformPoint("/map",stamped_n_right, stamped_n_right_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			n_right_t_(0) = stamped_n_right_t.x();
			n_right_t_(1) = stamped_n_right_t.y();
			n_right_t_(2) = stamped_n_right_t.z();

			tf::Point n_left(n_left_(0),n_left_(1),n_left_(2));
			tf::Stamped<tf::Point> stamped_n_left(n_left,ros::Time(0),"/head_tof_link");
			tf::Stamped<tf::Point> stamped_n_left_t;
			try{
				tf_listener_.transformPoint("/map",stamped_n_left, stamped_n_left_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			n_left_t_(0) = stamped_n_left_t.x();
			n_left_t_(1) = stamped_n_left_t.y();
			n_left_t_(2) = stamped_n_left_t.z();
		}


	protected:
	    ros::NodeHandle n_;
		ros::Subscriber point_cloud_sub_;
		ros::Publisher point_cloud_pub_;

		TransformListener tf_listener_;

		ipa_env_model::FieldOfViewSegmentation<pcl::PointXYZ> seg_;

		double sensor_fov_hor_;
		double sensor_fov_ver_;
		double sensor_max_range_;

		Eigen::Vector3d n_up_;
		Eigen::Vector3d n_down_;
		Eigen::Vector3d n_right_;
		Eigen::Vector3d n_left_;

		Eigen::Vector3d n_up_t_;
		Eigen::Vector3d n_down_t_;
		Eigen::Vector3d n_right_t_;
		Eigen::Vector3d n_left_t_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, SegmentFieldOfView, SegmentFieldOfView, nodelet::Nodelet)


