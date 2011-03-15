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
#include <visualization_msgs/Marker.h>
#include <cob_env_model/TriggerStamped.h>


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
			fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",100);
			trigger_srv_ = n_.advertiseService("trigger_stamped", &SegmentFieldOfView::srvCallback_Trigger, this);
			sensor_fov_hor_ = 40*M_PI/180;
			sensor_fov_ver_ = 40*M_PI/180;
			sensor_max_range_ = 5;
			seg_.computeFieldOfView(sensor_fov_hor_,sensor_fov_ver_,sensor_max_range_,n_up_,n_down_,n_right_,n_left_);
		}

		void pointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
		{
			ROS_INFO("map received");
			//*(map_.get()) = *(pc.get());
			ROS_INFO("map size: %d", pc->points.size());
			map_ = pc;
		}

		void segment()
		{
			ROS_INFO("map size: %d", map_->points.size());
			seg_.setInputCloud(map_);
			transformNormals(map_->header.frame_id, stamp_);
			visualization_msgs::Marker marker = generateMarker(sensor_fov_hor_,sensor_fov_ver_,sensor_max_range_, map_->header.frame_id, stamp_);
			pcl::PointIndices indices;
			boost::timer t;
			seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, n_origin_t_, sensor_max_range_);
			std::cout << "Time needed for fov segmentation: " << t.elapsed() << std::endl;
			t.restart();
			pcl::PointCloud<pcl::PointXYZ> frustum;
			pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
			extractIndices.setInputCloud(map_);
			extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
			extractIndices.filter(frustum);
			std::cout << "Time needed for index extraction: " << t.elapsed() << std::endl;
			point_cloud_pub_.publish(frustum);
			fov_marker_pub_.publish(marker);
		}

		void transformNormals(std::string& target_frame, ros::Time& stamp)
		{
			tf::Point n_up(n_up_(0),n_up_(1),n_up_(2));
			tf::Stamped<tf::Point> stamped_n_up(n_up,stamp,"/head_tof_link");
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
			tf::Stamped<tf::Point> stamped_n_down(n_down,stamp,"/head_tof_link");
			tf::Stamped<tf::Point> stamped_n_down_t;
			try{
				tf_listener_.transformPoint(target_frame,stamped_n_down, stamped_n_down_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			n_down_t_(0) = stamped_n_down_t.x();
			n_down_t_(1) = stamped_n_down_t.y();
			n_down_t_(2) = stamped_n_down_t.z();

			tf::Point n_right(n_right_(0),n_right_(1),n_right_(2));
			tf::Stamped<tf::Point> stamped_n_right(n_right,stamp,"/head_tof_link");
			tf::Stamped<tf::Point> stamped_n_right_t;
			try{
				tf_listener_.transformPoint(target_frame,stamped_n_right, stamped_n_right_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			n_right_t_(0) = stamped_n_right_t.x();
			n_right_t_(1) = stamped_n_right_t.y();
			n_right_t_(2) = stamped_n_right_t.z();

			tf::Point n_left(n_left_(0),n_left_(1),n_left_(2));
			tf::Stamped<tf::Point> stamped_n_left(n_left,stamp,"/head_tof_link");
			tf::Stamped<tf::Point> stamped_n_left_t;
			try{
				tf_listener_.transformPoint(target_frame,stamped_n_left, stamped_n_left_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			n_left_t_(0) = stamped_n_left_t.x();
			n_left_t_(1) = stamped_n_left_t.y();
			n_left_t_(2) = stamped_n_left_t.z();

			tf::Point n_origin(0,0,0);
			tf::Stamped<tf::Point> stamped_n_origin(n_origin,stamp,"/head_tof_link");
			tf::Stamped<tf::Point> stamped_n_origin_t;
			try{
				tf_listener_.transformPoint(target_frame,stamped_n_origin, stamped_n_origin_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			n_origin_t_(0) = stamped_n_origin_t.x();
			n_origin_t_(1) = stamped_n_origin_t.y();
			n_origin_t_(2) = stamped_n_origin_t.z();
		}

		visualization_msgs::Marker generateMarker(double fovHorizontal, double fovVertical, double maxRange, std::string& target_frame, ros::Time& stamp)
		{
			tf::Pose marker_pose(btMatrix3x3(1,0,0,0,1,0,0,0,1));
			tf::Stamped<tf::Pose> stamped_marker_pose(marker_pose, stamp, "/head_tof_link");
			tf::Stamped<tf::Pose> stamped_marker_pose_t;
			try{
				tf_listener_.transformPose(target_frame, stamped_marker_pose, stamped_marker_pose_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			visualization_msgs::Marker marker;
			marker.header.frame_id = target_frame;
			marker.header.stamp = stamp;
			geometry_msgs::Pose pose_msg;
			tf::poseTFToMsg(stamped_marker_pose_t, pose_msg);
			marker.pose = pose_msg;
			marker.action = visualization_msgs::Marker::ADD;
			marker.type = visualization_msgs::Marker::LINE_LIST;
			marker.lifetime = ros::Duration();
			marker.scale.x = 0.01;
			marker.points.resize(16);

			double fovHorFrac = fovHorizontal/2;
			double fovVerFrac = fovVertical/2;

			marker.points[0].x = 0;
			marker.points[0].y = 0;
			marker.points[0].z = 0;

			marker.points[1].x = tan(fovHorFrac)*maxRange;
			marker.points[1].y = -tan(fovVerFrac)*maxRange;
			marker.points[1].z = maxRange;

			marker.points[2].x = 0;
			marker.points[2].y = 0;
			marker.points[2].z = 0;

			marker.points[3].x = -marker.points[1].x;
			marker.points[3].y = marker.points[1].y;
			marker.points[3].z = maxRange;

			marker.points[4].x = 0;
			marker.points[4].y = 0;
			marker.points[4].z = 0;

			marker.points[5].x = -marker.points[1].x;
			marker.points[5].y = -marker.points[1].y;
			marker.points[5].z = maxRange;

			marker.points[6].x = 0;
			marker.points[6].y = 0;
			marker.points[6].z = 0;

			marker.points[7].x = marker.points[1].x;
			marker.points[7].y = -marker.points[1].y;
			marker.points[7].z = maxRange;

			marker.points[8].x = tan(fovHorFrac)*maxRange;
			marker.points[8].y = -tan(fovVerFrac)*maxRange;
			marker.points[8].z = maxRange;

			marker.points[9].x = -marker.points[1].x;
			marker.points[9].y = marker.points[1].y;
			marker.points[9].z = maxRange;

			marker.points[10].x = -marker.points[1].x;
			marker.points[10].y = marker.points[1].y;
			marker.points[10].z = maxRange;

			marker.points[11].x = -marker.points[1].x;
			marker.points[11].y = -marker.points[1].y;
			marker.points[11].z = maxRange;

			marker.points[12].x = -marker.points[1].x;
			marker.points[12].y = -marker.points[1].y;
			marker.points[12].z = maxRange;

			marker.points[13].x = marker.points[1].x;
			marker.points[13].y = -marker.points[1].y;
			marker.points[13].z = maxRange;

			marker.points[14].x = marker.points[1].x;
			marker.points[14].y = -marker.points[1].y;
			marker.points[14].z = maxRange;

			marker.points[15].x = tan(fovHorFrac)*maxRange;
			marker.points[15].y = -tan(fovVerFrac)*maxRange;
			marker.points[15].z = maxRange;

			/*marker.points[16].x = 0;
			marker.points[16].y = 0;
			marker.points[16].z = 0;

			marker.points[17].x = n_up_(0)/n_up_.norm();
			marker.points[17].y = n_up_(1)/n_up_.norm();
			marker.points[17].z = n_up_(2)/n_up_.norm();*/

			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;

			return marker;
		}

		bool srvCallback_Trigger(cob_env_model::TriggerStamped::Request &req,
				cob_env_model::TriggerStamped::Response &res)
		{
			ROS_INFO("SegmentFieldOfView Trigger");
			stamp_ = req.stamp;
			segment();
		}

	protected:
	    ros::NodeHandle n_;
		ros::Subscriber point_cloud_sub_;
		ros::Publisher point_cloud_pub_;
		ros::Publisher fov_marker_pub_;
		ros::ServiceServer trigger_srv_;
		ros::Time stamp_;

		TransformListener tf_listener_;

		pcl::PointCloud<pcl::PointXYZ>::Ptr map_;

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
		Eigen::Vector3d n_origin_t_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, SegmentFieldOfView, SegmentFieldOfView, nodelet::Nodelet)


