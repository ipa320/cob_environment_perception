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
 * Date of creation: 01/2011
 * ToDo:
 * Only update if new robot pose available
 * Resample point cloud
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
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cob_env_model/cpc_point.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/TriggerStamped.h>
#include <cob_env_model/field_of_view_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### node class ####
class AggregatePointMap : public pcl_ros::PCLNodelet
{
public:
    // Constructor
	AggregatePointMap()
	   : first_(true),
	     ctr_(0)
	{
	}


    // Destructor
    ~AggregatePointMap()
    {
    	/// void
    }

    void onInit()
    {
    	PCLNodelet::onInit();
    	n_ = getNodeHandle();

		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &AggregatePointMap::pointCloudSubCallback, this);
		point_cloud_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud2_map",1);
		point_cloud_pub_aligned_ = n_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud2_aligned",1);
		fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",10);

		//TODO: move to separate class, set as parameter
		sensor_fov_hor_ = 40*M_PI/180;
		sensor_fov_ver_ = 40*M_PI/180;
		sensor_max_range_ = 5;
		seg_.computeFieldOfView(sensor_fov_hor_,sensor_fov_ver_,sensor_max_range_,n_up_,n_down_,n_right_,n_left_);
    }

    void pointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
    {
    	//ROS_INFO("PointCloudSubCallback");
    	StampedTransform transform;
    	try
    	{
    		tf_listener_.lookupTransform("/map", pc->header.frame_id, pc->header.stamp/*ros::Time(0)*/, transform);
    		KDL::Frame frame_KDL, frame_KDL_old;
    		tf::TransformTFToKDL(transform, frame_KDL);
    		tf::TransformTFToKDL(transform_old_, frame_KDL_old);
    		double r,p,y;
    		frame_KDL.M.GetRPY(r,p,y);
    		double r_old,p_old,y_old;
    		frame_KDL_old.M.GetRPY(r_old,p_old,y_old);
    		if(fabs(r-r_old) > 0.1 || fabs(p-p_old) > 0.1 || fabs(y-y_old) > 0.1 ||
    				transform.getOrigin().distance(transform_old_.getOrigin()) > 0.3)
    		{
    			std::cout << "Registering new point cloud" << std::endl;
    			transform_old_ = transform;
				//transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
    			//pcl_ros::transformPointCloud ("/map", *(pc.get()), *(pc.get()), tf_listener_);
    			pcl_ros::transformPointCloud(*(pc.get()), *(pc.get()), transform);
    			pc->header.frame_id = "/map";
				std::stringstream ss2;
				ss2 << "/home/goa/pcl_daten/table/icp_no/pc_" << ctr_ << ".pcd";
				pcl::io::savePCDFileASCII (ss2.str(), *(pc.get()));
				if(first_)
				{
					map_ = *(pc.get());
					map2_ = *(pc.get());
					map_.header.frame_id="/map";
					map2_.header.frame_id="/map";
					map3_ = *(pc.get());
					map3_.header.frame_id="/map";
					first_ = false;
				}
				else
				{
					doFOVICP(pc);
					//doICP(pc);
					//map3_ += *(pc.get());
				}
				std::stringstream ss1;
				ss1 << "/home/goa/pcl_daten/table/icp_no/map_" << ctr_ << ".pcd";
				pcl::io::savePCDFileASCII (ss1.str(), map3_);
				ctr_++;
				/*pcl::VoxelGrid<pcl::PointXYZRGB> vox_filter;
				vox_filter.setInputCloud(map_.makeShared());
				vox_filter.setLeafSize(0.02, 0.02, 0.02);
				vox_filter.filter(map_);
				vox_filter.setInputCloud(map2_.makeShared());
				vox_filter.setLeafSize(0.02, 0.02, 0.02);
				vox_filter.filter(map2_);*/
				//point_cloud_pub_.publish(map_);

    		}
    		//else
    		//	ROS_INFO("Skipped");
    	}
    	catch (tf::TransformException ex)
    	{
    		ROS_ERROR("%s",ex.what());
    	}
    }


    void doFOVICP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
    {
    	//setup IO stuff
		std::fstream filestr;
		filestr.open("/home/goa/pcl_daten/table/icp_fov/meas.csv", std::fstream::in | std::fstream::out | std::fstream::app);

		//generate marker for FOV visualization in RViz
		visualization_msgs::Marker marker = generateMarker(sensor_fov_hor_,sensor_fov_ver_,sensor_max_range_, map_.header.frame_id, pc->header.stamp);
		fov_marker_pub_.publish(marker);

		//segment FOV
		//TODO: move to separate class
		seg_.setInputCloud(map_.makeShared());
		transformNormals(map_.header.frame_id, pc->header.stamp);
		pcl::PointIndices indices;
		seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, n_origin_t_, sensor_max_range_);
		pcl::PointCloud<pcl::PointXYZRGB> frustum;
		pcl::ExtractIndices<pcl::PointXYZRGB> extractIndices;
		extractIndices.setInputCloud(map_.makeShared());
		extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
		extractIndices.filter(frustum);

		//do ICP
		boost::timer t;
		pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
		icp.setInputCloud(pc->makeShared());
		icp.setInputTarget(frustum.makeShared());
		//TODO: set as parameters
		icp.setMaximumIterations(70);
		icp.setMaxCorrespondenceDistance(0.1);
		icp.setTransformationEpsilon (1e-6);
		pcl::PointCloud<pcl::PointXYZRGB> pc_aligned;
		icp.align(pc_aligned);
		map_ += pc_aligned;

		//do logging
		double time = t.elapsed();
		std::cout << "Aligning pc with " << pc->size() << " to map_fov with " << frustum.size() << std::endl;
		std::cout << "ICP has converged:" << icp.hasConverged() << std::endl;
		std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
		ROS_INFO("Aligned PC has %d points", map_.size());
		filestr << ctr_ <<";" << pc_aligned.size()<<";"<<map_.size() <<";"<<time<<";"<<icp.getFitnessScore()<<std::endl;
		ROS_INFO("\tTime: %f", time);
		std::stringstream ss1;
		ss1 << "/home/goa/pcl_daten/table/icp_fov/map_" << ctr_ << ".pcd";
		pcl::io::savePCDFileASCII (ss1.str(), map_);

		/*pcl::VoxelGrid<pcl::PointXYZRGB> vox_filter;
		vox_filter.setInputCloud(map_.makeShared());
		vox_filter.setLeafSize(0.03, 0.03, 0.03);
		vox_filter.filter(map_);*/

		point_cloud_pub_.publish(map_);
		point_cloud_pub_aligned_.publish(pc_aligned);
		std::stringstream ss;
		ss << "/home/goa/pcl_daten/table/icp_fov/pc_aligned_" << ctr_ << ".pcd";
		pcl::io::savePCDFileASCII (ss.str(), pc_aligned);
		std::stringstream ss2;
		ss2 << "/home/goa/pcl_daten/table/icp_fov/pc_" << ctr_ << ".pcd";
		pcl::io::savePCDFileASCII (ss2.str(), *(pc.get()));
		std::stringstream ss3;
		ss3 << "/home/goa/pcl_daten/table/icp_fov/map_fov_" << ctr_ << ".pcd";
		pcl::io::savePCDFileASCII (ss3.str(), frustum);

    	filestr.close();
    }


    void doICP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
    {
    	//TODO: change map2_ to map_, flag for IO operations, publish to callback
    	//Open file for timer log
		std::fstream filestr;
		filestr.open("/home/goa/pcl_daten/table/icp/meas.csv", std::fstream::in | std::fstream::out | std::fstream::app);
		boost::timer t;

		//Perform ICP
		pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
		icp.setInputCloud(pc);
		icp.setInputTarget(map2_.makeShared());
		//TODO: set parameter
		icp.setMaximumIterations(50);
		icp.setMaxCorrespondenceDistance(0.1);
		icp.setTransformationEpsilon (1e-6);
		pcl::PointCloud<pcl::PointXYZRGB> pc_aligned;
		icp.align(pc_aligned);
		map2_ += pc_aligned;

		//TODO: output as ROS_DEBUG
		double time = t.elapsed();
		std::cout << "Aligning pc with " << pc->size() << " to map with " << map2_.size() << std::endl;
		std::cout << "ICP has converged:" << icp.hasConverged() << std::endl;
		std::cout << "Fitness score: " << ctr_ << "," << icp.getFitnessScore() << std::endl;
		ROS_INFO("Aligned PC has %d points", map2_.size());
		filestr << ctr_ <<";" << pc_aligned.size()<<";"<<map2_.size() <<";"<<time<<";"<<icp.getFitnessScore()<<std::endl;
		ROS_INFO("\tTime: %f", time);

		//TODO: parameter for file path
		std::stringstream ss1;
		ss1 << "/home/goa/pcl_daten/table/icp/map_" << ctr_ << ".pcd";
		pcl::io::savePCDFileASCII (ss1.str(), map2_);

		/*pcl::VoxelGrid<pcl::PointXYZRGB> vox_filter;
		vox_filter.setInputCloud(map_.makeShared());
		vox_filter.setLeafSize(0.03, 0.03, 0.03);
		vox_filter.filter(map2_);*/
		point_cloud_pub_.publish(map2_);

    	filestr.close();
    }

    //TODO: move to separate class
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




    ros::NodeHandle n_;
    ros::Time stamp_;


protected:
    ros::Subscriber point_cloud_sub_;		//subscriber for input pc
    ros::Publisher point_cloud_pub_;		//publisher for map
    ros::Publisher point_cloud_pub_aligned_;//publisher for aligned pc
    ros::Publisher fov_marker_pub_;			//publisher for FOV marker

    TransformListener tf_listener_;
    StampedTransform transform_old_;

    pcl::PointCloud<pcl::PointXYZRGB> map_;	//FOV ICP map
    pcl::PointCloud<pcl::PointXYZRGB> map2_;//ICP map
    pcl::PointCloud<pcl::PointXYZRGB> map3_;//no ICP map

    bool first_;

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

	ipa_env_model::FieldOfViewSegmentation<pcl::PointXYZRGB> seg_;

	int ctr_;

};


PLUGINLIB_DECLARE_CLASS(cob_env_model, AggregatePointMap, AggregatePointMap, nodelet::Nodelet)

