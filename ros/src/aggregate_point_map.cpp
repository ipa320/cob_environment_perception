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
 * switch all console outputs to ROS_DEBUG erledigt
 * set flag to say whether pointclouds should be saved to files or not erledigt
 * rename variables according to coding guidelines: erledigt
 * 	see http://pointclouds.org/documentation/advanced/pcl_style_guide.php#variables
 * add comments to explain functionality
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
#include <cob_env_model/point_types.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/field_of_view_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model/GetFieldOfView.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### node class ####
class AggregatePointMap : public pcl_ros::PCLNodelet
{
	typedef pcl::PointXYZ Point;

public:
    // Constructor
	AggregatePointMap()
	   : first_(true),
	     ctr_(0)//,
	     /*set_maximumiterations_(50),
	     set_maxcorrespondencedistance_(0.1),
	     set_transformationepsilon_(1e-6),
	     file_path_("/home/goa/pcl_daten/table/icp/map_"),
	     ros_debug(true),
	     save_pc_(true),
	     save_icp_fov_map_(true),
	     save_pc_aligned_(true),
	     save_icp_fov_pc_(true),
	     save_map_fov_(true),
	     save_icp_map_(true),
	     vox_filter_setleafsize1(0.02),
		 vox_filter_setleafsize2(0.02),
		 vox_filter_setleafsize3(0.02),
		 r_limit_(0.1),
		 y_limit_(0.1),
		 p_limit_(0.1),
		 distance_limit_(0.3)*/
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
		point_cloud_pub_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_map",1);
		point_cloud_pub_aligned_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_aligned",1);
		//point_cloud_pub_aligned2_ = n_.advertise<pcl::PointCloud<Point> >("pc_aligned_and_boundary",1);
		fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",10);
		get_fov_srv_client_ = n_.serviceClient<cob_env_model::GetFieldOfView>("get_fov");
		//TODO: Read parameters from launch file

	/*	n_.param("aggregate_point_map/set_maxiterations_FOV_", set_maximumiterations_FOV_, 70);
		n_.param("aggregate_point_map/set_maxcorrespondencedistance_FOV_", set_maxcorrespondencedistance_FOV_ ,0.1);
		n_.param("aggregate_point_map/set_transformationepsilon_FOV_",set_transformationepsilon_FOV_ ,1e-6); */
		n_.param("aggregate_point_map/set_maximumiterations_" ,set_maximumiterations_ ,50);
		n_.param("aggregate_point_map/set_maxcorrespondencedistance_" ,set_maxcorrespondencedistance_,0.1);
		n_.param("aggregate_point_map/set_transformationepsilon_" ,set_transformationepsilon_,1e-6);
		n_.param("aggregate_point_map/file_path" ,file_path_ ,std::string("/home/goa/pcl_daten/table/icp/map_"));
		n_.param("aggregate_point_map/ros_debug" ,ros_debug ,true);
		n_.param("aggregate_point_map/save_pc_",save_pc_ , false);
		n_.param("aggregate_point_map/save_icp_fov_map_",save_icp_fov_map_ ,false);
		n_.param("aggregate_point_map/save_pc_aligned_",save_pc_aligned_,false);
		n_.param("aggregate_point_map/save_icp_fov_pc_" ,save_icp_fov_pc_,false);
		n_.param("aggregate_point_map/save_map_fov_" ,save_map_fov_,false);
		n_.param("aggregate_point_map/save_icp_map_" ,save_icp_map_,false);
		n_.param("aggregate_point_map/vox_filter_setleafsize1" ,vox_filter_setleafsize1, 0.001);
		n_.param("aggregate_point_map/vox_filter_setleafsize2" ,vox_filter_setleafsize2, 0.001);
		n_.param("aggregate_point_map/vox_filter_setleafsize3" ,vox_filter_setleafsize3, 0.001);
		n_.param("aggregate_point_map/r_limit_",r_limit_,0.1);
		n_.param("aggregate_point_map/y_limit_",y_limit_,0.1);
	    n_.param("aggregate_point_map/p_limit_",p_limit_,0.1);
	    n_.param("aggregate_point_map/distance_limit_",distance_limit_,0.3);
    }

    void pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc)
    {
    	boost::timer t;
    	ROS_INFO("PointCloudSubCallback");
    	StampedTransform transform;
    	try
    	{
       		tf_listener_.waitForTransform("/map", pc->header.frame_id, pc->header.stamp, ros::Duration(10));
    		tf_listener_.lookupTransform("/map", pc->header.frame_id, pc->header.stamp/*ros::Time(0)*/, transform);
    		KDL::Frame frame_KDL, frame_KDL_old;
    		tf::TransformTFToKDL(transform, frame_KDL);
    		tf::TransformTFToKDL(transform_old_, frame_KDL_old);
    		double r,p,y;
    		frame_KDL.M.GetRPY(r,p,y);
    		double r_old,p_old,y_old;
    		frame_KDL_old.M.GetRPY(r_old,p_old,y_old);

			if(first_)
			{
				pcl_ros::transformPointCloud(*(pc.get()), *(pc.get()), transform);
				map_ = *(pc.get());
				map_.header.frame_id="/map";
				downsampleMap();
				point_cloud_pub_.publish(map_);
				first_ = false;
			}
			else
			{
				if(fabs(r-r_old) > r_limit_ || fabs(p-p_old) > p_limit_ || fabs(y-y_old) > y_limit_ ||
						transform.getOrigin().distance(transform_old_.getOrigin()) > distance_limit_)
				{
					ROS_DEBUG_STREAM_COND(ros_debug ,  "Registering new point cloud" << std::endl);
					transform_old_ = transform;
					//transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
					//pcl_ros::transformPointCloud ("/map", *(pc.get()), *(pc.get()), tf_listener_);
					pcl_ros::transformPointCloud(*(pc.get()), *(pc.get()), transform);
					ROS_DEBUG_STREAM_COND(ros_debug ,  "frame_id " << pc->header.frame_id << std::endl);
					pc->header.frame_id = "/map";

					if(save_pc_==true)
					{
						std::stringstream ss2;
						ss2 << file_path_ << "/pc_" << ctr_ << ".pcd";
						pcl::io::savePCDFileASCII (ss2.str(), *(pc.get()));
					}
					doFOVICP(pc);
					//doICP(pc);
					//addToMap(pc);
					ctr_++;
					downsampleMap();
					point_cloud_pub_.publish(map_);
				}
			}
    	}
    	catch (tf::TransformException ex)
    	{
    		ROS_ERROR("%s",ex.what());
    	}
    	std::cout << t.elapsed() << std::endl;
    }


    void doFOVICP(const pcl::PointCloud<Point>::Ptr& pc)
    {
    	//setup IO stuff
		std::fstream filestr;
		//filestr.open("/home/goa/pcl_daten/table/icp_fov/meas.csv", std::fstream::in | std::fstream::out | std::fstream::app);

		//generate marker for FOV visualization in RViz
		//visualization_msgs::Marker marker = generateMarker(sensor_fov_hor_,sensor_fov_ver_,sensor_max_range_, map_.header.frame_id, pc->header.stamp);
		//fov_marker_pub_.publish(marker);
		cob_env_model::GetFieldOfView get_fov_srv;
		get_fov_srv.request.target_frame = std::string("/map");
		if(get_fov_srv_client_.call(get_fov_srv))
		{
			ROS_DEBUG_STREAM_COND(ros_debug ,"[aggregate_point_map] FOV service called [OK].");
		}
		else
		{
			ROS_ERROR("[aggregate_point_map] FOV service called [FAILED].");
			return;
		}
		n_up_t_(0) = get_fov_srv.response.fov.points[0].x;
		n_up_t_(1) = get_fov_srv.response.fov.points[0].y;
		n_up_t_(2) = get_fov_srv.response.fov.points[0].z;
		n_down_t_(0) = get_fov_srv.response.fov.points[1].x;
		n_down_t_(1) = get_fov_srv.response.fov.points[1].y;
		n_down_t_(2) = get_fov_srv.response.fov.points[1].z;
		n_right_t_(0) = get_fov_srv.response.fov.points[2].x;
		n_right_t_(1) = get_fov_srv.response.fov.points[2].y;
		n_right_t_(2) = get_fov_srv.response.fov.points[2].z;
		n_left_t_(0) = get_fov_srv.response.fov.points[3].x;
		n_left_t_(1) = get_fov_srv.response.fov.points[3].y;
		n_left_t_(2) = get_fov_srv.response.fov.points[3].z;
		n_origin_t_(0) = get_fov_srv.response.fov.points[4].x;
		n_origin_t_(1) = get_fov_srv.response.fov.points[4].y;
		n_origin_t_(2) = get_fov_srv.response.fov.points[4].z;
		n_max_range_t_(0) = get_fov_srv.response.fov.points[5].x;
		n_max_range_t_(1) = get_fov_srv.response.fov.points[5].y;
		n_max_range_t_(2) = get_fov_srv.response.fov.points[5].z;


		//segment FOV
		seg_.setInputCloud(map_.makeShared());
		//transformNormals(map_.header.frame_id, pc->header.stamp);
		pcl::PointIndices indices;
		seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, n_origin_t_, n_max_range_t_);
		/*pcl::PointCloud<Point> frustum;
		pcl::ExtractIndices<Point> extractIndices;
		extractIndices.setInputCloud(map_.makeShared());
		extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
		extractIndices.filter(frustum);*/
		if (save_map_fov_==true)
		{
			pcl::PointCloud<Point> frustum;
			pcl::ExtractIndices<Point> extractIndices;
			extractIndices.setInputCloud(map_.makeShared());
			extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
			extractIndices.filter(frustum);
			std::stringstream ss3;
			ss3 << file_path_ << "/map_fov_" << ctr_ << ".pcd";
			pcl::io::savePCDFileASCII (ss3.str(), frustum);
		}

		//do ICP
		boost::timer t;
		pcl::IterativeClosestPoint<Point,Point> icp;
		//TODO: Test
		icp.setInputCloud(map_.makeShared());
		icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
		icp.setInputTarget(pc->makeShared());
		icp.setMaximumIterations(set_maximumiterations_);
		icp.setMaxCorrespondenceDistance(set_maxcorrespondencedistance_);
		icp.setTransformationEpsilon (set_transformationepsilon_);
		pcl::PointCloud<Point> pc_aligned;
		icp.align(pc_aligned);
		map_ += pc_aligned;

		//do logging
		double time = t.elapsed();
		//ROS_DEBUG_STREAM_COND(ros_debug ,"Aligning pc with " << pc->size() << " to map_fov with " << frustum.size() << std::endl);
		ROS_DEBUG_STREAM_COND(ros_debug ,"ICP has converged:" << icp.hasConverged() << std::endl);
		ROS_DEBUG_STREAM_COND(ros_debug ,"Fitness score: " << icp.getFitnessScore() << std::endl);
		ROS_DEBUG_STREAM_COND(ros_debug ,"Aligned PC has %d points" << map_.size());
		filestr << ctr_ <<";" << pc_aligned.size()<<";"<<map_.size() <<";"<<time<<";"<<icp.getFitnessScore()<<std::endl;
		ROS_DEBUG_STREAM_COND(ros_debug ,"\tTime: %f"<< time);


		if(save_icp_fov_map_ ==true)
			{
				std::stringstream ss1;
				ss1 << file_path_ << "/map_" << ctr_ << ".pcd";
				pcl::io::savePCDFileASCII (ss1.str(), map_);
			}
		//frustum += pc_aligned;
		/*pcl::VoxelGrid<Point> vox_filter;
		vox_filter.setInputCloud(pc_aligned.makeShared());
		vox_filter.setLeafSize(0.005, 0.005, 0.005);
		vox_filter.filter(pc_aligned);*/
		point_cloud_pub_aligned_.publish(pc_aligned);
		//point_cloud_pub_aligned2_.publish(frustum);

		if(save_pc_aligned_==true)
		{
			ROS_INFO("Saving pc_aligned.");
			std::stringstream ss;
			ss << file_path_ << "/pc_aligned_" << ctr_ << ".pcd";
			pcl::io::savePCDFileASCII (ss.str(), pc_aligned);
		}
		if(save_icp_fov_pc_==true)
		{
			std::stringstream ss2;
			ss2 << file_path_ << "/pc_" << ctr_ << ".pcd";
			pcl::io::savePCDFileASCII (ss2.str(), *(pc.get()));
		}
    	//filestr.close();
    }


    void doICP(const pcl::PointCloud<Point>::Ptr& pc)
    {
    	//TODO: change map2_ to map_, flag for IO operations, publish to callback
    	//Open file for timer log
		std::fstream filestr;
		filestr.open("/home/goa/pcl_daten/table/icp/meas.csv", std::fstream::in | std::fstream::out | std::fstream::app);
		boost::timer t;

		//Perform ICP
		pcl::IterativeClosestPoint<Point,Point> icp;
		icp.setInputCloud(pc);
		icp.setInputTarget(map_.makeShared());
		icp.setMaximumIterations(set_maximumiterations_);
		icp.setMaxCorrespondenceDistance(set_maxcorrespondencedistance_);
		icp.setTransformationEpsilon (set_transformationepsilon_);
		pcl::PointCloud<Point> pc_aligned;
		icp.align(pc_aligned);
		map_ += pc_aligned;

		//TODO: output as ROS_DEBUG
		double time = t.elapsed();
		ROS_DEBUG_STREAM_COND(ros_debug ,  "Aligning pc with " << pc->size() << " to map with " << map_.size() << std::endl);
		ROS_DEBUG_STREAM_COND(ros_debug ,  "ICP has converged:" << icp.hasConverged() << std::endl);
		ROS_DEBUG_STREAM_COND(ros_debug ,  "Fitness score: " << ctr_ << "," << icp.getFitnessScore() << std::endl);
		ROS_DEBUG_STREAM_COND(ros_debug ,"Aligned PC has %d points"<< map_.size());
		filestr << ctr_ <<";" << pc_aligned.size()<<";"<<map_.size() <<";"<<time<<";"<<icp.getFitnessScore()<<std::endl;
		ROS_DEBUG_STREAM_COND(ros_debug ,"\tTime: %f"<< time);

		//TODO: parameter for file path
		if(save_icp_map_==true)
		{
			std::stringstream ss1;
			ss1 << file_path_ << ctr_ << ".pcd";
			pcl::io::savePCDFileASCII (ss1.str(), map_);
		}
		/*pcl::VoxelGrid<Point> vox_filter;
		vox_filter.setInputCloud(map_.makeShared());
		vox_filter.setLeafSize(0.03, 0.03, 0.03);
		vox_filter.filter(map2_);*/
		//point_cloud_pub_.publish(map2_);

    	filestr.close();
    }

    void addToMap(const pcl::PointCloud<Point>::Ptr& pc)
    {
    	map_ += *(pc.get());
    }

    void downsampleMap()
    {
		pcl::VoxelGrid<Point> vox_filter;
		vox_filter.setInputCloud(map_.makeShared());
		//TODO: launchfile parameter
		vox_filter.setLeafSize(vox_filter_setleafsize1,vox_filter_setleafsize2,vox_filter_setleafsize3);
		vox_filter.filter(map_);
    }


    ros::NodeHandle n_;
    ros::Time stamp_;


protected:
    ros::Subscriber point_cloud_sub_;		//subscriber for input pc
    ros::Publisher point_cloud_pub_;		//publisher for map
    ros::Publisher point_cloud_pub_aligned_;//publisher for aligned pc
    //ros::Publisher point_cloud_pub_aligned2_;//publisher for aligned pc
    ros::Publisher fov_marker_pub_;			//publisher for FOV marker
    ros::ServiceClient get_fov_srv_client_;

    TransformListener tf_listener_;
    StampedTransform transform_old_;

    pcl::PointCloud<Point> map_;	//FOV ICP map

    bool first_;
    int set_maximumiterations_;
    double set_maxcorrespondencedistance_;
    double set_transformationepsilon_;
/*
    int set_maximumiterations_FOV_;
    double set_maxcorrespondencedistance_FOV_;
    double set_transformationepsilon_FOV_;
*/
    double vox_filter_setleafsize1;
    double vox_filter_setleafsize2;
    double vox_filter_setleafsize3;

    bool ros_debug;
    std::string file_path_;

    //Speichervariablen
    bool save_pc_;
    bool save_icp_fov_map_;
    bool save_pc_aligned_;
    bool save_icp_fov_pc_;
    bool save_map_fov_;
    bool save_icp_map_;


    double y_limit_;
    double distance_limit_;
    double r_limit_;
    double p_limit_;

	Eigen::Vector3d n_up_t_;
	Eigen::Vector3d n_down_t_;
	Eigen::Vector3d n_right_t_;
	Eigen::Vector3d n_left_t_;
	Eigen::Vector3d n_origin_t_;
	Eigen::Vector3d n_max_range_t_;

	ipa_env_model::FieldOfViewSegmentation<Point> seg_;

	int ctr_;

};


PLUGINLIB_DECLARE_CLASS(cob_env_model, AggregatePointMap, AggregatePointMap, nodelet::Nodelet)

