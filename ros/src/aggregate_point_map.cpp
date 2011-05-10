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
#include <ros/console.h>
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
	     ctr_(0),
	     set_maximumiterations(50),
	     set_maxcorrespondencedistance(0.1),
	     set_transformationepsilon(1e-6),
	     file_path("/home/goa/pcl_daten/table/icp/map_"),
	     ros_debug(true),
	     save_pc(true),
	     save_icp_fov_map(true),
	     save_pc_aligned(true),
	     save_icp_fov_pc(true),
	     save_map_fov(true),
	     save_icp_map(true),
	     vox_filter_setleafsize1(0.02),
		 vox_filter_setleafsize2(0.02),
		 vox_filter_setleafsize3(0.02),
		 r_limit(0.1),
		 y_limit(0.1),
		 p_limit(0.1),
		 distance_limit(0.3),
		 mode(0)
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
		fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",10);
		get_fov_srv_client_ = n_.serviceClient<cob_env_model::GetFieldOfView>("get_fov");

		n_.param("aggregate_point_map/set_maximumiterations" ,set_maximumiterations ,50);
		n_.param("aggregate_point_map/set_maxcorrespondencedistance" ,set_maxcorrespondencedistance,0.1);
		n_.param("aggregate_point_map/set_transformationepsilon" ,set_transformationepsilon,1e-6);

		n_.param("aggregate_point_map/ros_debug" ,ros_debug ,true);
		n_.param("aggregate_point_map/save_pc",save_pc , true);
		n_.param("aggregate_point_map/save_icp_fov_map",save_icp_fov_map ,false);
		n_.param("aggregate_point_map/save_pc_aligned",save_pc_aligned,false);
		n_.param("aggregate_point_map/save_icp_fov_pc" ,save_icp_fov_pc,true);
		n_.param("aggregate_point_map/save_map_fov" ,save_map_fov,false);
		n_.param("aggregate_point_map/save_icp_map" ,save_icp_map,true);
		n_.param("aggregate_point_map/vox_filter_setleafsize1" ,vox_filter_setleafsize1, 0.02);
		n_.param("aggregate_point_map/vox_filter_setleafsize2" ,vox_filter_setleafsize2, 0.02);
		n_.param("aggregate_point_map/vox_filter_setleafsize3" ,vox_filter_setleafsize3, 0.02);
		n_.param("aggregate_point_map/r_limit",r_limit,0.01);
		n_.param("aggregate_point_map/y_limit",y_limit,0.01);
	    n_.param("aggregate_point_map/p_limit",p_limit,0.01);
	    n_.param("aggregate_point_map/distance_limit",distance_limit,0.03);
	    n_.param("aggregate_point_map/mode",mode,1);

    }

    void pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc)
    {
    	ROS_INFO("callback");
    	ROS_DEBUG_STREAM_COND(ros_debug ,"debug");
    	StampedTransform transform;
    	try
    	{
    		ROS_INFO("TRANSFORM");
       		tf_listener_.waitForTransform("/map", pc->header.frame_id, /*ros::Time(0)*/pc->header.stamp, ros::Duration(3));
    		tf_listener_.lookupTransform("/map", pc->header.frame_id, pc->header.stamp/*ros::Time(0)*/, transform);
    		KDL::Frame frame_KDL, frame_KDL_old;
    		tf::TransformTFToKDL(transform, frame_KDL);
    		tf::TransformTFToKDL(transform_old_, frame_KDL_old);
    		double r,p,y;
    		frame_KDL.M.GetRPY(r,p,y);
    		double r_old,p_old,y_old;
    		frame_KDL_old.M.GetRPY(r_old,p_old,y_old);
    		if(fabs(r-r_old) > r_limit || fabs(p-p_old) > p_limit || fabs(y-y_old) > y_limit ||
    				transform.getOrigin().distance(transform_old_.getOrigin()) > distance_limit)
    		{
    			ROS_INFO( "Registering new point cloud" );
    			transform_old_ = transform;
				//transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
    			//pcl_ros::transformPointCloud ("/map", *(pc.get()), *(pc.get()), tf_listener_);
    			pcl_ros::transformPointCloud(*(pc.get()), *(pc.get()), transform);
    			ROS_DEBUG_STREAM_COND(ros_debug ,  "frame_id " << pc->header.frame_id << std::endl);
    			pc->header.frame_id = "/map";




				if(save_pc==true)
					{
						std::stringstream ss2;
						ss2 << "/home/goa-hh/pcl_daten/table/pc_" << ctr_ << ".pcd";
						pcl::io::savePCDFileASCII (ss2.str(), *pc);
					}
					if(first_==true)
				{
					map_ = *(pc.get());
					map_.header.frame_id="/map";
					first_ = false;
				}
				else
				{
					switch (mode)
					{
					case 1 : ROS_INFO("doit");
							 doFOVICP(pc);
					case 2 : doICP(pc);
					case 3 : addToMap(pc);
					}

				}
				ctr_++;
				//downsampleMap();

				point_cloud_pub_.publish(map_);

    		}
    		//else
    		//	ROS_INFO("Skipped");
    	}
    	catch (tf::TransformException ex)
    	{
    		ROS_ERROR("%s",ex.what());
    	}
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
		ROS_INFO("testhier");
		if(get_fov_srv_client_.call(get_fov_srv))  {

			ROS_INFO("ja");
			ROS_DEBUG_STREAM_COND(true ,"[aggregate_point_map] FOV service called [OK].");
		}
		else
		{
			ROS_INFO("NEIN");
			ROS_ERROR("[aggregate_point_map] FOV service called [FAILED].");
			return;
		}
		ROS_INFO("schleifenende");
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
		pcl::PointCloud<Point> frustum;
		pcl::ExtractIndices<Point> extractIndices;
		extractIndices.setInputCloud(map_.makeShared());
		extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
		extractIndices.filter(frustum);
		ROS_INFO("nach filter");
		//do ICP
		boost::timer t;
		pcl::IterativeClosestPoint<Point,Point> icp;
		icp.setInputCloud(pc->makeShared());
		icp.setInputTarget(frustum.makeShared());
		icp.setMaximumIterations(set_maximumiterations);
		icp.setMaxCorrespondenceDistance(set_maxcorrespondencedistance);
		icp.setTransformationEpsilon (set_transformationepsilon);
		pcl::PointCloud<Point> pc_aligned;
		icp.align(pc_aligned);
		map_ += pc_aligned; // map filtern

		/*pcl::VoxelGrid<Point> vox_filter2;
		vox_filter2.setInputCloud(map_.makeShared());
		vox_filter2.setLeafSize(0.005, 0.005, 0.005);
		vox_filter2.filter(map_);
*/


		//do logging
		double time = t.elapsed();
		pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/table/icp_fov/map_test.pcd", map_);
		ROS_INFO("vor ausgabe");
		ROS_DEBUG_STREAM_COND(ros_debug ,"Aligning pc with " << pc->size() << " to map_fov with " << frustum.size());
		ROS_DEBUG_STREAM_COND(ros_debug ,"ICP has converged:" << icp.hasConverged());
		ROS_DEBUG_STREAM_COND(ros_debug ,"Fitness score: " << icp.getFitnessScore());
		ROS_DEBUG_STREAM_COND(ros_debug ,"Aligned PC has %d points" << map_.size());
		filestr << ctr_ <<";" << pc_aligned.size()<<";"<<map_.size() <<";"<<time<<";"<<icp.getFitnessScore()<<std::endl;
		ROS_DEBUG_STREAM_COND(ros_debug ,"\tTime: %f"<< time);





		if(save_icp_fov_map ==true)
			{
				std::stringstream ss1;
				ss1 << "/home/goa-hh/pcl_daten/table/icp_fov/map_" << ctr_ << ".pcd";
				pcl::io::savePCDFileASCII (ss1.str(), map_);
			}
		ROS_INFO("frustum");
		frustum += pc_aligned;
		pcl::VoxelGrid<Point> vox_filter;
		vox_filter.setInputCloud(frustum.makeShared());
		vox_filter.setLeafSize(0.005, 0.005, 0.005);
		vox_filter.filter(frustum);
		point_cloud_pub_aligned_.publish(frustum);
		ROS_INFO("frustum ok");

		if(save_pc_aligned==true)
		{
			std::stringstream ss;
			ss << "/home/goa-hh/pcl_daten/table/icp_fov/pc_aligned_" << ctr_ << ".pcd";
			pcl::io::savePCDFileASCII (ss.str(), pc_aligned);
		}
		if(save_icp_fov_pc==true)
		{
			std::stringstream ss2;
			ss2 << "/home/goa-hh/pcl_daten/table/icp_fov/pc_" << ctr_ << ".pcd";
			pcl::io::savePCDFileASCII (ss2.str(), *(pc.get()));
		}
		if (save_map_fov==true)
		{
			std::stringstream ss3;
			ss3 << "/home/goa/pcl_daten/table/icp_fov/map_fov_" << ctr_ << ".pcd";
			pcl::io::savePCDFileASCII (ss3.str(), frustum);
		}
    	//filestr.close();
		ROS_INFO("ende");
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
		//TODO: set parameter
		icp.setMaximumIterations(set_maximumiterations);
		icp.setMaxCorrespondenceDistance(set_maxcorrespondencedistance);
		icp.setTransformationEpsilon (set_transformationepsilon);
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
		if(save_icp_map==true)
		{
			std::stringstream ss1;
			ss1 << file_path << ctr_ << ".pcd";
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
    ros::Publisher fov_marker_pub_;			//publisher for FOV marker
    ros::ServiceClient get_fov_srv_client_;

    TransformListener tf_listener_;
    StampedTransform transform_old_;

    pcl::PointCloud<Point> map_;	//FOV ICP map

    bool first_ ;
    int ctr_;

    int set_maximumiterations ;
    double set_maxcorrespondencedistance;
    double set_transformationepsilon;

    double vox_filter_setleafsize1;
    double vox_filter_setleafsize2;
    double vox_filter_setleafsize3;

    bool ros_debug;
    std::string file_path;

    //Speichervariablen
    bool save_pc;
    bool save_icp_fov_map;
    bool save_pc_aligned;
    bool save_icp_fov_pc;
    bool save_map_fov;
    bool save_icp_map;


    double y_limit;
    double distance_limit;
    double r_limit;
    double p_limit;

	Eigen::Vector3d n_up_t_;
	Eigen::Vector3d n_down_t_;
	Eigen::Vector3d n_right_t_;
	Eigen::Vector3d n_left_t_;
	Eigen::Vector3d n_origin_t_;
	Eigen::Vector3d n_max_range_t_;

	ipa_env_model::FieldOfViewSegmentation<Point> seg_;


	int mode;

};


PLUGINLIB_DECLARE_CLASS(cob_env_model, AggregatePointMap, AggregatePointMap, nodelet::Nodelet)

