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

// ROS includes
#include <ros/ros.h>
#include <pcl/io/io.h>
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
	   : first_(true)
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

		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &AggregatePointMap::pointCloudSubCallbackInput, this);
		point_cloud_sub_fov_ = n_.subscribe("point_cloud2_fov", 1, &AggregatePointMap::pointCloudSubCallbackICP, this);
		point_cloud_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("point_cloud2_map",1);
		trigger_srv_client_ = n_.serviceClient<cob_env_model::TriggerStamped>("trigger_stamped");
    }

    void pointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
    {
    	ROS_INFO("PointCloudSubCallback");
    	static int frame_ctr=0;

    	StampedTransform transform;
    	try{
    		tf_listener_.lookupTransform("/map", pc->header.frame_id, ros::Time(0), transform);
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
				if(first_)
				{
					map_ = *(pc.get());
					map_.header.frame_id="/map";
					first_ = false;
				}
				else
					map_ += *(pc.get());
				//pcl::PointCloud<pcl::PointXYZ>::Ptr msg(&map_);
				point_cloud_pub_.publish(map_);

    		}
    		else
    			ROS_INFO("Skipped");
    	}
    	catch (tf::TransformException ex){
    		ROS_ERROR("%s",ex.what());
    	}
    }


    void pointCloudSubCallbackInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
    {
    	stamp_ = pc->header.stamp;
    	//ROS_INFO("PointCloudSubCallbackInput");
    	StampedTransform transform;
    	try{
    		tf_listener_.lookupTransform("/map", pc->header.frame_id, pc->header.stamp, transform);
    		KDL::Frame frame_KDL, frame_KDL_old;
    		tf::TransformTFToKDL(transform, frame_KDL);
    		tf::TransformTFToKDL(transform_old_, frame_KDL_old);
    		double r,p,y;
    		frame_KDL.M.GetRPY(r,p,y);
    		double r_old,p_old,y_old;
    		frame_KDL_old.M.GetRPY(r_old,p_old,y_old);
			if(first_)
			{
    			pcl_ros::transformPointCloud(*(pc.get()), pc_, transform);
    			pc_.header.frame_id = "/map";
    			transform_old_ = transform;
				map_ = pc_;
				map_.header.frame_id="/map";
				first_ = false;
				point_cloud_pub_.publish(map_);
			}
			else
			{
				if(fabs(r-r_old) > 0.1 || fabs(p-p_old) > 0.1 || fabs(y-y_old) > 0.1 ||
						transform.getOrigin().distance(transform_old_.getOrigin()) > 0.3)
				{
					pcl_ros::transformPointCloud(*(pc.get()), pc_, transform);
					pc_.header.frame_id = "/map";
					transform_old_ = transform;
					ROS_INFO("Trigger");
					cob_env_model::TriggerStamped trigger_req;
					trigger_req.request.stamp = pc->header.stamp;
					//pc_ = pc;
					if(trigger_srv_client_.call(trigger_req))
					{
						ROS_INFO("Trigger service called [OK].");
					}
					else
					{
						ROS_ERROR("Trigger service called [FAILED].");
					}
				}
			}
    	}
    	catch (tf::TransformException ex){
    		ROS_ERROR("%s",ex.what());
    	}
    }


	void pointCloudSubCallbackICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
	{
		StampedTransform transform;
		try{
			boost::timer t;
			std::cout << "Registering new point cloud using ICP" << std::endl;

			pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
			//pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(&pc_);
			icp.setInputCloud(pc_.makeShared());
			icp.setInputTarget(pc);
			icp.setMaximumIterations(50);
			icp.setMaxCorrespondenceDistance(0.1);
			icp.setTransformationEpsilon (1e-6);
			pcl::PointCloud<pcl::PointXYZ> pc_map_new;
			icp.align(pc_map_new);
			map_ += pc_map_new;
			std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
			ROS_INFO("Aligned PC has %d points", map_.size());
			ROS_INFO("\tTime: %f", t.elapsed());

			point_cloud_pub_.publish(map_);
			/*std::stringstream ss;
			ss << "/home/goa/pcl_daten/pc_" << i << ".pcd";
			pcl::io::savePCDFileASCII (ss.str(), map_);
			i++;*/
    		//else
    		//	ROS_INFO("Skipped");
    	}
    	catch (tf::TransformException ex){
    		ROS_ERROR("%s",ex.what());
    	}
    }



    ros::NodeHandle n_;
    ros::Time stamp_;


protected:
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber point_cloud_sub_fov_;
    ros::Publisher point_cloud_pub_;
    ros::ServiceClient trigger_srv_client_;

    TransformListener tf_listener_;
    StampedTransform transform_old_;

    pcl::PointCloud<pcl::PointXYZ> map_;
    pcl::PointCloud<pcl::PointXYZ> pc_;

    bool first_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, AggregatePointMap, AggregatePointMap, nodelet::Nodelet)

