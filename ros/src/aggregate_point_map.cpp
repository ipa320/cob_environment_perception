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

		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &AggregatePointMap::pointCloudSubCallbackICP, this);
		point_cloud_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("point_cloud2_map",1);
    }

    void pointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
    {
    	//ROS_INFO("PointCloudSubCallback");
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

    void pointCloudSubCallbackICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
    {
    	static int i;
    	//ROS_INFO("PointCloudSubCallback");
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
    	    	boost::timer t;
    			std::cout << "Registering new point cloud using ICP" << std::endl;
    			transform_old_ = transform;
    			pcl_ros::transformPointCloud(*(pc.get()), *(pc.get()), transform);
    			pc->header.frame_id = "/map";
				if(first_)
				{
					map_ = *(pc.get());
					map_.header.frame_id="/map";
					first_ = false;
				}
				else
				{
					pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
					icp.setInputCloud(pc);
					icp.setInputTarget(map_.makeShared());
					icp.setMaximumIterations(50);
					icp.setMaxCorrespondenceDistance(0.1);
					icp.setTransformationEpsilon (1e-6);
					pcl::PointCloud<pcl::PointXYZ> pc_map_new;
					icp.align(pc_map_new);
					map_ += pc_map_new;
					std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
					ROS_INFO("Aligned PC has %d points", map_.size());
			    	ROS_INFO("\tTime: %f", t.elapsed());
				}
				point_cloud_pub_.publish(map_);
				std::stringstream ss;
				ss << "/home/goa/pcl_daten/pc_" << i << ".pcd";
				pcl::io::savePCDFileASCII (ss.str(), map_);
				i++;
    		}
    		//else
    		//	ROS_INFO("Skipped");
    	}
    	catch (tf::TransformException ex){
    		ROS_ERROR("%s",ex.what());
    	}
    }

    boost::numeric::ublas::matrix<double> transformAsMatrix(const Transform& bt)
    {
      boost::numeric::ublas::matrix<double> outMat(4,4);

      //  double * mat = outMat.Store();

      double mv[12];
      bt.getBasis().getOpenGLSubMatrix(mv);

      Vector3 origin = bt.getOrigin();

      outMat(0,0)= mv[0];
      outMat(0,1)  = mv[4];
      outMat(0,2)  = mv[8];
      outMat(1,0)  = mv[1];
      outMat(1,1)  = mv[5];
      outMat(1,2)  = mv[9];
      outMat(2,0)  = mv[2];
      outMat(2,1)  = mv[6];
      outMat(2,2) = mv[10];

      outMat(3,0)  = outMat(3,1) = outMat(3,2) = 0;
      outMat(0,3) = origin.x();
      outMat(1,3) = origin.y();
      outMat(2,3) = origin.z();
      outMat(3,3) = 1;


      return outMat;
    };


    void transformPointCloud(const std::string & target_frame, const Transform& net_transform,
    						const ros::Time& target_time, const sensor_msgs::PointCloud2 & cloudIn, sensor_msgs::PointCloud2 & cloudOut)
    {
    	boost::numeric::ublas::matrix<double> transform = transformAsMatrix(net_transform);

		int x_offset = 0, y_offset = 0, z_offset = 0;
		for (size_t d = 0; d < cloudIn.fields.size(); ++d)
		{
			if(cloudIn.fields[d].name == "x")
				x_offset = cloudIn.fields[d].offset;
			if(cloudIn.fields[d].name == "y")
				y_offset = cloudIn.fields[d].offset;
			if(cloudIn.fields[d].name == "z")
				z_offset = cloudIn.fields[d].offset;
		}

      unsigned int length = cloudIn.height*cloudIn.width;

      boost::numeric::ublas::matrix<float> matIn(4, length);

      float * matrixPtr = matIn.data().begin();

      for (unsigned int i = 0; i < length ; i++)
      {
    	  memcpy(&matrixPtr[i], &cloudIn.data[i * cloudIn.point_step + x_offset], sizeof(float));
    	  memcpy(&matrixPtr[i+length], &cloudIn.data[i * cloudIn.point_step + y_offset], sizeof(float));
    	  memcpy(&matrixPtr[i+ 2* length], &cloudIn.data[i * cloudIn.point_step + z_offset], sizeof(float));
    	  matrixPtr[i+ 3* length] = 1;
      }

      boost::numeric::ublas::matrix<float> matOut = prod(transform, matIn);

      // Copy relevant data from cloudIn, if needed
      if (&cloudIn != &cloudOut)
      {
			cloudOut.header =  cloudIn.header;
			cloudOut.fields =  cloudIn.fields;
			cloudOut.width =  cloudIn.width;
			cloudOut.height =  cloudIn.height;
			cloudOut.is_bigendian =  cloudIn.is_bigendian;
			cloudOut.point_step =  cloudIn.point_step;
			cloudOut.row_step =  cloudOut.point_step * cloudOut.width;
			cloudOut.is_dense =  cloudIn.is_dense;
			cloudOut.data = cloudIn.data;
      }

      matrixPtr = matOut.data().begin();

      //Override the positions
      cloudOut.header.stamp = target_time;
      cloudOut.header.frame_id = target_frame;
      for (unsigned int i = 0; i < length ; i++)
      {
    	  memcpy(&cloudOut.data[i * cloudOut.point_step + x_offset], &matrixPtr[i], sizeof(float));
    	  memcpy(&cloudOut.data[i * cloudOut.point_step + y_offset], &matrixPtr[i+length], sizeof(float));
    	  memcpy(&cloudOut.data[i * cloudOut.point_step + z_offset], &matrixPtr[i+2*length], sizeof(float));
      };
    }


    ros::NodeHandle n_;


protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;

    TransformListener tf_listener_;
    StampedTransform transform_old_;

    pcl::PointCloud<pcl::PointXYZ> map_;

    bool first_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, AggregatePointMap, AggregatePointMap, nodelet::Nodelet)

