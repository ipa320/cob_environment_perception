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

// ROS includes
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cob_vision_ipa_utils/cpc_point.h>
#include "pcl/filters/voxel_grid.h"

// ROS message includes
#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### node class ####
class AggregatePointMap
{
public:
    // Constructor
	AggregatePointMap(const ros::NodeHandle& nh)
	  :	n_(nh),
	   	first_(true)
	{
		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &AggregatePointMap::pointCloudSubCallbackICP, this);
		point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("point_cloud2_map",1);
	}


    // Destructor
    ~AggregatePointMap()
    {
    	/// void
    }

    void pointCloudSubCallback(const sensor_msgs::PointCloud2Ptr& pc)
    {
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
    		if(fabs(r-r_old) > 0.05 || fabs(p-p_old) > 0.05 || fabs(y-y_old) > 0.05 ||
    				transform.getOrigin().distance(transform_old_.getOrigin()) > 0.1)
    		{
    			std::cout << "Registering new point cloud" << std::endl;
    			transform_old_ = transform;
				transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
				if(first_)
				{
					map_ = *(pc.get());
					map_.header.frame_id="/map";
					first_ = false;
				}
				//else
					//pcl::concatenatePointCloud (map_, *(pc.get()), map_);

				point_cloud_pub_.publish(map_);
    		}
    	}
    	catch (tf::TransformException ex){
    		ROS_ERROR("%s",ex.what());
    	}
    }

    void pointCloudSubCallbackICP(const sensor_msgs::PointCloud2Ptr& pc)
    {
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
    		if(fabs(r-r_old) > 0.05 || fabs(p-p_old) > 0.05 || fabs(y-y_old) > 0.05 ||
    				transform.getOrigin().distance(transform_old_.getOrigin()) > 0.1)
    		{
    			std::cout << "Registering new point cloud using ICP" << std::endl;
    			transform_old_ = transform;
				transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
				if(first_)
				{
					map_ = *(pc.get());
					map_.header.frame_id="/map";
					first_ = false;
				}
				else
				{
					pcl::PointCloud<pcl::PointXYZ> pc_in, pc_map;
					pcl::PCDReader reader;
					//reader.read ("/home/goa/git/cob3_intern/cob_sandbox/pcl_test/common/files/cob3-2/pcd_kitchen/kitchen_01_world.pcd", pc_in);
					//reader.read ("/home/goa/git/cob3_intern/cob_sandbox/pcl_test/common/files/cob3-2/pcd_kitchen/kitchen_02_world.pcd", pc_map);
					pcl::fromROSMsg(*(pc.get()), pc_in);
					pcl::fromROSMsg(map_, pc_map);
					pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
					//pcl::VoxelGrid<CPCPoint> vox_filter;
					icp.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(pc_in));
					icp.setInputTarget(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(pc_map));
					icp.setMaximumIterations(50);
					icp.setMaxCorrespondenceDistance(0.1);
					icp.setTransformationEpsilon (1e-6);
					pcl::PointCloud<pcl::PointXYZ> pc_map_new;
					icp.align(pc_map_new);
					sensor_msgs::PointCloud2 pc_msg_map_new;
					pcl::toROSMsg(pc_map_new, pc_msg_map_new);
					ROS_INFO("Aligned PC has %d points", pc_msg_map_new.height*pc_msg_map_new.width);
					ROS_INFO("Old map has %d points", map_.height*map_.width);
					pcl::concatenatePointCloud (map_, pc_msg_map_new, map_);
					pcl::fromROSMsg(map_, pc_map);
					ROS_INFO("New map has %d points", map_.height*map_.width);
					pcl::io::savePCDFileASCII ("cloud_in.pcd", pc_in);
					pcl::io::savePCDFileASCII ("cloud_out.pcd", pc_map_new);
					pcl::io::savePCDFileASCII ("map.pcd", pc_map);
				}

				point_cloud_pub_.publish(map_);
    		}
    		else
    			ROS_INFO("Skipped");
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

    sensor_msgs::PointCloud2 map_;

    bool first_;

};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "point_cloud_filter");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	/// Create camera node class instance
	AggregatePointMap aggregate_point_map(nh);

	ros::spin();

	return 0;
}

