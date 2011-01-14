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
 * Create ROS parameters for filter
 * Write as nodelet
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
#include <pcl_ros/subscriber.h>
#include <pcl_ros/publisher.h>
#include <pcl/io/io.h>
#include <tf/transform_listener.h>

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
	  :	n_(nh)
	{
		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &AggregatePointMap::pointCloudSubCallback, this);
		point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("point_cloud2_map",1);
	}


    // Destructor
    ~AggregatePointMap()
    {
    	/// void
    }

    void pointCloudSubCallback(const sensor_msgs::PointCloud2Ptr& pc)
    {
    	ROS_INFO("PointCloudSubCallback");

    	StampedTransform transform;
    	try{
    		tf_listener_.lookupTransform("/map", pc->header.frame_id, ros::Time(0), transform);
    		transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
    	}
    	catch (tf::TransformException ex){
    		ROS_ERROR("%s",ex.what());
    	}
    	//TODO: get new version of PCL
    	//pcl::concatenatePointCloud (map_, *(pc.get()), map_);

		point_cloud_pub_.publish(map_);
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

    sensor_msgs::PointCloud2 map_;

    bool filter_by_amplitude_;
    bool filter_tearoff_;
    bool filter_speckle_;
    bool filter_by_confidence_;
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

