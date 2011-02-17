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
#include <tf_conversions/tf_kdl.h>
#include <pcl_ros/point_cloud.h>
#include <cob_vision_ipa_utils/cpc_point.h>
#include <opencv2/core/core.hpp>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

#include <cob_vision_features/SURFDetector.h>
#include <cob_vision_features/AbstractFeatureVector.h>



//####################
//#### node class ####
class ExtractFeatures
{
public:
    // Constructor
	ExtractFeatures(const ros::NodeHandle& nh)
	  :	n_(nh)
	{
		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &ExtractFeatures::pointCloudSubCallback, this);
		point_cloud_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("point_cloud2_featured",1);
	}

    // Destructor
    ~ExtractFeatures()
    {
    	/// void
    }

    void pointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
	{
    	ROS_INFO("[env_model_node] Detecting features");

    	cv::Mat color_image(pc->height,pc->width,CV_8UC3);
    	unsigned char* c_ptr = 0;
    	int pc_pt_idx=0;
    	for (int row = 0; row < color_image.rows; row++)
    	{
    		c_ptr = color_image.ptr<unsigned char>(row);
    		for (int col = 0; col < color_image.cols; col++, pc_pt_idx++)
    		{
    			memcpy(&c_ptr[3*col], &pc->points[pc_pt_idx].rgb, 3*sizeof(unsigned char));
    		}
    	}
    	ipa_Features::AbstractLocalFeatureDetector* feature_detector = new ipa_Features::SURFDetector();
    	((SURFDetector*)feature_detector)->Init(500);
    	ipa_Features::AbstractFeatureVectorPtr feature_vector = AbstractFeatureVectorPtr(new AbstractFeatureVector());
		feature_detector->DetectFeatures(color_image, *feature_vector);

		pc_pt_idx=0;
    	for (int row = 0; row < color_image.rows; row++)
    	{
    		c_ptr = color_image.ptr<unsigned char>(row);
    		for (int col = 0; col < color_image.cols; col++, pc_pt_idx++)
    		{
				AbstractFeatureVector::iterator It;
				for (It=feature_vector->begin(); It!=feature_vector->end(); It++)
				{
					if((*It)->Get<double>(M_U) == col && (*It)->Get<double>(M_V) == row)
					{
						//pc->points[pc_pt_idx].isFeature = 1;
						feature_vector->erase(It);
						break;
					}
				}
    		}
    	}
    	point_cloud_pub_.publish(pc);
	}

    ros::NodeHandle n_;

protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
};


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "extract features");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	/// Create camera node class instance
	ExtractFeatures extract_features(nh);

	ros::spin();

	return 0;
}


