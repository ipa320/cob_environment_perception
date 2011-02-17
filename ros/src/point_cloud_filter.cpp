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

#include <opencv/cv.h>
#include <opencv/highgui.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>
#include <cob_vision_utils/VisionUtils.h>
#include "pcl/filters/statistical_outlier_removal.h"
 #include "pcl/point_types.h"

//####################
//#### node class ####
class PointCloudFilter
{
public:
    // Constructor
	PointCloudFilter(const ros::NodeHandle& nh)
	  :	n_(nh),
	    filter_by_amplitude_(false),
	    filter_tearoff_(false),
	    filter_speckle_(false),
	    filter_by_confidence_(false)
	{
		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &PointCloudFilter::PointCloudSubCallback, this);
		point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("point_cloud2_filtered",1);
		//filter_speckle_ = true;
		n_.getParam("/cob_env_model/point_cloud_filter/filter_by_amplitude", filter_by_amplitude_);
		std::cout << "filter by amplitude: " << filter_by_amplitude_ << std::endl;
		n_.getParam("/cob_env_model/point_cloud_filter/filter_tearoff", filter_tearoff_);
		std::cout << "filter tearoff: " << filter_tearoff_<< std::endl;
		n_.getParam("/cob_env_model/point_cloud_filter/filter_speckle", filter_speckle_);
		std::cout << "filter speckle: " << filter_speckle_ << std::endl;
		n_.getParam("/cob_env_model/point_cloud_filter/filter_by_confidence", filter_by_confidence_);
		std::cout << "filter by confidence: " << filter_by_confidence_<< std::endl;
		n_.getParam("/cob_env_model/point_cloud_filter/statistical_outlier_filter",statistical_filter_);
		std::cout<<"Statistical Outlier Removal filter :"<<statistical_filter_<<std::endl;

	}


    // Destructor
    ~PointCloudFilter()
    {
    	/// void
    }

    void PointCloudSubCallback(const sensor_msgs::PointCloud2Ptr& pc)
    {
    	//ROS_INFO("PointCloudSubCallback");
		cv::Mat xyz_mat_32F3 = cv::Mat(pc->height, pc->width, CV_32FC3);
		cv::Mat intensity_mat_32F1 = cv::Mat(pc->height, pc->width, CV_32FC1);

		//assumption: data order is always x->y->z in PC, datatype = float
		int x_offset = 0, i_offset = 0;
		for (size_t d = 0; d < pc->fields.size(); ++d)
		{
			if(pc->fields[d].name == "x")
				x_offset = pc->fields[d].offset;
			if(pc->fields[d].name == "intensity")
				i_offset = pc->fields[d].offset;
		}

		float* f_ptr = 0;
		float* i_ptr = 0;
		int pc_msg_idx=0;
		for (int row = 0; row < xyz_mat_32F3.rows; row++)
		{
			f_ptr = xyz_mat_32F3.ptr<float>(row);
			i_ptr = intensity_mat_32F1.ptr<float>(row);
			for (int col = 0; col < xyz_mat_32F3.cols; col++, pc_msg_idx++)
			{
				memcpy(&f_ptr[3*col], &pc->data[pc_msg_idx * pc->point_step + x_offset], 3*sizeof(float));
				memcpy(&i_ptr[col], &pc->data[pc_msg_idx * pc->point_step + i_offset], sizeof(float));
			}
		}
		///////////////////////////////////////////////////////////////////
				std::cout<<" pc height::"<<pc->height;

				std::cout<<" pc width ::"<<pc->width;
				std::cout<<" pc ::"<<pc<<std::endl;
				std::cout<<" pc data ::"<<(int)xyz_mat_32F3.flags<<std::endl;
		////////////////////////////////////////////////////////////////////
		if(filter_speckle_ || filter_by_amplitude_ || filter_tearoff_)
		{
			if(filter_by_amplitude_) FilterByAmplitude(xyz_mat_32F3, intensity_mat_32F1);
			if(filter_speckle_) FilterSpeckles(xyz_mat_32F3);
			if(filter_tearoff_) FilterTearOffEdges(xyz_mat_32F3);
			pc_msg_idx=0;
			for (int row = 0; row < xyz_mat_32F3.rows; row++)
			{
				f_ptr = xyz_mat_32F3.ptr<float>(row);
				for (int col = 0; col < xyz_mat_32F3.cols; col++, pc_msg_idx++)
				{
					memcpy(&pc->data[pc_msg_idx * pc->point_step + x_offset], &f_ptr[3*col], 3*sizeof(float));
				}
			}
		}
		if(filter_by_confidence_) FilterByConfidence(pc);
		if(statistical_filter_) FilterStatisticalOutlierRemoval(xyz_mat_32F3);

		point_cloud_pub_.publish(pc);
    }

    void FilterSpeckles(cv::Mat xyz_mat_32F3)
    {
    	cv::Mat buf;
    	ipa_Utils::FilterSpeckles(xyz_mat_32F3, 50,0.1, buf);
    }

    void FilterByAmplitude(cv::Mat xyz_mat_32F3, cv::Mat intensity_mat_32F1)
    {
    	ipa_Utils::FilterByAmplitude(xyz_mat_32F3, intensity_mat_32F1, 0, 0, 1000, 60000);
    }

    void FilterTearOffEdges(cv::Mat xyz_mat_32F3)
    {
    	ipa_Utils::FilterTearOffEdges(xyz_mat_32F3, 0, 6);
    }


    void FilterByConfidence(const sensor_msgs::PointCloud2Ptr& pc)
    {
		//assumption: data order is always x->y->z in PC, datatype = float
		int x_offset = 0, c_offset = 0;
		for (size_t d = 0; d < pc->fields.size(); ++d)
		{
			if(pc->fields[d].name == "x")
				x_offset = pc->fields[d].offset;
			if(pc->fields[d].name == "confidence")
				c_offset = pc->fields[d].offset;
		}
		float c_ptr;
		float f_ptr[3];
		f_ptr[0] = f_ptr[1] = f_ptr[2] = 0;
		for (int pc_msg_idx = 0; pc_msg_idx < pc->height*pc->width; pc_msg_idx++)
		{
			memcpy(&c_ptr, &pc->data[pc_msg_idx * pc->point_step + c_offset], sizeof(float));
			if(*(float*)&pc->data[pc_msg_idx * pc->point_step + c_offset] < 55000)
			{
				memcpy(&pc->data[pc_msg_idx * pc->point_step + x_offset], &f_ptr, 3*sizeof(float));
			}
		}
    }
    void FilterStatisticalOutlierRemoval(cv::Mat xyz_mat_32F3)
    {
    	/*int i;
    	for(i=0;i<)
    	pcl::StatisticalOutlierRemoval<xyz_mat_32F3>::applyFilter(PointCloud2 &  output);

    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ()), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
    		  // Fill in the cloud data
    		  pcl::PCDReader reader;
    		  reader.read<pcl::PointXYZ> ("data/table_scene_lms400.pcd", *cloud);

    		  std::cerr << "Cloud before filtering: " << std::endl;
    		  std::cerr << *cloud << std::endl;

    		  // Create the filtering object
    		  pcl::StatisticalOutlierRemoval<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &output)

    		  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    		  sor.setInputCloud (cloud);
    		  sor.setMeanK (50);
    		  sor.setStddevMulThresh (1.0);
    		  sor.filter (*cloud_filtered);

    		  std::cerr << "Cloud after filtering: " << std::endl;
    		  std::cerr << *cloud_filtered << std::endl;

    		  pcl::PCDWriter writer;
    		  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    		  sor.setNegative (true);
    		  sor.filter (*cloud_filtered);
    		  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
*/
    }
    ros::NodeHandle n_;


protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;

    bool filter_by_amplitude_;
    bool filter_tearoff_;
    bool filter_speckle_;
    bool filter_by_confidence_;
    bool statistical_filter_;
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
	PointCloudFilter point_cloud_filter(nh);

	ros::spin();

	return 0;
}


