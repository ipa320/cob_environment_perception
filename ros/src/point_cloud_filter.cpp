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
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/cpc_point.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include "pcl/impl/instantiate.hpp"

//####################
//#### node class ####
class PointCloudFilter : public pcl_ros::PCLNodelet
{
public:
    // Constructor
	PointCloudFilter()
	  :	filter_by_amplitude_(false),
	    filter_tearoff_(false),
	    filter_speckle_(false),
	    filter_by_confidence_(false)

	{
	}


    // Destructor
    ~PointCloudFilter()
    {
    	/// void
    }

    void onInit()
    {
    	PCLNodelet::onInit();
    	n_ = getNodeHandle();

		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &PointCloudFilter::PointCloudSubCallback, this);
		point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("point_cloud2_filtered",1);
		//filter_speckle_ = true;
		n_.param("/point_cloud_filter/filter_by_amplitude", filter_by_amplitude_, false);
		std::cout << "filter by amplitude: " << filter_by_amplitude_ << std::endl;
		n_.param("/point_cloud_filter/filter_tearoff", filter_tearoff_, false);
		std::cout << "filter tearoff: " << filter_tearoff_<< std::endl;
		n_.param("/point_cloud_filter/filter_speckle", filter_speckle_, false);
		std::cout << "filter speckle: " << filter_speckle_ << std::endl;
		n_.param("/point_cloud_filter/filter_by_confidence", filter_by_confidence_, false);
		std::cout << "filter by confidence: " << filter_by_confidence_<< std::endl;
		n_.param("/point_cloud_filter/amplitude_min_threshold", amplitude_min_threshold_, 1000);
		std::cout << "amplitude_min_threshold: " << amplitude_min_threshold_<< std::endl;
		n_.param("/point_cloud_filter/amplitude_max_threshold", amplitude_max_threshold_, 60000);
    }

    void PointCloudSubCallback(const pcl::PointCloud<CPCPoint>::Ptr& pc)
        {
        	//ROS_INFO("PointCloudSubCallback");
    		pcl::PointCloud<CPCPoint>::Ptr pc_out(new pcl::PointCloud<CPCPoint>());
    		pcl::PointCloud<CPCPoint>::Ptr pc_out2(new pcl::PointCloud<CPCPoint>());
    		//pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZ>());
    		pc_out2->points.resize(pc->points.size());
    		pc_out2->header = pc->header;

    		cv::Mat xyz_mat_32F3 = cv::Mat(pc->height, pc->width, CV_32FC3);
    		cv::Mat intensity_mat_32F1 = cv::Mat(pc->height, pc->width, CV_32FC1);

    		//assumption: data order is always x->y->z in PC, datatype = float
    		/*int x_offset = 0, i_offset = 0;
    		for (size_t d = 0; d < pc->fields.size(); ++d)
    		{
    			if(pc->fields[d].name == "x")
    				x_offset = pc->fields[d].offset;
    			if(pc->fields[d].name == "intensity")
    				i_offset = pc->fields[d].offset;
    		}*/

    		float* f_ptr = 0;
    		float* i_ptr = 0;
    		int pc_msg_idx=0;
    		for (int row = 0; row < xyz_mat_32F3.rows; row++)
    		{
    			f_ptr = xyz_mat_32F3.ptr<float>(row);
    			i_ptr = intensity_mat_32F1.ptr<float>(row);
    			for (int col = 0; col < xyz_mat_32F3.cols; col++, pc_msg_idx++)
    			{
    				//memcpy(&f_ptr[3*col], &pc->data[pc_msg_idx * pc->point_step + x_offset], 3*sizeof(float));
    				//memcpy(&i_ptr[col], &pc->data[pc_msg_idx * pc->point_step + i_offset], sizeof(float));
    				//memcpy(&f_ptr[3*col], &pc->points[pc_msg_idx].x, 3*sizeof(float));
    				//memcpy(&i_ptr[col], &pc->points[pc_msg_idx].intensity, sizeof(float));
    			}
    		}

    		if(filter_speckle_ || filter_tearoff_)
    		{
    			if(filter_speckle_) FilterSpeckles(xyz_mat_32F3);
    			//if(filter_tearoff_) FilterTearOffEdges(xyz_mat_32F3);
    			if(filter_tearoff_) FilterTearOffEdges(pc, pc_out);
    			pc_msg_idx=0;
    			for (int row = 0; row < xyz_mat_32F3.rows; row++)
    			{
    				f_ptr = xyz_mat_32F3.ptr<float>(row);
    				for (int col = 0; col < xyz_mat_32F3.cols; col++, pc_msg_idx++)
    				{
    					memcpy(&pc->points[pc_msg_idx].x, &f_ptr[3*col], 3*sizeof(float));
    				}
    			}
    		}
			if(filter_by_amplitude_) FilterByAmplitude(pc_out, pc_out2);
    		//if(filter_by_confidence_) FilterByConfidence(pc);


			//std::cout << pc_out2->size() << " points in pc_out" << std::endl;
    		point_cloud_pub_.publish(pc_out2);

        }



    void FilterSpeckles(cv::Mat xyz_mat_32F3)
    {
    	cv::Mat buf;
    	ipa_Utils::FilterSpeckles(xyz_mat_32F3, 50,0.1, buf);
    	ROS_INFO("\tTime (FilterSpeckles) : %f", t.elapsed());
    	t.restart();
    }

    void FilterByAmplitude(const pcl::PointCloud<CPCPoint>::Ptr& pc, const pcl::PointCloud<CPCPoint>::Ptr& pc_out)
    {
    	//ROS_INFO("Filter by amplitude");
		//assumption: data order is always x->y->z in PC, datatype = float
		/*int x_offset = 0, c_offset = 0;
		for (size_t d = 0; d < pc->fields.size(); ++d)
		{
			if(pc->fields[d].name == "x")
				x_offset = pc->fields[d].offset;
			if(pc->fields[d].name == "intensity")
				c_offset = pc->fields[d].offset;
		}*/
		int nr_p = 0;
    	for( unsigned int i = 0; i < pc->points.size(); i++)
    	{
    		if( pc->points[i].intensity > amplitude_min_threshold_ && pc->points[i].intensity < amplitude_max_threshold_ && pc->points[i].confidence > 64000)
    		{
    			pc_out->points[nr_p++] = pc->points[i];
    		}
    		//else
    		//	pts_to_remove->indices.push_back(i);
    	}
		/*float c_ptr;
		float f_ptr[3];
		f_ptr[0] = f_ptr[1] = 0;
		f_ptr[2] = -5;
		for ( unsigned int pc_msg_idx = 0; pc_msg_idx < pc->height*pc->width; pc_msg_idx++)
		{
			//memcpy(&c_ptr, &pc->data[pc_msg_idx * pc->point_step + c_offset], sizeof(float));
			if(*(float*)&pc->data[pc_msg_idx * pc->point_step + c_offset] > 1000 && *(float*)&pc->data[pc_msg_idx * pc->point_step + c_offset] < 60000)
			{
				memcpy(&pc_out->data[pc_msg_idx * pc->point_step], &pc->data[pc_msg_idx * pc->point_step], 3*sizeof(float));
				nr_p++;
			}
		}*/
		pc_out->width = nr_p;
		pc_out->height = 1;
		pc_out->points.resize(nr_p);
		pc_out->is_dense = true;
    }

    void FilterTearOffEdges(const pcl::PointCloud<CPCPoint>::Ptr& pc, const pcl::PointCloud<CPCPoint>::Ptr& pc_out/*const pcl::PointIndices::Ptr& points_to_remove*/)
    {
    	//ipa_Utils::FilterTearOffEdges(xyz_mat_32F3, 0, 6);
    	boost::timer t;
    	pcl::PointIndices::Ptr points_to_remove (new pcl::PointIndices ());
    	double upper_angle_thresh = 170.0/180*M_PI;
    	double lower_angle_thresh = (180-170.0)/180*M_PI;
    	for( unsigned int i = 0; i < pc->points.size(); i++)
    	{
    		if(i< pc->width || i%pc->width==0 || i%pc->width==3 || i>pc->width*(pc->height-1)) continue; //skip border points
    		Eigen::Vector3f v_m(pc->points[i].x,pc->points[i].y,pc->points[i].z);
    		Eigen::Vector3f v_m_n = v_m.normalized();
    		//if(i==21037)std::cout << "v_m: " << v_m << std::endl;
    		int index = i-pc->width-1;
    		//if(i==21037)std::cout << "v_ul: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    		Eigen::Vector3f vd_ul(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		//if(i==21037)std::cout << "vd_ul: " << vd_ul << std::endl;
    		vd_ul.normalize();
    		double angle = std::acos(v_m_n.dot(vd_ul));
    		//if(i==21037)std::cout << "\na: " << angle*180/M_PI << std::endl;
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		//if(i==24660) std::cout << i << " a: " << angle << ", xyz: " << v_m <<  ", xyz_ul: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    			continue;
    		}
    		index = i-pc->width;
    		Eigen::Vector3f vd_u(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		//if(i==21037)std::cout << "v_u: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    		vd_u.normalize();
    		angle = std::acos(v_m_n.dot(vd_u));
    		//if(i==21037)std::cout << "a: " << angle*180/M_PI << std::endl;
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		//std::cout << i << " angle: " << angle << std::endl;
    			continue;
    		}
    	    index = i-pc->width+1;
    		Eigen::Vector3f vd_ur(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		//if(i==21037)std::cout << "v_ur: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    		vd_ur.normalize();
    		angle = std::acos(v_m_n.dot(vd_ur));
    		//if(i==21037)std::cout << "a: " << angle*180/M_PI << std::endl;
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		//std::cout << i << " angle: " << angle << std::endl;
    			continue;
    		}
    	    index = i-1;
    		Eigen::Vector3f vd_l(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		//if(i==21037)std::cout << "v_l: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    		vd_l.normalize();
    		angle = std::acos(v_m_n.dot(vd_l));
    		//if(i==21037)std::cout << "a: " << angle*180/M_PI << std::endl;
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		//std::cout << i << " angle: " << angle << std::endl;
    			continue;
    		}
    	    index = i+1;
    		Eigen::Vector3f vd_r(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		//if(i==21037)std::cout << "v_r: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    		vd_r.normalize();
    		angle = std::acos(v_m_n.dot(vd_r));
    		//if(i==21037)std::cout << "a: " << angle*180/M_PI << std::endl;
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		//std::cout << i << " angle: " << angle << std::endl;
    			continue;
    		}
    	    index = i+pc->width-1;
    		Eigen::Vector3f vd_ll(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		//if(i==21037)std::cout << "v_ll: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    		vd_ll.normalize();
    		angle = std::acos(v_m_n.dot(vd_ll));
    		//if(i==21037)std::cout << "a: " << angle*180/M_PI << std::endl;
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		//std::cout << i << " angle: " << angle << std::endl;
    			continue;
    		}
    	    index = i+pc->width;
    		Eigen::Vector3f vd_lo(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		//if(i==21037)std::cout << "v_lo: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    		vd_lo.normalize();
    		angle = std::acos(v_m_n.dot(vd_lo));
    		//if(i==21037)std::cout << "a: " << angle*180/M_PI << std::endl;
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		//std::cout << i << " angle: " << angle << std::endl;
    			continue;
    		}
    	    index = i+pc->width+1;
    		Eigen::Vector3f vd_lr(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		//if(i==21037)std::cout << "v_lr: " << pc->points[index].x << "," << pc->points[index].y << "," << pc->points[index].z << std::endl;
    		vd_lr.normalize();
    		angle = std::acos(v_m_n.dot(vd_lr));
    		//if(i==21037)std::cout << "a: " << angle*180/M_PI << std::endl;
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		//std::cout << i << " angle: " << angle << std::endl;
    			continue;
    		}
    	}
		//std::cout << points_to_remove->indices.size() << " points filtered" << std::endl;
		pcl::ExtractIndices<CPCPoint> extractIndices;
		extractIndices.setInputCloud(pc);
		extractIndices.setIndices(points_to_remove);
		extractIndices.setNegative(true);
		extractIndices.filter(*(pc_out.get()));
		//std::cout << "Jump edge filter time: " << t.elapsed() << std::endl;
		/*pc_out->width = nr_p;
		pc_out->height = 1;
		pc_out->points.resize(nr_p);
		pc_out->is_dense = true;*/
    }


    void FilterByConfidence(const pcl::PointCloud<CPCPoint>::Ptr& pc)
    {
		//assumption: data order is always x->y->z in PC, datatype = float
		/*int x_offset = 0, c_offset = 0;
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
		for ( unsigned int pc_msg_idx = 0; pc_msg_idx < pc->height*pc->width; pc_msg_idx++)
		{
			memcpy(&c_ptr, &pc->data[pc_msg_idx * pc->point_step + c_offset], sizeof(float));
			if(*(float*)&pc->data[pc_msg_idx * pc->point_step + c_offset] < 55000)
			{
				memcpy(&pc->data[pc_msg_idx * pc->point_step + x_offset], &f_ptr, 3*sizeof(float));
			}
		}*/
    }

    ros::NodeHandle n_;
    boost::timer t;

protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;

    bool filter_by_amplitude_;
    bool filter_tearoff_;
    bool filter_speckle_;
    bool filter_by_confidence_;
    bool statistical_filter_;

    int amplitude_min_threshold_;
    int amplitude_max_threshold_;

};

//#######################
//#### main programm ####
//int main(int argc, char** argv)
//{
//	/// initialize ROS, specify name of node
//	ros::init(argc, argv, "point_cloud_filter");
//
//	/// Create a handle for this node, initialize node
//	ros::NodeHandle nh;
//
//	/// Create camera node class instance
//	PointCloudFilter point_cloud_filter(nh);
//
//	ros::spin();
//
//	return 0;
//}
using namespace pcl;
PCL_INSTANTIATE(ExtractIndices, (CPCPoint));

PLUGINLIB_DECLARE_CLASS(cob_env_model, PointCloudFilter, PointCloudFilter, nodelet::Nodelet)


