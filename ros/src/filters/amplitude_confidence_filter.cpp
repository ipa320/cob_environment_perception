/*
 * amplitude_filter.cpp
 *
 *  Created on: Mar 02, 2011
 *      Author: goa-wq
 */

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

#include <opencv/cv.h>
//#include <opencv/highgui.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>
//#include <cob_vision_utils/VisionUtils.h>
#include "pcl/point_types.h"
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/cpc_point.h>

//####################
//#### nodelet class####
class AmplitudeConfidenceFilter : public pcl_ros::PCLNodelet
{
public:
    // Constructor
	 AmplitudeConfidenceFilter()
	  :	filter_by_amplitude_(false),
	   	filter_by_confidence_(false)
	{
		t_check=1;
		filter_info=1;
	}

    // Destructor
    ~ AmplitudeConfidenceFilter()
    {
    	/// void
    }

    void onInit()
    {
    	PCLNodelet::onInit();
    	n_ = getNodeHandle();

		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, & AmplitudeConfidenceFilter::PointCloudSubCallback, this);
		point_cloud_pub_= n_.advertise<sensor_msgs::PointCloud2>("point_cloud2_filtered",1);

		n_.param("/amplitude_confidence_filter_nodelet/filter_by_amplitude", filter_by_amplitude_, false);
	//	std::cout << "filter_by_amplitude: " << filter_by_amplitude_<< std::endl;
		n_.param("/amplitude_confidence_filter_nodelet/filter_by_confidence", filter_by_confidence_, false);
	//	std::cout << "filter_by_confidence: " << filter_by_confidence_<< std::endl;
		n_.param("/amplitude_confidence_filter_nodelet/amplitude_min_threshold", amplitude_min_threshold_, 1000);
		//std::cout << "amplitude_min_threshold: " << amplitude_min_threshold_<< std::endl;
		n_.param("/amplitude_confidence_filter_nodelet/amplitude_max_threshold", amplitude_max_threshold_, 60000);
		//std::cout << "amplitude_max_threshold: " << amplitude_max_threshold_<< std::endl;
		n_.param("/amplitude_confidence_filter_nodelet/confidence_threshold", confidence_threshold_, 60000);
		//std::cout << "confidence_threshold: " << confidence_threshold_<< std::endl;
    }

    void PointCloudSubCallback(const pcl::PointCloud<CPCPoint>::Ptr& pc)
	{
		//ROS_INFO("PointCloudSubCallback");
		pcl::PointCloud<CPCPoint>::Ptr pc_out(new pcl::PointCloud<CPCPoint>());
		pc_out->points.resize(pc->points.size());
		pc_out->header = pc->header;

		FilterByAmplitude(pc, pc_out);
		point_cloud_pub_.publish(pc_out);
		if (t_check==1)
		{
			ROS_INFO("Time elapsed (Amplitude_Confidence_Filter) : %f", t.elapsed());
			t.restart();
			t_check=0;
		}
	}

    void FilterByAmplitude(const pcl::PointCloud<CPCPoint>::Ptr& pc, const pcl::PointCloud<CPCPoint>::Ptr& pc_out)
    {
    	//ROS_INFO("Filter by amplitude");
		int nr_p = 0;

		if(filter_by_amplitude_==true && filter_by_confidence_==true)
		{
			if (filter_info==1)
			{
				ROS_INFO("Applying Amplitude and confidence Filter");
			    filter_info=0;
			}
			for( unsigned int i = 0; i < pc->points.size(); i++)
			{
				if( pc->points[i].intensity > amplitude_min_threshold_ && pc->points[i].intensity < amplitude_max_threshold_ && pc->points[i].confidence > confidence_threshold_)
				pc_out->points[nr_p++] = pc->points[i];
			}
		}

		if(filter_by_amplitude_==true && filter_by_confidence_==false)
		{
			if (filter_info==1)
			{
				ROS_INFO("Applying Amplitude Filter");
				filter_info=0;
			}
			for( unsigned int i = 0; i < pc->points.size(); i++)
			{
				if( pc->points[i].intensity > amplitude_min_threshold_ && pc->points[i].intensity < amplitude_max_threshold_)
				pc_out->points[nr_p++] = pc->points[i];
			}
    	}
		if(filter_by_amplitude_==false && filter_by_confidence_==true)
		{
			if (filter_info==1)
			{
				ROS_INFO("Applying confidence Filter");
				filter_info=0;
			}
			for( unsigned int i = 0; i < pc->points.size(); i++)
			{
				if( pc->points[i].confidence > confidence_threshold_)
				pc_out->points[nr_p++] = pc->points[i];
			}
		}

		pc_out->width = nr_p;
		pc_out->height = 1;
		pc_out->points.resize(nr_p);
		pc_out->is_dense = true;
    }

    ros::NodeHandle n_;
    boost::timer t;
    bool t_check;
    bool filter_info;

protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;

    bool filter_by_amplitude_;
    bool filter_by_confidence_;

    int amplitude_min_threshold_;
    int amplitude_max_threshold_;
    int confidence_threshold_;

};


PLUGINLIB_DECLARE_CLASS(cob_env_model,  AmplitudeConfidenceFilter,  AmplitudeConfidenceFilter, nodelet::Nodelet)


