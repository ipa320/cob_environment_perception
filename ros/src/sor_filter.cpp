/*
 * sor_filter.cpp
 *
 *  Created on: Feb 11, 2011
 *      Author: goa-wq
 */
//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>
#include <pcl/io/io.h>
#include "pcl/point_types.h"
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/filters/statistical_outlier_removal.h"


//####################
//#### node class ####
class SORFilter
{
public:
    // Constructor
	 SORFilter(const ros::NodeHandle& nh)
	  :	n_(nh),
		statistical_filter_(false)
	{

		point_cloud_sub_sor_ = n_.subscribe("point_cloud2", 1, &SORFilter::PointCloudSubCallback, this);
		point_cloud_pub_sor_ =  n_.advertise<sensor_msgs::PointCloud2>("point_cloud2_filtered",1);
		//timer = nh.createTimer(ros::Duration(1), timerCallback);
		n_.getParam("/cob_env_model/sor_filter/statistical_outlier_filter",statistical_filter_);
		std::cout<<"Statistical Outlier Removal filter :"<<statistical_filter_<<std::endl;

	}


    // Destructor
    ~SORFilter()
    {
    	/// void
    }
    void PointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        if(statistical_filter_)
        {
        	//timer.start();
        	FilterStatisticalOutlierRemoval(cloud);
        	point_cloud_pub_sor_.publish(cloud);
        	ROS_INFO("\tTime: %f", t.elapsed());
        	t.restart();
        }
    }

    void FilterStatisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {

    	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    	sor.setInputCloud (cloud);
    	sor.setMeanK (30);
    	sor.setStddevMulThresh (1.0);
    	sor.filter(*cloud);
    }
    ros::NodeHandle n_;
    boost::timer t;


protected:
    ros::Subscriber point_cloud_sub_sor_;
    ros::Publisher point_cloud_pub_sor_;



    bool statistical_filter_;

};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{

	/// initialize ROS, specify name of node
	ros::init(argc, argv, "sor_filter");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	/// Create camera node class instance
	SORFilter sor_filter(nh);

	ros::spin();

	return 0;
}


