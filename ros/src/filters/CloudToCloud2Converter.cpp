

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>

#include "pcl/point_types.h"


// ROS includes
#include <ros/ros.h>
#include <opencv/cv.h>
//#include <opencv/highgui.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>
#include <cob_vision_utils/VisionUtils.h>
#include "pcl/point_types.h"
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/cpc_point.h>


class CloudToCloud2Converter : public pcl_ros::PCLNodelet
{

public:

	void onInit()
    {
    	PCLNodelet::onInit();
    	n_ = getNodeHandle();

		point_cloud_sub_ = n_.subscribe("point_cloud1", 1 ,&CloudToCloud2Converter::PointCloudSubCallback, this);
		point_cloud_pub_= n_.advertise<sensor_msgs::PointCloud2>("point_cloud2",1);



    }


	void PointCloudSubCallback(const sensor_msgs::PointCloud pc1 )
	{

		sensor_msgs::PointCloud2 pc2;


		sensor_msgs::convertPointCloudToPointCloud2(pc1 ,pc2);
		point_cloud_pub_.publish(pc2);
	}


	ros::NodeHandle n_ ;


protected:
ros::Subscriber point_cloud_sub_;
ros::Publisher point_cloud_pub_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, CloudToCloud2Converter, CloudToCloud2Converter, nodelet::Nodelet)



