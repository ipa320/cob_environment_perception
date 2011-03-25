
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/point_types.h"
#include <pcl_ros/pcl_nodelet.h>



class CloudToCloud2Converter : public pcl_ros::PCLNodelet
{
private:
	CloudToCloud2Converter()
	{
		sensor_msgs::PointCloud2 pc2;

	}

	 ~CloudToCloud2Converter()
	    {
	    	/// void
	    }


void onInit()
    {
    	PCLNodelet::onInit();
    	n_ = getNodeHandle();

		point_cloud_sub_ = n_.subscribe("point_cloud1",&CloudToCloud2Converter::Converter, 1,this);
		point_cloud_pub_= n_.advertise<sensor_msgs::PointCloud2>("point_cloud2",1);



    }


void Converter(const sensor_msgs::PointCloud & pc1 )
{
	sensor_msgs::convertPointCloudToPointCloud2(sensor_msgs::PointCloud & pc1 ,sensor_msgs::PointCloud2 & pc2);
	point_cloud_pub_.publish(pc2);
}
ros::NodeHandle n_;


protected:
ros::Subscriber point_cloud_sub_;
ros::Publisher point_cloud_pub_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, CloudToCloud2Converter, CloudToCloud2Converter, nodelet::Nodelet)

