/*
 * JumpEdge_filter.cpp
 *
 *  Created on: Mar 2, 2011
 *      Author: goa-wq
 */
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
#include "pcl/point_types.h"
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/cpc_point.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include "pcl/impl/instantiate.hpp"

//#######################
//#### nodelet class ####
class JumpEdgeFilter : public pcl_ros::PCLNodelet
{
public:
	// Constructor
	JumpEdgeFilter()
	{
		t_check=1;
	}

	// Destructor
	~JumpEdgeFilter()
	{
		/// void
	}

	void onInit()
	{
		PCLNodelet::onInit();
		n_ = getNodeHandle();

		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &JumpEdgeFilter::PointCloudSubCallback, this);
		point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("point_cloud2_filtered",1);

		ROS_INFO("Applying JumpEdgeEdges Filter");
		//n_.param("/JumpEdge_filter_nodelet/filter_JumpEdge", filter_JumpEdge_,false);
		//std::cout << "filter JumpEdge: " << filter_JumpEdge_<< std::endl;
	}

	void PointCloudSubCallback(const pcl::PointCloud<CPCPoint>::Ptr& pc)
	{
		//ROS_INFO("PointCloudSubCallback");
		pcl::PointCloud<CPCPoint>::Ptr pc_out(new pcl::PointCloud<CPCPoint>());
		pc_out->points.resize(pc->points.size());
		pc_out->header = pc->header;

		FilterJumpEdges(pc, pc_out);

		point_cloud_pub_.publish(pc_out);
		if (t_check==1)
		{
			ROS_INFO("Time elapsed (JumpEdgeEdges_Filter) : %f", t.elapsed());
			t.restart();
			t_check=0;
		}
	}

    void FilterJumpEdges(const pcl::PointCloud<CPCPoint>::Ptr& pc, const pcl::PointCloud<CPCPoint>::Ptr& pc_out)
    {
    	pcl::PointIndices::Ptr points_to_remove (new pcl::PointIndices ());
    	double upper_angle_thresh = 170.0/180*M_PI;
    	double lower_angle_thresh = (180-170.0)/180*M_PI;
    	for( unsigned int i = 0; i < pc->points.size(); i++)
    	{
    		if(i< pc->width || i%pc->width==0 || i%pc->width==3 || i>pc->width*(pc->height-1)) continue; //skip border points
    		Eigen::Vector3f v_m(pc->points[i].x,pc->points[i].y,pc->points[i].z);
    		Eigen::Vector3f v_m_n = v_m.normalized();
    		int index = i-pc->width-1;
    		Eigen::Vector3f vd_ul(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		vd_ul.normalize();
    		double angle = std::acos(v_m_n.dot(vd_ul));
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		continue;
    		}
    		index = i-pc->width;
    		Eigen::Vector3f vd_u(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		vd_u.normalize();
    		angle = std::acos(v_m_n.dot(vd_u));
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
    			continue;
    		}
    	    index = i-pc->width+1;
    		Eigen::Vector3f vd_ur(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		vd_ur.normalize();
    		angle = std::acos(v_m_n.dot(vd_ur));
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		continue;
    		}
    	    index = i-1;
    		Eigen::Vector3f vd_l(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		vd_l.normalize();
    		angle = std::acos(v_m_n.dot(vd_l));
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		continue;
    		}
    	    index = i+1;
    		Eigen::Vector3f vd_r(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		vd_r.normalize();
    		angle = std::acos(v_m_n.dot(vd_r));
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		continue;
    		}
    	    index = i+pc->width-1;
    		Eigen::Vector3f vd_ll(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		vd_ll.normalize();
    		angle = std::acos(v_m_n.dot(vd_ll));
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		continue;
    		}
    	    index = i+pc->width;
    		Eigen::Vector3f vd_lo(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		vd_lo.normalize();
    		angle = std::acos(v_m_n.dot(vd_lo));
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		continue;
    		}
    	    index = i+pc->width+1;
    		Eigen::Vector3f vd_lr(v_m(0)-pc->points[index].x, v_m(1)-pc->points[index].y, v_m(2)-pc->points[index].z);
    		vd_lr.normalize();
    		angle = std::acos(v_m_n.dot(vd_lr));
    		if(angle > upper_angle_thresh || angle < lower_angle_thresh)
    		{
    			points_to_remove->indices.push_back(i);
        		continue;
    		}
    	}
		pcl::ExtractIndices<CPCPoint> extractIndices;
		extractIndices.setInputCloud(pc);
		extractIndices.setIndices(points_to_remove);
		extractIndices.setNegative(true);
		extractIndices.filter(*(pc_out.get()));
    }

	ros::NodeHandle n_;
	boost::timer t;
	bool t_check;

protected:
	ros::Subscriber point_cloud_sub_;
	ros::Publisher point_cloud_pub_;

	//bool filter_JumpEdge_;
};

using namespace pcl;
PCL_INSTANTIATE(ExtractIndices, (CPCPoint));

PLUGINLIB_DECLARE_CLASS(cob_env_model, JumpEdgeFilter, JumpEdgeFilter, nodelet::Nodelet)

