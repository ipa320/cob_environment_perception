#pragma once

#include <cob_3d_mapping_geometry_map_v2/types/context.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>

class GeometryNode {
	ros::NodeHandle nh_;
	cob_3d_geometry_map::GlobalContext ctxt_;

	void callback(const cob_3d_mapping_msgs::PlaneSceneConstPtr& scene, const sensor_msgs::ImageConstPtr& color_img)
	{
	  ROS_INFO("callback");
	  
	  ctxt_.add_scene(*scene);
	}
	
	typedef message_filters::sync_policies::ApproximateTime<cob_3d_mapping_msgs::PlaneScene, sensor_msgs::Image> TSyncPolicy;
	
	boost::shared_ptr<message_filters::Subscriber<cob_3d_mapping_msgs::PlaneScene> > sub_scene_;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_col_img_;
	boost::shared_ptr<message_filters::Synchronizer<TSyncPolicy> > sync_;
	
public:
	GeometryNode() {
		 sub_scene_.reset(new message_filters::Subscriber<cob_3d_mapping_msgs::PlaneScene>(nh_, "scene", 1));
		 sub_col_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "color_image", 1));
		 
		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
		sync_.reset(new message_filters::Synchronizer<TSyncPolicy>(TSyncPolicy(10), *sub_scene_, *sub_col_img_));
		sync_->registerCallback(boost::bind(&GeometryNode::callback, this, _1, _2));
	}


};
