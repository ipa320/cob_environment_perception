#pragma once

#include <pcl/point_types.h>
#include <pcl/correspondence.h>

//http://kfu.googlecode.com/svn/trunk/pcl/src/tools/test_registration.cpp

class VRegistration {
	typedef pcl::PointXYZ Point;
	
	pcl::PointCloud<Point>::Ptr pcA_, pcB_;
	pcl::Correspondences correspondeces_;
	
public:
	VRegistration():
		pcA_(new pcl::PointCloud<Point>), pcB_(new pcl::PointCloud<Point>)
	{}
	
	void add(const Point &ptA, const Point &ptB) {
		const float dist = (ptA.getVector3fMap()-ptB.getVector3fMap()).norm();
		//ROS_INFO("dist %f", dist);
		correspondeces_.push_back(pcl::Correspondence(pcA_->size(), pcB_->size(), dist));
		pcA_->push_back(ptA);
		pcB_->push_back(ptB);
	}
	
	Eigen::Matrix4f computeTF(bool *result=NULL) const;
};

#include "impl/registration.hpp"
