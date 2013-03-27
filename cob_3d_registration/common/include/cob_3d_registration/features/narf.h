/*
 * narf.h
 *
 *  Created on: Nov 15, 2011
 *      Author: goa-jh
 */

#ifndef NARF_H_
#define NARF_H_

#include "pcl/range_image/range_image.h"
#include "pcl/features/range_image_border_extractor.h"
#include "pcl/keypoints/narf_keypoint.h"
#include "pcl/features/narf_descriptor.h"
#include <pcl/filters/extract_indices.h>


template<typename Point>
class Feature_NARF : public FeatureContainerInterface_Euclidean<Point>
{
  pcl::PointCloud<pcl::Narf36> feature_in_, feature_out_;

public:
  static void extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<pcl::Narf36> &narf_descriptors);

  const pcl::PointCloud<Point> &getFilteredInputCloud() {return this->org_in_;}
  const pcl::PointCloud<Point> &getFilteredOutputCloud() {return this->org_out_;}

  virtual bool hidden_build() {

    ROS_INFO("calc narf for source %d", (int)this->org_in_.size());
    {
      extractFeatures(this->org_in_, feature_in_);
      this->org_in_.points.clear();
      for(int i=0; i<(int)feature_in_.size(); i++) {
        Point p;
        memset(&p,0,sizeof(Point));
        p.x=feature_in_.points[i].x;
        p.y=feature_in_.points[i].y;
        p.z=feature_in_.points[i].z;
        this->org_in_.points.push_back(p);
      }
      this->org_in_.height=1;
      this->org_in_.width=this->org_in_.size();
    }

    ROS_INFO("calc narf for target %d", (int)this->org_out_.size());
    {
      extractFeatures(this->org_out_, feature_out_);
      this->org_out_.points.clear();
      for(int i=0; i<(int)feature_out_.size(); i++) {
        Point p;
        memset(&p,0,sizeof(Point));
        p.x=feature_out_.points[i].x;
        p.y=feature_out_.points[i].y;
        p.z=feature_out_.points[i].z;
        this->org_out_.points.push_back(p);
      }
      this->org_out_.height=1;
      this->org_out_.width=this->org_out_.size();
    }

    //need to rebuild tree
    //ROS_INFO("build tree %d %d", this->org_in_.size(), this->org_out_.size());
#ifdef PCL_VERSION_COMPARE
    this->tree_.reset (new pcl::search::KdTree<Point>);
#else
	this->tree_.reset (new pcl::KdTreeFLANN<Point>);
#endif
    this->tree_->setInputCloud(this->org_in_.makeShared());
    //ROS_INFO("build tree done");

    return true;
  }

  virtual Eigen::VectorXf getFeatureOut(const int index) {
    Eigen::VectorXf b;
    b.resize(36);
    for(int i=0; i<36; i++) b(i) = feature_out_.points[index].descriptor[i];
    return b;
  }
  virtual Eigen::VectorXf getFeatureIn(const int index) {
    Eigen::VectorXf b;
    b.resize(36);
    for(int i=0; i<36; i++) b(i) = feature_in_.points[index].descriptor[i];
    return b;
  }
};

#include "impl/narf.hpp"

#endif /* NARF_H_ */
