/*
 * edges.h
 *
 *  Created on: Nov 18, 2011
 *      Author: goa-jh
 */

#ifndef EDGES_H_
#define EDGES_H_


#include "pcl/range_image/range_image.h"
#include "pcl/features/range_image_border_extractor.h"
#include "pcl/keypoints/narf_keypoint.h"
#include "pcl/features/narf_descriptor.h"
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <cob_3d_mapping_features/fast_edge_estimation_3d_omp.h>
#include <cob_3d_mapping_common/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/visualization/cloud_viewer.h>

namespace cob_3d_registration {

template<typename Point>
class Feature_Edges : public FeatureContainerInterface_Euclidean<Point>
{
  pcl::PointCloud<Point> feature_in_, feature_out_;

  float radius_;
  float thr_;
  float dist_threshold_;

public:
  Feature_Edges():
    radius_(0.05), thr_(0.5), dist_threshold_(0.1)
  {}

  void setRadius(float v) {radius_ = v;}
  void setThreshold(float v) {thr_ = v;}
  void setDisThreshold(float v) {dist_threshold_ = v;}

  void extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<Point> &narf_descriptors);

  const pcl::PointCloud<Point> &getFilteredInputCloud() {return this->org_in_;}
  const pcl::PointCloud<Point> &getFilteredOutputCloud() {return this->org_out_;}

  virtual bool hidden_build() {

    ROS_INFO("calc edges for source %d", this->org_in_.size());
    {
      /*extractFeatures(this->org_in_, feature_in_);
      this->org_in_.points.clear();
      for(int i=0; i<feature_in_.size(); i++) {
        Point p;
        memset(&p,0,sizeof(Point));
        p.x=feature_in_.points[i].x;
        p.y=feature_in_.points[i].y;
        p.z=feature_in_.points[i].z;
        this->org_in_.points.push_back(p);
      }
      this->org_in_.height=1;
      this->org_in_.width=this->org_in_.size();*/
      feature_in_=this->org_in_;
    }

    ROS_INFO("calc edges for target %d", this->org_out_.size());
    {
      extractFeatures(this->org_out_, feature_out_);
      this->org_out_.points.clear();
      for(int i=0; i<feature_out_.size(); i++) {
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
    ROS_INFO("build tree %d %d", this->org_in_.size(), this->org_out_.size());
    this->tree_.reset (new pcl::KdTreeFLANN<Point>);
    if(this->org_in_.size()>0) this->tree_->setInputCloud(this->org_in_.makeShared());
    ROS_INFO("build tree done");

    return true;
  }

  virtual Eigen::VectorXf getFeatureOut(const int index) {
    return this->org_out_.points[index].getVector3fMap();
  }
  virtual Eigen::VectorXf getFeatureIn(const int index) {
    return this->org_in_.points[index].getVector3fMap();
  }
};

#include "impl/edges.hpp"
}

#endif /* EDGES_H_ */

