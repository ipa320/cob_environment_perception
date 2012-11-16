/*
 * moments.h
 *
 *  Created on: Nov 11, 2011
 *      Author: goa-jh
 */

#ifndef MOMENTS_H_
#define MOMENTS_H_

#include "../feature_container.h"
#include <pcl/features/moment_invariants.h>
#include <pcl/features/impl/moment_invariants.hpp>
#ifdef GICP_ENABLE
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#else
#include <pcl/kdtree/kdtree.h>
#endif

template<typename Point>
class Feature_MomentInvariants : public FeatureContainerInterface_Euclidean<Point>
{
  float radius_;

  pcl::PointCloud<pcl::MomentInvariants> feature_in_, feature_out_;

public:
  Feature_MomentInvariants():
    radius_(0.1)
  {
  }

  void setMomentRadius(float v) {radius_ = v;}

  virtual bool hidden_build() {

    ROS_INFO("calc moments for source");
    {
      pcl::MomentInvariantsEstimation<Point,pcl::MomentInvariants> est;
#ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
#else
  #ifdef PCL_VERSION_COMPARE
    boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
  #else
      boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
  #endif
#endif

      est.setInputCloud(this->org_in_.makeShared());
      est.setSearchMethod(tree);
      est.setRadiusSearch(radius_);

      est.compute(feature_in_);
    }

    ROS_INFO("calc moments for target");
    {
      pcl::MomentInvariantsEstimation<Point,pcl::MomentInvariants> est;
#ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
#else
  #ifdef PCL_VERSION_COMPARE
    boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
  #else
      boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
  #endif
#endif

      est.setInputCloud(this->org_out_.makeShared());
      est.setSearchMethod(tree);
      est.setRadiusSearch(radius_);

      est.compute(feature_out_);
    }

    return true;
  }

  virtual Eigen::VectorXf getFeatureOut(const int index) {
    Eigen::Vector3f b;
    b(0) = feature_out_.points[index].j1;
    b(1) = feature_out_.points[index].j2;
    b(2) = feature_out_.points[index].j3;
    return b;
  }
  virtual Eigen::VectorXf getFeatureIn(const int index) {
    Eigen::Vector3f b;
    b(0) = feature_in_.points[index].j1;
    b(1) = feature_in_.points[index].j2;
    b(2) = feature_in_.points[index].j3;
    return b;
  }

};


#endif /* MOMENTS_H_ */
