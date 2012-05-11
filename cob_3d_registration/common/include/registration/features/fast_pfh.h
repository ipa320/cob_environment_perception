/*
 * fast_pfh.h
 *
 *  Created on: Nov 14, 2011
 *      Author: goa-jh
 */

#ifndef FAST_PFH_H_
#define FAST_PFH_H_

#include "../feature_container.h"
#include <pcl/features/fpfh.h>
#ifdef GICP_ENABLE
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#else
#include <pcl/kdtree/kdtree.h>
#endif

template<typename Point>
class Feature_FPFH : public FeatureContainerInterface_Euclidean<Point>
{
  float radius_;

  pcl::PointCloud<pcl::FPFHSignature33> feature_in_, feature_out_;

public:
  Feature_FPFH():
    radius_(0.1)
  {
  }

  void setFPFHRadius(float v) {radius_ = v;}

  virtual bool hidden_build() {

    ROS_INFO("calc fpfh for source");
    {
#ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
#else
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
#endif
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

      pcl::NormalEstimation<Point, pcl::Normal> ne;
      ne.setSearchMethod (tree);
      ne.setInputCloud (this->org_in_.makeShared());
      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.05);

      // Compute the features
      ne.compute (*normals);

      // Create the FPFH estimation class, and pass the input dataset+normals to it
      pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud (this->org_in_.makeShared());
      fpfh.setInputNormals (normals);

      fpfh.setSearchMethod (tree);
      fpfh.setRadiusSearch (radius_);

      // Compute the features
      fpfh.compute (feature_in_);
    }

    ROS_INFO("calc fpfh for target");
    {
#ifdef GICP_ENABLE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
#else
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::search::KdTree<Point>);
#endif
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

      pcl::NormalEstimation<Point, pcl::Normal> ne;
      ne.setSearchMethod (tree);
      ne.setInputCloud (this->org_out_.makeShared());
      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.1);

      // Compute the features
      ne.compute (*normals);

      // Create the FPFH estimation class, and pass the input dataset+normals to it
      pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud (this->org_out_.makeShared());
      fpfh.setInputNormals (normals);

      fpfh.setSearchMethod (tree);
      fpfh.setRadiusSearch (radius_);

      // Compute the features
      fpfh.compute (feature_out_);
    }

    return true;
  }

  virtual Eigen::VectorXf getFeatureOut(const int index) {
    Eigen::VectorXf b;
    b.resize(33);
    for(int i=0; i<33; i++) b(i) = feature_out_.points[index].histogram[i];
    return b;
  }
  virtual Eigen::VectorXf getFeatureIn(const int index) {
    Eigen::VectorXf b;
    b.resize(33);
    for(int i=0; i<33; i++) b(i) = feature_in_.points[index].histogram[i];
    return b;
  }

};


#endif /* FAST_PFH_H_ */
