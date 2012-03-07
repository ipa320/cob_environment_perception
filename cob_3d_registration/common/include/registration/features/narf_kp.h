/*
 * narf_kp.h
 *
 *  Created on: Nov 29, 2011
 *      Author: goa-jh
 */

#ifndef NARF_KP_H_
#define NARF_KP_H_

struct EIGEN_ALIGN16 NarfKPoint
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])

  pcl::FPFHSignature33 fpfh;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename Point>
class Keypoints_Narf : public RegKeypointCorrespondence<Point, NarfKPoint>
{
  float radius_;
  float thr_;
  float dist_threshold_;

  Point keypoint2point(const NarfKPoint &kp) {
    Point p;
    p.x=kp.x;
    p.y=kp.y;
    p.z=kp.z;
    return p;
  }

  void extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<NarfKPoint> &narf_descriptors);

public:
  Keypoints_Narf():
    radius_(0.05), thr_(0.1), dist_threshold_(0.1)
  {}

  void setRadius(float v) {radius_ = v;}
  void setThreshold(float v) {thr_ = v;}
  void setDisThreshold(float v) {dist_threshold_ = v;}

  virtual bool compute(const pcl::PointCloud<Point> &src, const pcl::PointCloud<Point> &tgt) {
    ROS_INFO("calc narf for source %d", src.size());
    {
      extractFeatures(src, this->keypoints_src_);
    }

    ROS_INFO("calc narf for target %d", tgt.size());
    {
      extractFeatures(tgt, this->keypoints_tgt_);
    }

    ROS_INFO("%d %d", this->keypoints_src_.size(), this->keypoints_tgt_.size());

    return this->keypoints_src_.size()>0 && this->keypoints_tgt_.size()>0;
  }

  virtual void getCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences) {
    pcl::PointCloud<pcl::FPFHSignature33> tsrc, ttgt;

    if(this->keypoints_src_.size()<1||this->keypoints_tgt_.size()<1) {
      ROS_ERROR("cannot compute 0 correspondences!");
      return;
    }

    for(int i=0; i<this->keypoints_src_.size(); i++)
      tsrc.points.push_back(this->keypoints_src_.points[i].fpfh);
    for(int i=0; i<this->keypoints_tgt_.size(); i++)
      ttgt.points.push_back(this->keypoints_tgt_.points[i].fpfh);
    tsrc.height=ttgt.height=1;
    ttgt.width=ttgt.size();
    tsrc.width=tsrc.size();

    {
      IndicesPtr indices_;
      indices_.reset (new std::vector<int>);
      indices_->resize (tsrc.points.size ());
      for(int i=0; i<tsrc.size(); i++)
        (*indices_)[i]=i;

      // setup tree for reciprocal search
      pcl::KdTreeFLANN<pcl::FPFHSignature33> tree_reciprocal;
      tree_reciprocal.setInputCloud(tsrc.makeShared(), indices_);
      pcl::KdTreeFLANN<pcl::FPFHSignature33> tree_;
      tree_.setInputCloud(ttgt.makeShared());

      correspondences.resize(indices_->size());
      std::vector<int> index(1);
      std::vector<float> distance(1);
      std::vector<int> index_reciprocal(1);
      std::vector<float> distance_reciprocal(1);
      pcl::registration::Correspondence corr;
      unsigned int nr_valid_correspondences = 0;

      for (unsigned int i = 0; i < indices_->size(); ++i)
      {
        tree_.nearestKSearch(tsrc.points[(*indices_)[i]], 1, index, distance);
        tree_reciprocal.nearestKSearch(ttgt.points[index[0]], 1, index_reciprocal, distance_reciprocal);

        if ( (*indices_)[i] == index_reciprocal[0] )
        {
          corr.indexQuery = (*indices_)[i];
          corr.indexMatch = index[0];
          corr.distance = (this->keypoints_tgt_[index[0]].getVector3fMap()-this->keypoints_src_[(*indices_)[i]].getVector3fMap()).squaredNorm();
          correspondences[nr_valid_correspondences] = corr;
          ++nr_valid_correspondences;
        }
      }
      correspondences.resize(nr_valid_correspondences);
    }

    ROS_INFO("cor found %d",correspondences.size());
  }

  Point getPointForKeypointSrc(const int ind) {return keypoint2point(this->keypoints_src_[ind]);}
  Point getPointForKeypointTgt(const int ind) {return keypoint2point(this->keypoints_tgt_[ind]);}
};

#include "impl/narf_kp.hpp"

#endif /* NARF_KP_H_ */
