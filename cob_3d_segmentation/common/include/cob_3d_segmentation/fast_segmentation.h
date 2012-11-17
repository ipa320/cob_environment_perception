/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef __FAST_SEGMENTATION_H__
#define __FAST_SEGMENTATION_H__

#include <math.h>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_common/sensor_model.h"
#include "cob_3d_segmentation/general_segmentation.h"
#include "cob_3d_segmentation/cluster_handler.h"

namespace cob_3d_segmentation
{
  enum SeedMethod
  {
    SEED_RANDOM,
    SEED_LINEAR
  };

  struct SeedPoint
  {
    typedef boost::shared_ptr<SeedPoint> Ptr;
    SeedPoint(int idx_, int from_, float value_) : idx(idx_), i_came_from(from_), value(value_) { }
    int idx;
    int i_came_from;
    float value;
  };

  inline const bool operator<  (const SeedPoint& lhs, const SeedPoint& rhs) { return lhs.value < rhs.value; }
  inline const bool operator>  (const SeedPoint& lhs, const SeedPoint& rhs) { return  operator< (rhs, lhs); }
  inline const bool operator<= (const SeedPoint& lhs, const SeedPoint& rhs) { return !operator> (lhs, rhs); }
  inline const bool operator>= (const SeedPoint& lhs, const SeedPoint& rhs) { return !operator< (lhs, rhs); }

  struct ptr_deref
  {
    template<typename T>
    bool operator() (const boost::shared_ptr<T>& lhs, const boost::shared_ptr<T>& rhs) const { return operator< (*lhs, *rhs); }
  };

  typedef std::priority_queue<SeedPoint::Ptr, std::vector<SeedPoint::Ptr>, ptr_deref> SegmentationQueue;


  template<
    typename PointT,
    typename PointNT,
    typename PointLabelT,
    typename SensorT = cob_3d_mapping::PrimeSense,
    typename ClusterHdlT = cob_3d_segmentation::DepthClusterHandler<PointLabelT, PointT, PointNT>
    >
    class FastSegmentation : public GeneralSegmentation<PointT, PointLabelT>
  {
    public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::PointCloud<PointNT> NormalCloud;
    typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;
    typedef pcl::PointCloud<PointLabelT> LabelCloud;
    typedef typename LabelCloud::Ptr LabelCloudPtr;
    typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

    typedef typename ClusterHdlT::Ptr ClusterHdlPtr;
    typedef typename ClusterHdlT::ClusterPtr ClusterPtr;

    public:
    FastSegmentation ()
    : clusters_(new ClusterHdlT)
    , min_angle_(cos(30.0f / 180.0f * M_PI))
    , max_angle_(cos(45.0f / 180.0f * M_PI)) // threshold converges with increasing cluster size from max_angle_ to min_angle_
    , min_cluster_size_(200)
    , seed_method_(SEED_LINEAR)
    { }
    ~FastSegmentation () { }

    virtual void setInputCloud(const PointCloudConstPtr& points) { surface_ = points; clusters_->setPointCloudIn(points); }
    virtual LabelCloudConstPtr getOutputCloud() { return labels_; }
    virtual bool compute();


    void setNormalCloudIn(const NormalCloudConstPtr& normals) { normals_ = normals; clusters_->setNormalCloudIn(normals); }
    void setLabelCloudInOut(const LabelCloudPtr & labels) { labels_ = labels; clusters_->setLabelCloudInOut(labels); }
    void setSeedMethod(SeedMethod type) { seed_method_ = type; }
    void createSeedPoints();
    void mapSegmentColor(pcl::PointCloud<PointXYZRGB>::Ptr color_cloud) { clusters_->mapClusterColor(color_cloud); }

    private:
    inline float n_threshold(int size)
    { // currently linear degression of angle threshold
      if(size > min_cluster_size_) size = min_cluster_size_;
      return (min_angle_ - max_angle_) / min_cluster_size_ * size + max_angle_;
    }


    ClusterHdlPtr clusters_;
    PointCloudConstPtr surface_;
    NormalCloudConstPtr normals_;
    LabelCloudPtr labels_;

    float min_angle_;
    float max_angle_;
    float min_cluster_size_;
    SeedMethod seed_method_;
    std::vector<std::list<unsigned int>::iterator> p_seeds_;
    std::list<unsigned int> seeds_;
  };
}


#endif
