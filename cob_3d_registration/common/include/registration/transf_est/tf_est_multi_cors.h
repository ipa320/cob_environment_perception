/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: dynamic_tutorials
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Joshua Hampp
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Oct 26, 2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef TF_EST_MULTI_CORS_H_
#define TF_EST_MULTI_CORS_H_

#include <pcl/registration/registration.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>

/**
 * global transformation estimation using HIRN implications
 */
template <typename Point>
class TransformationEstimationMultipleCorrespondences : public pcl::Registration<Point, Point>
{

  /// search structure
  struct COR_S {
    int ind_o, ind_n;
    float dis;
  };

  /// search structure
  struct SORT_S2 {
    float dis;
    int ind;

    bool operator() (const SORT_S2 &i, const SORT_S2 &j) const { return (i.dis<j.dis);}
  };

  typedef typename pcl::Registration<Point, Point>::PointCloudSource PointCloudSource;
//  typedef typename registration<Point, Point>::CorrespondenceEstimation::PointCloudSource PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

public:

  TransformationEstimationMultipleCorrespondences () :
    tmax_(0.1f), rmax_(0.1f)
  {
    reg_name_ = "TransformationEstimationMultipleCorrespondences";
  };

  /// max. rotation movement beteween frames
  void setMaxAngularDistance(const float f) {rmax_=f;}
  /// max. translation movement beteween frames
  void setMaxTranslationDistance(const float f) {tmax_=f;}

  /// use this function for speed
  Eigen::Matrix4f compute(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new);

protected:
  virtual void
  computeTransformation (PointCloudSource &output);

  virtual void
  computeTransformation (PointCloudSource &output,const Eigen::Matrix4f& guess){};


  Eigen::Matrix4f findTF_fast
  (const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
                         const float rmax=0.1, const float tmax=0.1, Eigen::Matrix4f tf=Eigen::Matrix4f::Identity());
  int search_sorted_vector(const std::vector<SORT_S2> &tv, const float val);

  float tmax_,rmax_;

  using pcl::Registration<Point, Point>::converged_;
  using pcl::Registration<Point, Point>::reg_name_;
  using pcl::Registration<Point, Point>::final_transformation_;
  using pcl::Registration<Point, Point>::target_;

};

#include "impl/tf_est_multi_cors.hpp"

#endif /* TF_EST_MULTI_CORS_H_ */
