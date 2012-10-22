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
 * Author: goa-jh
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

#ifndef REGISTRATION_ICP_H_
#define REGISTRATION_ICP_H_

#include "general_registration.h"
#include "impl/modified_icp.hpp"
#include "feature_container.h"
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

namespace cob_3d_registration {

/**
 * encapsulates icp with extension
 * extending icp to allow featurebased matching, as keypoint matching
 */
template <typename Point>
class Registration_ICP : public GeneralRegistration<Point>
{
public:
  Registration_ICP():
    icp_max_iterations_(50),
    icp_max_corr_dist_(0.05),
    outlier_rejection_threshold_(0.01),
    icp_trf_epsilon_(0.0001),
    non_linear_(false),
    use_only_last_refrence_(false),
    use_gicp_(false)
  {}

  /// non linear uses LM instead of SVD
  void setNonLinear(bool b) {non_linear_=b;}

  /// set false to build up a map, instead of using only last frame for registration
  void setUseOnlyLastReference(bool b) {use_only_last_refrence_=b;}

  /// switch backend to gicp
  void setUseGICP(bool b) {
    use_gicp_=b;
#ifndef PCL_DEPRECATED
    ROS_ERROR("GICP is not supported by this version");
#endif
  }

  /// return pointlcoud to register against
  virtual boost::shared_ptr<pcl::PointCloud<Point> > getMap() {return register_.makeShared();}

  /// set maximum number of iterations
  void setMaxIterations(int v) {icp_max_iterations_=v;}

  /// set maximum correlation distance
  void setCorrDist(float v) {icp_max_corr_dist_=v;}

  /// sets transformation epsilon as break condition for icp
  void setTrfEpsilon(float v) {icp_trf_epsilon_=v;}

  /// sets maximum distance for RANSAC
  void setOutlierRejectionThreshold(float v) {outlier_rejection_threshold_=v;}

protected:

  virtual bool compute_features();
  virtual bool compute_corrospondences();
  virtual bool compute_transformation();

  /**
   * function for setting up icp
   * can be overwritten by subclasses to modify backend
   */
  virtual void setSettingsForICP(ModifiedICP_G *icp);

  //internal states
  pcl::PointCloud<Point> register_;

  // parameters
  int icp_max_iterations_;
  float icp_max_corr_dist_,
  outlier_rejection_threshold_,
  icp_trf_epsilon_;
  bool non_linear_, use_only_last_refrence_, use_gicp_;
};

/**
 * extending icp to features and keypoints
 */
template <typename Point>
class Registration_ICP_Features : public Registration_ICP<Point>
{
public:

  /**
   * sets a featureinterface
   * it is used for the correlation calculation in icp
   */
  void setFeatures(FeatureContainerInterface* features) {features_ = features;}
protected:

  FeatureContainerInterface* features_;

  virtual void setSettingsForICP(ModifiedICP_G *icp);
};

template <typename Point, typename FeatureType>
class Registration_ICP_Features_Extra : public Registration_ICP_Features<Point>
{
public:
  //virtual bool calculateFeature(pcl::PointCloud<Point>::Ptr input, pcl::PointCloud<Point>::Ptr output, pcl::PointCloud<FeatureType>::Ptr features);
  virtual bool calculateFeature(boost::shared_ptr<pcl::PointCloud<Point> > input, boost::shared_ptr<pcl::PointCloud<Point> > output, boost::shared_ptr<pcl::PointCloud<FeatureType> > features);
protected:
  virtual bool compute_features();
};


}

#endif
