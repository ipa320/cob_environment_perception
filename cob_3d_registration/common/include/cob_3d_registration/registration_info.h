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

#ifndef REGISTRATION_INFO_H_
#define REGISTRATION_INFO_H_

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include "transf_est/tf_est_multi_cors.h"
#include <pcl/registration/transformation_estimation_svd.h>
#include "impl/modified_icp.hpp"

namespace cob_3d_registration {

/**
 * registration based on HIRN
 * implementation after diploma thesis of Joshua Hampp
 * using points of high interest for registration of 3d data
 * input data have to stay within a defined movement speed
 * using data without color is faster
 */
template <typename Point>
class Registration_Infobased : public GeneralRegistration<Point>
{

  /**
   * structure for search an fast access
   */
  struct SORT_S {
    float dis;
    int ind;

    bool operator() (const SORT_S &i, const SORT_S &j) const { return (i.dis<j.dis);}
  };

  /// binary map of interesting points
  unsigned char *depth_map;

  /// frame to register against
  boost::shared_ptr<const pcl::PointCloud<Point> > last_input_, last_input2_;

  /// indices of HIRN points
  std::vector<int> indices_pos2, indices_neg2;
  pcl::PointCloud<Point> source, target;

  ///debug output
  pcl::PointCloud<Point> source2_, target2_;

  /// use icp instead of global optimation
  bool use_icp_;

  /// check result with raycasted samples, forcing well aligned results for mapping (only with Kinect!!!)
  bool check_samples_;

  //settings

  /// see thesis
  float threshold_diff_, threshold_step_;
  int min_changes_, min_info_, max_info_;

  /// max. translation speed between two frames
  float tmax_;

  /// max. rotation speed between two frames
  float rmax_;

  int bad_counter_;
public:
  Registration_Infobased():
    depth_map(NULL), use_icp_(false),
    check_samples_(true),
    threshold_diff_(0.08), threshold_step_(0.08), min_changes_(800), min_info_(2), max_info_(16),
    tmax_(0.1), rmax_(0.1), bad_counter_(0),
    //threshold_diff_(0.06), min_changes_(4500), min_info_(1), max_info_(17), threshold_step_(0.06),
    //threshold_diff_(0.06), min_changes_(2600), min_info_(1), max_info_(17), threshold_step_(0.06),
    //threshold_diff_(0.06), min_changes_(4500), min_info_(1), max_info_(17), threshold_step_(0.06),
    odometry_last_(Eigen::Matrix4f::Identity()), odometry_(Eigen::Matrix4f::Identity()), failed_(0), standing_(0),
    use_odometry_(false), always_relevant_changes_(false), odo_is_good_(0), kinect_f_(0)
  {}

  //for freehand usage
  int getBadCounter() const {return bad_counter_;}
  void reset() {this->last_input_.reset();}

  void setThresholdDiff(const float f) {threshold_diff_=f;}
  void setThresholdStep(const float f) {threshold_step_=f;}
  void setMinChanges(const int f) {min_changes_=f;}
  void setMinInfo(const int f) {min_info_=f;}
  void setMaxInfo(const int f) {max_info_=f;}
  void setMaxAngularDistance(const float f) {rmax_=f;}
  void setMaxTranslationDistance(const float f) {tmax_=f;}
  /// check plausibility of result (only with Kinect)
  void setCheckBySamples(const bool b) {check_samples_=b;} /// for kinect only
  /// always use ICP instead of HIRN-backend
  void setUseICP(const bool b) {use_icp_=b;}

  /// setting up reprojection parameters, if not set it will be calculated (at least tried to)
  void setKinectParameters(const float f, const float dx, const float dy) {
    kinect_f_=f;
    kinect_dx_=dx;
    kinect_dy_=dy;
  }

  /// debug purpose: get HIRN points H+ and H-
  void getClouds(pcl::PointCloud<Point> &tmp_pc_old, pcl::PointCloud<Point> &tmp_pc_new);

  virtual ~Registration_Infobased() {
    delete [] depth_map;
  }

  virtual boost::shared_ptr<pcl::PointCloud<Point> > getMap() {return register_.makeShared();}

  /// inserted input (debug)
  pcl::PointCloud<Point> getLastInput() {return *this->last_input_;}
  /// reprojected input (debug)
  pcl::PointCloud<Point> getLastInput2() {return *this->last_input2_;}
  //debug HIRN-Points (activate DEBUG-Switch)
  pcl::PointCloud<Point> &getSource() {return source2_;}
  //debug HIRN-Points (activate DEBUG-Switch)
  pcl::PointCloud<Point> &getTarget() {return target2_;}
  virtual boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > getMarkers2() {return this->markers_.makeShared();}

  /// change transformation matrix (for example if odometry results are known)
  virtual void setOdometry (const Eigen::Matrix4f &odometry) {
    odometry_ = odometry;
    if(!use_odometry_) {
      odometry_last_ = odometry_;
      use_odometry_=true;
    }
  }

  virtual bool compute() {
    if(!compute_features()) {
      bad_counter_=0;
      return false;
    }
    if(!compute_transformation())
      return false;
    bad_counter_=0;
    return true;
  }

  /// if pose information is used, the changes are always relevant!
  void SetAlwaysRelevantChanges(const bool b) {always_relevant_changes_=b;}

  virtual void setTransformation(const Eigen::Matrix4f &mat) {this->transformation_=mat;this->odometry_last_=Eigen::Matrix4f::Identity();this->odometry_=Eigen::Matrix4f::Identity();}

private:
  pcl::PointCloud<Point> register_;
  pcl::PointCloud<pcl::PointXYZRGB> markers_;
  Eigen::Matrix4f odometry_last_, odometry_;
  int failed_, standing_;
  bool use_odometry_, always_relevant_changes_, odo_is_good_;
  float kinect_f_, kinect_dx_, kinect_dy_;

  virtual bool compute_features();
  virtual bool compute_corrospondences();
  virtual bool compute_transformation();

  void reproject();
  void getKinectParameters();
  bool checkSamples(const Eigen::Matrix4f &T, int *bad_out=NULL);

  inline float getMaxDiff2(const pcl::PointCloud<Point> &pc,const int ind,const pcl::PointCloud<Point> &pc2, int &mi);
  inline float getMaxDiff(const pcl::PointCloud<Point> &pc,const int ind);
  inline int getI(const int ind, const pcl::PointCloud<Point> &pc);
};

typedef Registration_Infobased<pcl::PointXYZ> Registration_HIRN_XYZ;

}


#endif /* REGISTRATION_INFO_H_ */
