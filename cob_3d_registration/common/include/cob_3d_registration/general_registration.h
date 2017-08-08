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
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: dynamic_tutorials
 *
 * \author
 *  Author: goa-jh
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: Oct 26, 2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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

#pragma once

#include <ros/console.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>

namespace cob_3d_registration {

/**
 * a general abstract class for registration purpose of 3d pointclouds
 * as input it's espected to provide ...
 *      - the original (dense) pointcloud
 *      - the preprocessed pointcloud
 *      - the color image
 *      - the depth image
 * not all of the input data are necessary to all registration methods
 * if you are in need to spare one or more please refer to the used algorithms
 * result of compute is the success status and the transformation
 */
template <typename Point>
class GeneralRegistration
{
public:
  GeneralRegistration():transformation_(Eigen::Matrix4f::Identity()),moved_(true),scene_changed_(false)
  {}

  virtual ~GeneralRegistration() {}

  /// change transformation matrix (for example if odometry results are known)
  virtual void setOdometry (const Eigen::Matrix4f &odometry) {
    transformation_ = odometry;
  }

  /// sets preprocessed input cloud
  virtual void setInputCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud) {//(const pcl::PointCloud<Point>::ConstPtr &cloud) {
    input_ = cloud;
  }

  //TODO: change this mispelled word
  /// sets orginal input cloud
  virtual void setInputOginalCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud) {//(const pcl::PointCloud<Point>::ConstPtr &cloud) {
    input_org_ = cloud;
  }

  /// set color image
  virtual void setInputImage (const boost::shared_ptr<const cv::Mat> &img) {
    input_image_ = img;
  }

  /// set disparity image
  virtual void setInputDepthImage (const boost::shared_ptr<const cv::Mat> &img) {
    input_depth_image_ = img;
  }

  /// debug function for marker visualization
  virtual boost::shared_ptr<const pcl::PointCloud<Point> > getMarkers() {return boost::shared_ptr<const pcl::PointCloud<Point> >();}

  /**
   * compute transformation
   * @return true for success
   */
  virtual bool compute() {
    if(!compute_features())
      return false;
    if(!compute_corrospondences())
      return false;
    if(!compute_transformation())
      return false;
    return true;
  }

  /// get transformation
  virtual Eigen::Matrix4f getTransformation() const {return transformation_;}
  virtual void setTransformation(const Eigen::Matrix4f &mat) {transformation_=mat;}
  virtual void setMoved(const bool b) {moved_=b;}
  virtual bool getSceneChanged() const {return scene_changed_;}

  /// map is not necessarily implemented
  virtual boost::shared_ptr<pcl::PointCloud<Point> > getMap() {ROS_ERROR("should never happ"); return boost::shared_ptr<pcl::PointCloud<Point> >(new pcl::PointCloud<Point>);}

protected:

  virtual bool compute_features()=0;
  virtual bool compute_corrospondences()=0;
  virtual bool compute_transformation()=0;

  boost::shared_ptr<const pcl::PointCloud<Point> > input_, input_org_;
  boost::shared_ptr<const cv::Mat> input_image_, input_depth_image_;
  Eigen::Matrix4f transformation_;
  bool moved_, scene_changed_;

};

}
