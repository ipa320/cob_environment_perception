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
 *  Author: Joshua Hampp
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

#ifndef REGISTRATION_CORRESPONDENCE_H_
#define REGISTRATION_CORRESPONDENCE_H_
#ifdef PCL_VERSION_COMPARE
  #include <pcl/correspondence.h>
#endif
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace cob_3d_registration {

template <typename Point>
class RegKeypointCorrespondenceAbstract
{
public:
  virtual boost::shared_ptr<pcl::PointCloud<Point> > getSourcePoints()=0;
  virtual boost::shared_ptr<pcl::PointCloud<Point> > getTargetPoints()=0;

  #ifdef PCL_VERSION_COMPARE
    virtual void getCorrespondences(pcl::Correspondences &correspondences)=0;
  #else
    virtual void getCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences)=0;
  #endif

  virtual bool compute(const pcl::PointCloud<Point> &src, const pcl::PointCloud<Point> &tgt)=0;
};

template <typename Point, typename Keypoint>
class RegKeypointCorrespondence : public RegKeypointCorrespondenceAbstract<Point>
{
protected:
  pcl::PointCloud<Keypoint> keypoints_src_, keypoints_tgt_;

  virtual Point getPointForKeypointSrc(const int ind)=0;
  virtual Point getPointForKeypointTgt(const int ind)=0;

public:
  #ifdef PCL_VERSION_COMPARE
    virtual void getCorrespondences(pcl::Correspondences &correspondences) {
  #else
    virtual void getCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences) {
  #endif
    pcl::registration::CorrespondenceEstimation<Keypoint, Keypoint> est;
    est.setInputCloud (keypoints_src_.makeShared());
    est.setInputTarget (keypoints_tgt_.makeShared());
    est.determineReciprocalCorrespondences (correspondences);
  }

  virtual boost::shared_ptr<pcl::PointCloud<Point> > getSourcePoints() {
    boost::shared_ptr<pcl::PointCloud<Point> > ret(new pcl::PointCloud<Point>);
    for(int i=0; i<(int)keypoints_src_.size(); i++)
      ret->points.push_back(getPointForKeypointSrc(i));
    ret->height=1;
    ret->width=ret->size();
    return ret;
  }

  virtual boost::shared_ptr<pcl::PointCloud<Point> > getTargetPoints() {
    boost::shared_ptr<pcl::PointCloud<Point> > ret(new pcl::PointCloud<Point>);
    for(int i=0; i<(int)keypoints_tgt_.size(); i++)
      ret->points.push_back(getPointForKeypointTgt(i));
    ret->height=1;
    ret->width=ret->size();
    return ret;
  }
};


template <typename Point>
class Registration_Corrospondence : public GeneralRegistration<Point>
{
public:
  Registration_Corrospondence():
    rejection_dis_(0.5), keypoints_(NULL)
  {}

  void setKeypoints(RegKeypointCorrespondenceAbstract<Point> *k) {keypoints_=k;}
  RegKeypointCorrespondenceAbstract<Point> *getKeypoints() {return keypoints_;}

protected:

  virtual bool compute_features();
  virtual bool compute_corrospondences();
  virtual bool compute_transformation();

  #ifdef PCL_VERSION_COMPARE
    void rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                                 pcl::Correspondences &remaining_correspondences);
  #else
    void rejectBadCorrespondences (const pcl::registration::CorrespondencesPtr &all_correspondences,
                                 pcl::registration::Correspondences &remaining_correspondences);
  #endif

  //internal states
  pcl::PointCloud<Point> register_;

  float rejection_dis_;

  RegKeypointCorrespondenceAbstract<Point> *keypoints_;
  #ifdef PCL_VERSION_COMPARE
    pcl::CorrespondencesPtr all_correspondences_;
  #else
    pcl::registration::CorrespondencesPtr all_correspondences_;
  #endif
};

}

//keypoints...
//#include "features/segments.h"
#include "features/narf_kp.h"

#endif /* REGISTRATION_CORRESPONDENCE_H_ */
