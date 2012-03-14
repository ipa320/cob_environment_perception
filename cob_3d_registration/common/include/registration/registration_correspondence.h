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

#ifndef REGISTRATION_CORRESPONDENCE_H_
#define REGISTRATION_CORRESPONDENCE_H_

#include <pcl/registration/correspondence_estimation.h>


template <typename Point>
class RegKeypointCorrespondenceAbstract
{
public:
  virtual boost::shared_ptr<pcl::PointCloud<Point> > getSourcePoints()=0;
  virtual boost::shared_ptr<pcl::PointCloud<Point> > getTargetPoints()=0;

  virtual void getCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences)=0;

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
  virtual void getCorrespondences(std::vector<pcl::registration::Correspondence> &correspondences) {
    pcl::registration::CorrespondenceEstimation<Keypoint, Keypoint> est;
    est.setInputCloud (keypoints_src_.makeShared());
    est.setInputTarget (keypoints_tgt_.makeShared());
    est.determineReciprocalCorrespondences (correspondences);
  }

  virtual boost::shared_ptr<pcl::PointCloud<Point> > getSourcePoints() {
    boost::shared_ptr<pcl::PointCloud<Point> > ret(new pcl::PointCloud<Point>);
    for(int i=0; i<keypoints_src_.size(); i++)
      ret->points.push_back(getPointForKeypointSrc(i));
    ret->height=1;
    ret->width=ret->size();
    return ret;
  }

  virtual boost::shared_ptr<pcl::PointCloud<Point> > getTargetPoints() {
    boost::shared_ptr<pcl::PointCloud<Point> > ret(new pcl::PointCloud<Point>);
    for(int i=0; i<keypoints_tgt_.size(); i++)
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

  void rejectBadCorrespondences (const pcl::registration::CorrespondencesPtr &all_correspondences,
                                 pcl::registration::Correspondences &remaining_correspondences);

  //internal states
  pcl::PointCloud<Point> register_;

  float rejection_dis_;

  RegKeypointCorrespondenceAbstract<Point> *keypoints_;
  pcl::registration::CorrespondencesPtr all_correspondences_;
};


#include "impl/registration_correspondence.hpp"

//keypoints...
#include "features/segments.h"
#include "features/narf_kp.h"

#endif /* REGISTRATION_CORRESPONDENCE_H_ */
