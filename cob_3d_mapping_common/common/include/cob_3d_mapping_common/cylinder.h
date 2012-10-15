/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Thomas Zwölfer, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 06/2012
*
* \brief
* Class representing cylinder shapes
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#ifndef CYLINDER_H_
#define CYLINDER_H_

#include "cob_3d_mapping_common/shape.h"
#include "cob_3d_mapping_common/polygon.h"

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>
extern "C" {
#include "cob_3d_mapping_common/gpc.h"
}
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#ifdef PCL_VERSION_COMPARE
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/exceptions.h>
#include <pcl/common/common.h>

#include <math.h>

#include <sstream>





namespace cob_3d_mapping{

class Cylinder: public Polygon

{

public:
  typedef boost::shared_ptr<Cylinder> Ptr; /**< Cylinder pointer. Boost shared pointer to cylinder. */

  /**
  * Constructor
  */
  Cylinder():merged_limit(50)
  {
  }

  //##############Methods to initialize cylinder and its paramers#########
  void ContoursFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud);
  void ContoursFromList( std::vector<std::vector<Eigen::Vector3f> >& in_list);
  void ParamsFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud, std::vector<int>& indices);
  void ParamsFromShapeMsg();
  virtual void computeAttributes(const Eigen::Vector3f & sym_axis,const Eigen::Vector3f &new_normal, const Eigen::Vector3f & new_origin);
  virtual void transform2tf(Eigen::Affine3f & tf);
  void GrabParams(Cylinder& c_src);


  //################## methods to roll and unroll cylinder###############
  void getCyl3D(Cylinder& c3d);
  void makeCyl2D();
  void makeCyl3D();
  void getCyl2D(Cylinder& c2d);
  void getPt3D(Eigen::Vector3f& pt2d,Eigen::Vector3f& pt3d);

  //################## methods for merging############################
  virtual void isMergeCandidate(const std::vector<Cylinder::Ptr >& cylinder_array,const merge_config& limits,std::vector<int>& intersections);
  virtual void merge(std::vector<Cylinder::Ptr >& c_array);
  virtual void applyWeighting(std::vector<Cylinder::Ptr >& merge_candidates);

  //############## debugging methods ####################
  void dbg_out(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,std::string& name);
  void printAttributes(std::string & name);
  void dump_params(std::string  name);


  //################# member variables########################
  double r_; /**< Radius of cylinder. */
  double h_min_; /**< Point at the bottom of cylinder.*/
  double h_max_; /**< Point on top of cylinder */
  Eigen::Vector3f sym_axis; /**< Symmetry axis of cylinder. Direction Vector of symmetry axis. */
  Eigen::Vector3f origin_; /**< Origin of cylinder. */
  int merged_limit; /**< Limit for merge counter */
protected:
  //################ private methods for merging to avoid confusion by user################
  void getArc(const Eigen::Vector3f& goal,const Eigen::Vector3f& start, float& Tx,bool first);
  void compensate_offset(Cylinder::Ptr& c_ref);

};
}

#endif
