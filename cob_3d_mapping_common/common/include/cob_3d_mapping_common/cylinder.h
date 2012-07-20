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
 * ROS stack name: cob_environment_perception
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-tz
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2012
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

#ifndef CYLINDER_H_
#define CYLINDER_H_

#include "cob_3d_mapping_common/shape.h"
#include "cob_3d_mapping_common/polygon.h"

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
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





namespace cob_3d_mapping{

class Cylinder: public Polygon

/*
 * Cylinder:
 *
 * Members:
 *        sym_axis .............................3x1 ........ symmetry axis of cylinder
 * 				normal		........................... 3x1 ........ second axis of cylinder, normal of the polygon, which represents the cylinder
 *        r         ........................... 1x1 ......... radius of the cylinder
 *
 */



{

public:

  Cylinder():debug_(false)
  {


  }

  //##############Methods to initialize cylinder and its paramers#########

  void ContoursFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud);
  void ContoursFromList( std::vector<std::vector<Eigen::Vector3f> >& in_list);
  void ParamsFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud, std::vector<int>& indices);
  void ParamsFromShapeMsg();
  virtual void computeAttributes(const Eigen::Vector3f & sym_axis,const Eigen::Vector3f &new_normal, const Eigen::Vector3f & new_origin);
  virtual void transform2tf(Eigen::Affine3f & tf);



  //################## methods to roll and unroll cylinder###############
  void getCyl3D(std::vector<std::vector<Eigen::Vector3f> >& contours3D);
  void makeCyl2D();

  //################## methods for merging############################
  virtual void isMergeCandidate(const std::vector<boost::shared_ptr<Cylinder> >& cylinder_array,const merge_config& limits,std::vector<int>& intersections);
  virtual void merge(std::vector<boost::shared_ptr<Cylinder> >& c_array);

//############## debugging methods ####################
  void dbg_out(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,std::string& name);
  void printAttributes(std::string & name);


//################# member variables########################
  double r_;
 Eigen::Vector3f sym_axis;
  Eigen::Vector3f origin_;
  //	Polygon unrolled_;
  bool debug_;

private:
//################ private methods for merging to avoid confusion by user################
  void getTrafo2d(const Eigen::Vector3f& vec3d, float& Tx, float& alpha);
  void getShiftedCylinder(Cylinder& c,Cylinder& shifted_cylinder);
  void applyWeightingCylinder(std::vector<boost::shared_ptr<Cylinder> >& merge_candidates);

};



typedef boost::shared_ptr<Cylinder> CylinderPtr;

}

#endif
