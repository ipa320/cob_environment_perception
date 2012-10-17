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

//general includes
#include <math.h>
#include <sstream>

//cob includes
#include "cob_3d_mapping_common/shape.h"
#include "cob_3d_mapping_common/polygon.h"
extern "C" {
#include "cob_3d_mapping_common/gpc.h"
}
//boost includes
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>
//pcl includes
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






namespace cob_3d_mapping{
/**
* \detail Class representing cylinder shapes.
* Cylinder Parameter Estimation can be performed.
* Cylinder Merging is handled.
*/
class Cylinder: public Polygon

{

public:
  typedef boost::shared_ptr<Cylinder> Ptr; /**< Cylinder pointer. Boost shared pointer to cylinder. */

  /**
  * \brief Constructor of Cylinder object.
  */
  Cylinder():merged_limit(50)
  {
  }

  //##############Methods to initialize cylinder and its paramers#########

  /**
  * \brief Assign points from pointcloud to contours of this cylinder.
  * \param[in] in_cloud Pointcloud containing contour points.
  */
  void ContoursFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud);


  /**
  * \brief Assign points from input list to contours of this cylinder.
  * \param[in] in_list List Vector containing contour points.
  */
  void ContoursFromList( std::vector<std::vector<Eigen::Vector3f> >& in_list);


  /**
  * \brief Paramter estimation and assignment from pointcloud.
  *
  * \detail Estimation uses pcl::SACSegmentation.Parameters assigned
  * to cylinder accordingly.
  * \param[in] in_cloud Input pointcloud
  * \param[in] indices Indices of contour points
  */
  void ParamsFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud, std::vector<int>& indices);


  /**
  * \brief Assign paramters from shape message to cylinder.
  *
  * \detail Assignment and completion of parameter set from shape
  * message to cylinder.
  */
  void ParamsFromShapeMsg();


  /**
  * \brief Compute Attributes of cylinder.
  *
  * \detail Compute attributes of cylinder depending on input parameters.
  * \param[in] sym_axis Symmetry axis of cylinder
  * \param[in] new_normal Normal of 2d representation of cylinder
  * \param[in] new_origin Origin of cylinder
  */
  virtual void computeAttributes(const Eigen::Vector3f & sym_axis,const Eigen::Vector3f &new_normal, const Eigen::Vector3f & new_origin);


   /**
  * \brief Transform cylinder to target frame.
  *
  * \param[in] trafo Transformation from source frame to target frame.
  */
  virtual void transform2tf(Eigen::Affine3f & tf);


  /**
  * \brief Grab parameters from source cylinder.
  * \param c_src Source cylinder
  */
  void GrabParams(Cylinder& c_src);


  //################## methods to roll and unroll cylinder###############
  /**
  * \brief Get 3d cylinder from 2d shape
  *
  * \detail 2D shape is transformed to 3D shape and copied.
  * \param[out] c3d Cylinder, 3D cylinder is copied to.
  * \see Cylinder::makeCyl3D()
  */
  void getCyl3D(Cylinder& c3d);


  /**
  *\brief Transformation to 3D of 2D shape
  *
  * \detail Transformation of 2D shape to 3D using polar coordinates.
  */
  void makeCyl3D();


  /**
  * \brief Transform 3D cylinder to 2D shape.
  *
  * \detail Projection of cylinder onto plane , by means of arclength.
  * This way treatment as a polygon is possible.
  */
  void makeCyl2D();


  /**
  * \brief Get 2D cylinder from 3D shape
  *
  * \detail 3D shape is transformed to 3D shape and copied.
  * \param[out] c2d cylinder , the 2D-shape is copied to.
  * \see Cylinder::makeCyl2D()
  */
  void getCyl2D(Cylinder& c2d);


   /**
  * \brief Transform Point from 2D to 3D
  *
  * \detail Transformation of Point, that is part of cylinder
  * from flat/2D shape to 3D shape.
  * \note Transformation depends on
  * position relative to normal and radius of the cylinder.
  * \note Coordinates have to be in cylinder system.
  * \param[in] pt2d Point , part of 2D shape
  * \param[out] pt3d Point, part of 3D shape
  */
  void getPt3D(Eigen::Vector3f& pt2d,Eigen::Vector3f& pt3d);



  //################## methods for merging############################
  /**
  * \brief Check for merge candidates.
  *
  * \detail  Cylinders are checked, if they have to be merged with
  * this cylinder under the given merge configuration.
  * Parameters of the Cylinders are compared as well if their contours are
  * intersected.
  * \param[in] poly_vec Vector of cylinders, that are checked.
  * \param[in] merge_config Conditions, under which merge will be performed
  * \param[out] intersections Indices of merge candidates
  */
  virtual void isMergeCandidate(const std::vector<Cylinder::Ptr >& cylinder_array,const merge_config& limits,std::vector<int>& intersections);


  /**
  * \brief Merge cylinders.
  *
  * \detail This cylinder is merged with cylinders in input array.
  * Therefore cylinders are transformed to flat polygons. Then the
  * Polygon merge process is applied.
  * The result is weighted,merged cylinder.
  * \param[in] c_array Array of cylinders, cylinder object is merged with.
  * \see Polygon::merge()
  */
  virtual void merge(std::vector<Cylinder::Ptr >& c_array);


  /**
  * \brief Weighting of cylinders to be merged.
  * \detail According to the merge weights of the individual cylinders,
  * this cylinder is assigned a merge weight.
  * \param[in] merge_candidates Cylinders, that the merge weight of this Cylinder depends on.
  * \see assignWeight()
  */
  virtual void applyWeighting(std::vector<Cylinder::Ptr >& merge_candidates);


  //############## debugging methods ####################
  /**
  * \brief Debug output of points to file.
  * \param[in] points Contour points of the Cylinder.
  * \param[in] name Name of the output file.
  */
  void dbg_out(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,std::string& name);


  /**
  * \brief Debug Output to terminal.
  * \param[in] name Name of the output file.
  */
  void printAttributes(std::string & name);


  /**
  * \brief Debug output of parameters to file.
  * \param[in] name Name of the output file.
  */
  void dump_params(std::string  name);


  //################# member variables########################
  double r_;                       /**< Radius of cylinder. */
  double h_min_;                   /**< Point at the bottom of cylinder.*/
  double h_max_;                   /**< Point on top of cylinder */
  Eigen::Vector3f sym_axis;        /**< Symmetry axis of cylinder. Direction Vector of symmetry axis. */
  Eigen::Vector3f origin_;         /**< Origin of cylinder. */
  int merged_limit;                /**< Limit for merge counter */

protected:
  //################ private methods for merging to avoid confusion by user################
  /**
  * \brief Compute arclength between 2 points.
  * \param[in] goal The end point of the arc.
  * \param[in] start The start point of the arc.
  * \param[out] Tx Translation in x direction, corresponds to the arclength.
  * \param[in] first True if goal is first point in a Cylinder contour.
  * \note The arclength is computed based on the radius of the Cylinder and the
  * relative position  of the points. Therefore both start and goal have to be in local Cylinder-coordinates
  * and they have to be positioned on the Cylinder surface.
  */
  void getArc(const Eigen::Vector3f& goal,const Eigen::Vector3f& start, float& Tx,bool first);


  /**
  * \brief Compensate offset
  *
  * \detail Transformation accounting for offset in symmetry axis and x,y -direction of origin.
  * This Cylinder is transformed to the Reference Cylinder.
  * \param[in] c_ref Reference Cylinder
  * \note Apply this, when two cylinders  have to be merged, whose origin and symmetry axis are not perfectly
  * similar.
  */
  void compensate_offset(Cylinder::Ptr& c_ref);

};
}

#endif
