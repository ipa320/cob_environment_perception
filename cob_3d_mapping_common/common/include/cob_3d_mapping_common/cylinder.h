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
extern "C"
{
#include "libgpc/gpc.h"
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

namespace cob_3d_mapping
{

  /**
   * \brief Get the radius and origin of a point cloud representing a cylinder.
   * \param[in] in_cloud The input point cloud.
   * \param[in] indices The point indices representing the cylinder.
   * \param[out] origin The origin of the cylinder.
   * \param[in] sym_axis The symmetry axis of the cylinder.
   *
   * \return The radius of the cylinder.
   */
  double
  radiusAndOriginFromCloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud, std::vector<int>& indices,
                            Eigen::Vector3f& origin, const Eigen::Vector3f& sym_axis);

  /**
   * \brief Class representing Cylinder shapes.
   * \note Cylinder Parameter Estimation can be performed.
   * \note Cylinder Merging is handled.
   */
  class Cylinder : public Polygon
  {

  public:
    /**
     * \brief Cylinder pointer.
     * \details  Boost shared pointer to cylinder.
     */
    typedef boost::shared_ptr<Cylinder> Ptr;

    /**
     * \brief Construct empty Cylinder object.
     */
    Cylinder () :
        Polygon ()
    {
    }

    /**
     * \brief Construct Cylinder object from parameters.
     *
     * \param[in] id A unique id.
     * \param[in] origin The origin of the cylinder.
     * \param[in] sym_axis The symmetry axis of the cylinder.
     * \param[in] radius The radius of the cylinder.
     * \param[in] contours_3d The 3D contour points of the cylinder (represented by separate contours).
     * \param[in] holes A vector showing which contour is a hole.
     * \param[in] color The color of the cylinder.
     */
    Cylinder (unsigned int id, Eigen::Vector3f origin, Eigen::Vector3f sym_axis, double radius,
              std::vector<std::vector<Eigen::Vector3f> >& contours_3d, std::vector<bool> holes,
              std::vector<float> color);

    /**
     * \brief Construct Cylinder object from parameters.
     *
     * \param[in] id A unique id.
     * \param[in] origin The origin of the cylinder.
     * \param[in] sym_axis The symmetry axis of the cylinder.
     * \param[in] radius The radius of the cylinder.
     * \param[in] contours_3d The 3D contour points of the cylinder (represented by separate contours).
     * \param[in] holes A vector showing which contour is a hole.
     * \param[in] color The color of the cylinder.
     */
    Cylinder (unsigned int id, Eigen::Vector3f origin, Eigen::Vector3f sym_axis, double radius,
              std::vector<pcl::PointCloud<pcl::PointXYZ> >& contours_3d, std::vector<bool> holes,
              std::vector<float> color);

    //##############Methods to initialize cylinder and its paramers#########

    /**
     * \brief Set the 2D contours of the cylinder from 3D points.
     *
     * \param[in] contours_3d The 3D contours.
     */
    virtual void
    setContours3D (std::vector<std::vector<Eigen::Vector3f> >& contours_3d);

    /**
     * \brief Get the contours of the cylinder as 3D points.
     *
     * \return The 3D contours.
     */
    virtual std::vector<std::vector<Eigen::Vector3f> >
    getContours3D ();

    /**
     * \brief Compute Attributes (pose, sym axis) of cylinder.
     *
     * \details Compute attributes of cylinder depending on input parameters.
     * \param[in] sym_axis Symmetry axis of cylinder
     * \param[in] origin Origin of cylinder
     * \param[in] z_axis z axis of cylinder
     */
    virtual void
    updateAttributes (const Eigen::Vector3f &sym_axis, const Eigen::Vector3f &origin, const Eigen::Vector3f &z_axis);

    /**
     * \brief Transform the cylinder by tf.
     *
     * \param[in] tf Transformation to be applied.
     */
    virtual void
    transform (Eigen::Affine3f & tf);

    //################## methods for merging############################
    /**
     * \brief Find merge candidates.
     *
     * \param[in] cylinder_array The cylinders to be checked.
     * \param[out] intersections Indices of merge candidates.
     */
    virtual void
    getMergeCandidates (const std::vector<Polygon::Ptr>& cylinder_array, std::vector<int>& intersections);

    /**
     * \brief Merge cylinders.
     *
     *\param[in] poly_vec The cylinders to be merged with this.
     */
    virtual void
    merge (std::vector<Polygon::Ptr>& poly_vec);

    /**
     * \brief Obtain the params of this from another cylinder.
     *
     * \param[in] p The cylinder the parameters are copied from.
     */
    virtual void
    setParamsFrom (Polygon::Ptr& p);

    /**
     * \brief Compute the pose from origin and z axis.
     *
     * \param[in] origin The origin of the cylinder.
     * \param[in] z_axis The z_axis (perpendicular to the sym_axis) of the cylinder.
     */
    void
    computePose (Eigen::Vector3f origin, Eigen::Vector3f z_axis);

    /**
     * \brief Compute the pose from origin and a 3D contour.
     *
     * \param[in] origin The origin of the cylinder.
     * \param[in] contours_3d The 3D contour of the cylinder.
     */
    void
    computePose (Eigen::Vector3f origin, std::vector<std::vector<Eigen::Vector3f> >& contours_3d);

    /**
     * \brief Compute the height of the cylinder.
     */
    void
    computeHeight ();

    /**
     * \brief Triangulate surface polygons from the contours.
     *
     * \param[out] tri_list The list of polygons.
     */
    void
    triangulate (std::list<TPPLPoly>& tri_list) const;

    //############## debugging methods ####################
    /**
     * \brief Debug output of points to file.
     * \param[in] points Contour points of the Cylinder.
     * \param[in] name Name of the output file.
     */
    void
    dbgOut (pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, std::string& name);

    /**
     * \brief Debug Output to terminal.
     * \param[in] name Name of the output file.
     */
    void
    printAttributes (std::string & name);

    /**
     * \brief Debug output of parameters to file.
     * \param[in] name Name of the output file.
     */
    void
    dumpParams (std::string name);

    //################# member variables########################
    double r_; /**< Radius of cylinder. */
    double h_min_; /**< Point at the bottom of cylinder.*/
    double h_max_; /**< Point on top of cylinder */
    Eigen::Vector3f sym_axis_; /**< Symmetry axis of cylinder. Direction vector of symmetry axis. */
    //Eigen::Vector3f origin_;         /**< Origin of cylinder. */

  protected:
    /*
     * \brief Compute average cylinder from a set of cylinders.
     *
     * \param[in] poly_vec The set of cylinders.
     * \param[out] p_average The average cylinder.
     */
    virtual void
    computeAverage (const std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr& p_average);

  };
}

#endif
