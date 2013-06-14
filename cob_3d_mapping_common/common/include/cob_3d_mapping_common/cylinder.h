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

  double
  radiusAndOriginFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud,
                           std::vector<int>& indices,
                           Eigen::Vector3f& origin,
                           const Eigen::Vector3f& sym_axis);


  /**
   * \brief Class representing Cylinder shapes.
   * \note Cylinder Parameter Estimation can be performed.
   * \note Cylinder Merging is handled.
   */
  class Cylinder: public Polygon
  {

  public:
    /**
     * \brief Cylinder pointer.
     * \details  Boost shared pointer to cylinder.
     */
    typedef boost::shared_ptr<Cylinder> Ptr;


    /**
     * \brief Constructor of Cylinder object.
     */
    Cylinder()
    : Polygon()
    {
    }

    Cylinder(unsigned int id,
             Eigen::Vector3f origin,
             Eigen::Vector3f sym_axis,
             double radius,
             std::vector<std::vector<Eigen::Vector3f> >& contours_3d,
             std::vector<bool> holes,
             std::vector<float> color);

    Cylinder(unsigned int id,
             Eigen::Vector3f origin,
             Eigen::Vector3f sym_axis,
             double radius,
             std::vector<pcl::PointCloud<pcl::PointXYZ> >& contours_3d,
             std::vector<bool> holes,
             std::vector<float> color);

    //##############Methods to initialize cylinder and its paramers#########

    //void computePose(Eigen::Vector3f origin);

    virtual void setContours3D(std::vector<std::vector<Eigen::Vector3f> >& contours_3d);
    virtual std::vector<std::vector<Eigen::Vector3f> > getContours3D();

    void computePose(Eigen::Vector3f origin, Eigen::Vector3f z_axis);

    void computePose(Eigen::Vector3f origin, std::vector<std::vector<Eigen::Vector3f> >& contours_3d);

    void computeHeight();






    /**
     * \brief Compute Attributes (pose, sym axis) of cylinder.
     *
     * \details Compute attributes of cylinder depending on input parameters.
     * \param[in] sym_axis Symmetry axis of cylinder
     * \param[in] origin Origin of cylinder
     * \param[in] z_axis z axis of cylinder
     */
    virtual void updateAttributes(const Eigen::Vector3f &sym_axis, const Eigen::Vector3f &origin, const Eigen::Vector3f &z_axis);


    virtual void setParamsFrom(Polygon::Ptr& p);

    /**
     * \brief Transform cylinder to target frame.
     *
     * \param[in] trafo Transformation from source frame to target frame.
     */
    virtual void transform(Eigen::Affine3f & tf);

    void triangulate(list<TPPLPoly>& tri_list) const;



    //################## methods for merging############################
    /**
     * \brief Check for merge candidates.
     *
     * \details  Cylinders are checked, if they have to be merged with
     * this cylinder under the given merge configuration.
     * Parameters of the Cylinders are compared as well if their contours are
     * intersected.
     * \param[in] poly_vec Vector of cylinders, that are checked.
     * \param[in] merge_config Conditions, under which merge will be performed
     * \param[out] intersections Indices of merge candidates
     */
    virtual void getMergeCandidates(const std::vector<Polygon::Ptr >& cylinder_array, std::vector<int>& intersections);

    /**
     * \brief Merge cylinders.
     *
     * \details This cylinder is merged with cylinders in input array.
     * Therefore cylinders are transformed to flat polygons. Then the
     * Polygon merge process is applied.
     * The result is weighted,merged cylinder.
     * \param[in] c_array Array of cylinders, cylinder object is merged with.
     * \see Polygon::merge()
     */
    virtual void merge(std::vector<Polygon::Ptr>& poly_vec);



    //############## debugging methods ####################
    /**
     * \brief Debug output of points to file.
     * \param[in] points Contour points of the Cylinder.
     * \param[in] name Name of the output file.
     */
    void dbgOut(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,std::string& name);


    /**
     * \brief Debug Output to terminal.
     * \param[in] name Name of the output file.
     */
    void printAttributes(std::string & name);


    /**
     * \brief Debug output of parameters to file.
     * \param[in] name Name of the output file.
     */
    void dumpParams(std::string  name);


    //################# member variables########################
    double r_;                       /**< Radius of cylinder. */
    double h_min_;                   /**< Point at the bottom of cylinder.*/
    double h_max_;                   /**< Point on top of cylinder */
    Eigen::Vector3f sym_axis_;        /**< Symmetry axis of cylinder. Direction Vector of symmetry axis. */
    //Eigen::Vector3f origin_;         /**< Origin of cylinder. */

  protected:
    virtual void computeAverage(const std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr& p_average);

  };
}

#endif
