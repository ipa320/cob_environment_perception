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
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 10/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#ifndef __EDGE_ESTIMATION_3D_H__
#define __EDGE_ESTIMATION_3D_H__

#include <pcl/features/feature.h>
#include <pcl/common/eigen.h>
//#include <Eigen/Geometry>

namespace cob_3d_features
{
  /** \brief Compute the angle in the [ 0, 2*PI ) interval of a point (direction) with a reference (0, 0) in 2D.
    * \param point a 2D point
    */
  inline float 
  getAngle2D (const float point[2])
  {
    float rad = atan2(point[1], point[0]);
    if (rad < 0)
      rad += 2 * M_PI;
    return (rad);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b EdgeEstimation estimates whether a set of points is lying on surface boundaries using an angle
    * criterion. The code makes use of the estimated surface normals at each point in the input dataset.
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \a NormalEstimationOpenMP and \a NormalEstimationTBB for examples on how to extend this to parallel implementations.
    * \author Radu Bogdan Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class EdgeEstimation3D: public pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using pcl::Feature<PointInT, PointOutT>::feature_name_;
      using pcl::Feature<PointInT, PointOutT>::getClassName;
      using pcl::Feature<PointInT, PointOutT>::input_;
      using pcl::Feature<PointInT, PointOutT>::indices_;
      using pcl::Feature<PointInT, PointOutT>::k_;
      using pcl::Feature<PointInT, PointOutT>::search_parameter_;
      using pcl::Feature<PointInT, PointOutT>::surface_;
      using pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    public:
      /** \brief Empty constructor. */
      EdgeEstimation3D () : dist_threshold_ (0.05), search_radius_ (20)
      {
        feature_name_ = "EdgeEstimation3D";
      };

      /** \brief Get a u-v-n coordinate system that lies on a plane defined by its normal
        * \param p_coeff the plane coefficients (containing the plane normal)
        * \param u the resultant u direction
        * \param v the resultant v direction
        */
      inline void 
      getCoordinateSystemOnPlane (const PointNT &p_coeff, 
                                  Eigen::Vector3f &u, Eigen::Vector3f &v)
      {
        pcl::Vector3fMapConst p_coeff_v = p_coeff.getNormalVector3fMap ();
        v = p_coeff_v.unitOrthogonal ();
        u = p_coeff_v.cross (v);
      }

      /** \brief Check whether a point is a boundary point in a planar patch of projected points given by indices.
        * \note A coordinate system u-v-n must be computed a-priori using \a getCoordinateSystemOnPlane
        * \param cloud a pointer to the input point cloud
        * \param q_idx the index of the query point in \a cloud
        * \param indices the estimated point neighbors of the query point
        * \param u the u direction
        * \param v the v direction
        * \param angle_threshold the threshold angle (default \f$\pi / 2.0\f$)
        */
      /*bool
      isBoundaryPoint (const pcl::PointCloud<PointInT> &cloud, 
                       int q_idx, const std::vector<int> &indices, 
                       const Eigen::Vector3f &u, const Eigen::Vector3f &v, float angle_threshold);*/

      /** \brief Check whether a point is a boundary point in a planar patch of projected points given by indices.
        * \note A coordinate system u-v-n must be computed a-priori using \a getCoordinateSystemOnPlane
        * \param cloud a pointer to the input point cloud
        * \param q_point a pointer to the querry point
        * \param indices the estimated point neighbors of the query point
        * \param u the u direction
        * \param v the v direction
        * \param angle_threshold the threshold angle (default \f$\pi / 2.0\f$)
        */
      /*bool
      isBoundaryPoint (const pcl::PointCloud<PointInT> &cloud, 
                       const PointInT &q_point, 
                       const std::vector<int> &indices, 
                       const Eigen::Vector3f &u, const Eigen::Vector3f &v, float angle_threshold);*/

      double
      isEdgePoint (
            const pcl::PointCloud<PointInT> &cloud, const PointInT &q_point,
            const std::vector<int> &indices,
            const Eigen::Vector3f &n, const float &ang_threshold);

      int
      searchForNeighbors (
  			int index,
  			double radius,
  			std::vector<int>& indices,
  			std::vector<float>& distances);

      /**calculate threshold with average distance of the points **/
      float
      calc_dist_threshold(const pcl::PointCloud<PointInT> &cloud , int idx);

      /** \brief The decision boundary (angle threshold) that marks points as boundary or regular. (default \f$\pi / 2.0\f$) */
      float dist_threshold_;
      float search_radius_;


    protected:

      /** \brief Estimate whether a set of points is lying on surface boundaries using an angle criterion for all points
        * given in <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains boundary point estimates
        */
      void 
      computeFeature (PointCloudOut &output);

      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output) { PCL_ERROR("[pcl::%s::computeFeatureEigen] Not implemented\n", getClassName().c_str()); }
  };
}

//#include "cob_3d_mapping_features/impl/edge_estimation_3d.hpp"

#endif  //#ifndef __EDGE_ESTIMATION_3D_H__


