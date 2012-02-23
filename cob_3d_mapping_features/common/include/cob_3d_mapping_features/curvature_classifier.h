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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 02/2012
 * ToDo:
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

#ifndef COB_3D_MAPPING_FEATURES_CURVATURE_CLASSIFIER_H_
#define COB_3D_MAPPING_FEATURES_CURVATURE_CLASSIFIER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

// define indices to access labels list:
#define I_NAN    0
#define I_BORDER 1
#define I_EDGE   2
#define I_PLANE  3
#define I_CYL    4
#define I_SPHERE 5

namespace cob_3d_mapping_features
{
  /*! @brief principal curvature classifier */
  template<typename PointInT, typename PointOutT> class CurvatureClassifier : public pcl::PCLBase<PointInT>
  {
  public:
    using pcl::PCLBase<PointInT>::input_;
    using pcl::PCLBase<PointInT>::indices_;

    typedef pcl::PointCloud<PointInT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef pcl::PointCloud<PointOutT> PointCloudOut;
    typedef typename PointCloudOut::Ptr PointCloudOutPtr;
    typedef typename PointCloudOut::ConstPtr PointCloudOutConstPtr;

    /*! Empty constructor */
    CurvatureClassifier()
      :pc_c_max_th_cylinder_upper_(0.11),
       pc_c_max_th_sphere_upper_(0.11),
       pc_c_max_th_lower_(0.004),
       pc_max_min_ratio_(5.0)
    {}
    /*! Empty destructor */
    ~CurvatureClassifier() {}

    /*!
     * @brief classify a point cloud
     *
     * @param[in] output a curvature point cloud to be classified
     */
    void classify(PointCloudOut &output);

  protected:
    //virtual inline bool
    //  initCompute();

    float pc_c_max_th_cylinder_upper_; //everything above is edge, should be the same value as pc.cmax_sphere_upper
    float pc_c_max_th_sphere_upper_; //everything above is edge, should be the same value as pc.cmax_cylinder_upper
    float pc_c_max_th_lower_; //everything below is plane
    float pc_max_min_ratio_; //ratio for pc to differentiate between cylinder and sphere

  };
}

#endif
