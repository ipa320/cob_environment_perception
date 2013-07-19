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
 * Author: josh
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

#ifndef MULTIPLANE_H_
#define MULTIPLANE_H_

#include <visualization_msgs/MarkerArray.h>

#include <pcl/search/organized.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/filters/extract_indices.h>

#include "../general_segmentation.h"

namespace Segmentation
{

  /**
   * a segmentation implementation based on quad-trees and regression
   */
  template <typename Point, typename PointTypeNormal, typename PointLabel>
  class Segmentation_MultiPlane : public GeneralSegmentation<Point, PointLabel>
  {
    std::vector<typename pcl::PlanarRegion<Point>, Eigen::aligned_allocator<typename pcl::PlanarRegion<Point> > > regions;
    std::vector<pcl::ModelCoefficients> coef;
    std::vector<pcl::PointIndices> inlier_indices;
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    pcl::PointCloud<pcl::Label>::Ptr l;

    boost::shared_ptr<const pcl::PointCloud<Point> > input_;

  public:
    /// constructor, setups variables
    Segmentation_MultiPlane()
    {}

    /// destructor
    virtual ~Segmentation_MultiPlane() {
    }

    /// sets preprocessed input cloud
    virtual void setInputCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud)
    {
      input_ = cloud;
    }

    virtual bool compute();

    /// convert edges to ROS message
    operator visualization_msgs::Marker() const;

    /*** evaluation purposes ***/
    void compute_accuracy(float &mean, float &var, float &mean_weighted, float &var_weighted, size_t &used, size_t &mem, size_t &points, float &avg_dist, const boost::shared_ptr<const pcl::PointCloud<PointLabel> > &labeled_pc, double &true_positive, double &false_positive);

    /// gets reconstructed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getReconstructedOutputCloud();

    /// gets preprocessed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getOutputCloud () {return getReconstructedOutputCloud();}

    /// convert to ROS message
    virtual operator cob_3d_mapping_msgs::ShapeArray() const {return cob_3d_mapping_msgs::ShapeArray();}
  };

#include "impl/multi_plane.hpp"

}



#endif /* RANSAC_H_ */
