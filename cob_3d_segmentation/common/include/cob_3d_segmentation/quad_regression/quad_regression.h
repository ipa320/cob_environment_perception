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
 *  Author: josh
 * \author
 *  Supervised by: *
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

#ifndef SEGMENTATION_QUAD_REGR_H_
#define SEGMENTATION_QUAD_REGR_H_


#include <cob_3d_mapping_msgs/PlaneScene.h>
#include <visualization_msgs/MarkerArray.h>
#include "../point_types.h"
#include <cob_3d_mapping_common/stop_watch.h>
#include "quad_regression_algo.h"

namespace Segmentation
{


  //example for parent: QPPF::QuadRegression<QPPF::Degree2, Point, QPPF::CameraModel_Kinect<Point> >

  /**
   * a segmentation implementation based on quad-trees and regression
   */
  template <typename Point, typename PointLabel, typename Parent>
  class Segmentation_QuadRegression : public GeneralSegmentation<Point, PointLabel>, public Parent
  {

    void back_check_repeat(); /// repeat back check on model

    boost::shared_ptr<const pcl::PointCloud<PointLabel> > compute_labeled_pc();
    boost::shared_ptr<const pcl::PointCloud<PointLabel> > compute_reconstructed_pc();

  public:
    /// destructor
    virtual ~Segmentation_QuadRegression() {
    }

    /// gets preprocessed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getOutputCloud() {
      return compute_labeled_pc();
    }

    /// gets reconstructed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getReconstructedOutputCloud() {
      return compute_reconstructed_pc();
    }

    /// sets preprocessed input cloud
    virtual void setInputCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud)
    {
      this->Parent::setInputCloud(cloud);
    }

    virtual bool compute();

    virtual bool extractImages();

    /// convert to ROS message
    operator cob_3d_mapping_msgs::ShapeArray() const;

    /// convert to ROS message
    operator cob_3d_mapping_msgs::PlaneScene() const;

    /// convert edges to ROS message
    operator visualization_msgs::Marker() const;

    /*** evaluation purposes ***/
    void compute_accuracy(float &mean, float &var, float &mean_weighted, float &var_weighted, size_t &used, size_t &mem, size_t &points, float &avg_dist, const boost::shared_ptr<const pcl::PointCloud<PointLabel> > &labeled_pc, double &true_positive, double &false_positive);

    /// for easy testing purpose: serialize surfaces
    std::istream &serialize(std::istream &is);
    /// for easy testing purpose: serialize surfaces
    std::ostream &serialize(std::ostream &is) const;
  };

#include "impl/quad_regression.hpp"
}

#endif /* SEGMENTATION_QUAD_REGR_H_ */
