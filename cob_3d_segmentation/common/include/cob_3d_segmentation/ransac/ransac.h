
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

#ifndef RANSAC_H_
#define RANSAC_H_

#include <visualization_msgs/MarkerArray.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/extract_indices.h>

#include "../general_segmentation.h"

namespace Segmentation
{

  /**
   * a segmentation implementation based on quad-trees and regression
   */
  template <typename Point, typename PointLabel>
  class Segmentation_RANSAC : public GeneralSegmentation<Point, PointLabel>
  {

    struct SHAPE_S {
      Eigen::VectorXf coeff_;
      std::vector<int> inliers_;
      enum {PLANE=1, CYLINDER=2, SPHERE=3} type_;
    };

    boost::shared_ptr<const pcl::PointCloud<Point> > input_;
    std::vector<SHAPE_S> shapes_;

    bool planes_, cylinders_, spheres_;

  public:
    /// constructor, setups variables
    Segmentation_RANSAC() : planes_(true), cylinders_(true), spheres_(true)
    {}

    /// destructor
    virtual ~Segmentation_RANSAC() {
    }

    /// sets preprocessed input cloud
    virtual void setInputCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud)
    {
      pcl::PointCloud<Point> *pc = new pcl::PointCloud<Point>;
      pc->header = cloud->header;
      pc->width  = cloud->width /2;
      pc->height = cloud->height/2;
      pc->resize(pc->width*pc->height);

      for(size_t x=0; x<pc->width; x++) {
        for(size_t y=0; y<pc->height; y++) {
          (*pc)(x,y) = (*cloud)(2*x,2*y);
          (*pc)(x,y).x = ((*cloud)(2*x,2*y).x+(*cloud)(2*x+1,2*y).x+(*cloud)(2*x,2*y+1).x+(*cloud)(2*x+1,2*y+1).x)/4;
          (*pc)(x,y).y = ((*cloud)(2*x,2*y).y+(*cloud)(2*x+1,2*y).y+(*cloud)(2*x,2*y+1).y+(*cloud)(2*x+1,2*y+1).y)/4;
          (*pc)(x,y).z = ((*cloud)(2*x,2*y).z+(*cloud)(2*x+1,2*y).z+(*cloud)(2*x,2*y+1).z+(*cloud)(2*x+1,2*y+1).z)/4;
        }
      }

      input_.reset(pc);
    }

    virtual bool compute();

    /// convert edges to ROS message
    operator visualization_msgs::Marker() const;

    /*** evaluation purposes ***/
    void compute_accuracy(float &mean, float &var, size_t &used, size_t &mem, size_t &points, float &avg_dist);

    void enablePlanes(const bool b) {planes_=b;}
    void enableSpheres(const bool b) {spheres_=b;}
    void enableCylinders(const bool b) {cylinders_=b;}

    /// gets reconstructed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getReconstructedOutputCloud();

    /// gets preprocessed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getOutputCloud () {return getReconstructedOutputCloud();}

    /// convert to ROS message
    virtual operator cob_3d_mapping_msgs::ShapeArray() const {return cob_3d_mapping_msgs::ShapeArray();}
  };

#include "impl/ransac.hpp"

}



#endif /* RANSAC_H_ */
