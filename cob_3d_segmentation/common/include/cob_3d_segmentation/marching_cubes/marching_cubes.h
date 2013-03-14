
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

#ifndef MARCHING_CUBES_H_
#define MARCHING_CUBES_H_

#include <visualization_msgs/MarkerArray.h>

#ifndef PCL_VERSION_COMPARE
#define USE_GREEDY
#endif

#include <pcl/surface/marching_cubes.h>
#ifdef USE_GREEDY
#include <pcl/surface/marching_cubes_greedy.h>
#define MARCHING_CUBES_INST MarchingCubesGreedy
#else
#include <pcl/surface/marching_cubes_rbf.h>
#define MARCHING_CUBES_INST MarchingCubesRBF
#endif
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "../general_segmentation.h"

namespace Segmentation
{

  /**
   * a segmentation implementation based on quad-trees and regression
   */
  template <typename Point, typename PointTypeNormal, typename PointLabel>
  class Segmentation_MarchingCubes : public GeneralSegmentation<Point, PointLabel>
  {

    boost::shared_ptr<const pcl::PointCloud<Point> > input_;
    pcl::PolygonMesh mesh_;

    float leafSize_;
    float isoLevel_;

  public:
    /// constructor, setups variables
    Segmentation_MarchingCubes() : leafSize_(0.025), isoLevel_(0.02f)
    {}

    /// destructor
    virtual ~Segmentation_MarchingCubes() {
    }

    /// sets preprocessed input cloud
    virtual void setInputCloud (const boost::shared_ptr<const pcl::PointCloud<Point> > &cloud)
    {
      pcl::PointCloud<Point> *pc = new pcl::PointCloud<Point>;
      pc->header = cloud->header;
      pc->height  = 1;

      for(size_t x=0; x<cloud->size(); x++) {
          if(pcl_isfinite((*cloud)[x].x))
            pc->push_back((*cloud)[x]);
      }

      pc->width = pc->size();

      input_.reset(pc);
    }

    virtual bool compute();

    /// convert edges to ROS message
    operator visualization_msgs::Marker() const;

    /*** evaluation purposes ***/
    void compute_accuracy(float &mean, float &var, size_t &used, size_t &mem, size_t &points, float &avg_dist);

    /// gets reconstructed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getReconstructedOutputCloud();

    /// gets preprocessed output cloud
    virtual boost::shared_ptr<const pcl::PointCloud<PointLabel> > getOutputCloud () {return getReconstructedOutputCloud();}

    /// convert to ROS message
    virtual operator cob_3d_mapping_msgs::ShapeArray() const {return cob_3d_mapping_msgs::ShapeArray();}
  };

#include "impl/marching_cubes.hpp"

}



#endif /* MARCHING_CUBES_H_ */
