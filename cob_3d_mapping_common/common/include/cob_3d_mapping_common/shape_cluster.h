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
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2012
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

#ifndef SHAPE_CLUSTER_H_
#define SHAPE_CLUSTER_H_

#include "cob_3d_mapping_common/shape.h"
#include <list>

namespace cob_3d_mapping
{
  class ShapeCluster : public Shape
  {
  public:
    typedef boost::shared_ptr<ShapeCluster> Ptr;
    typedef boost::shared_ptr<const ShapeCluster> ConstPtr;

  public:
    ShapeCluster()
      : shapes()
      , scale(Eigen::Vector3f::Zero())
      , d_(0.1)
    { }

    void computeAttributes();
    void transform2tf(const Eigen::Affine3f& trafo);

    void getMergeCandidates(const std::vector<ShapeCluster::Ptr>& sc_vec,
                            std::vector<int>& intersections) const;
    void merge(std::vector<ShapeCluster::Ptr>& sc_vec);
    void insert(Shape::Ptr shape);

    inline bool hasSimilarParametersWith(ShapeCluster::Ptr sc) const
    {
      return ( (this->centroid - sc->centroid).norm() < d_ );
    }

    std::list<Shape::Ptr> shapes;
    Eigen::Vector3f scale;

  private:
    float d_;
  };
}

#endif
