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
 *e
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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

#include "cob_3d_mapping_common/shape_cluster.h"
#include <iostream>

void
cob_3d_mapping::ShapeCluster::computeAttributes()
{
  // TODO: proper computation of bounding box size
  scale = Eigen::Vector3f(0.1,0.1,0.1);
  this->centroid = Eigen::Vector4f::Zero();
  int i = 0;
  for(std::list<Shape::Ptr>::iterator it=shapes.begin(); it!=shapes.end(); ++it)
  {
    this->centroid += (*it)->centroid;
    ++i;
  }
  this->centroid /= static_cast<float>(i);
}

void
cob_3d_mapping::ShapeCluster::transform2tf(const Eigen::Affine3f& trafo)
{
  for(std::list<Shape::Ptr>::iterator i=shapes.begin(); i!=shapes.end(); ++i) (*i)->transform2tf(trafo);
}

void
cob_3d_mapping::ShapeCluster::getMergeCandidates(const std::vector<ShapeCluster::Ptr>& sc_vec,
                                                 std::vector<int>& intersections) const
{
  for(size_t i=0; i<sc_vec.size(); ++i)
  {
    //float length = (this->centroid.head(3) - sc_vec[i]->centroid.head(3)).norm();
    //std::cout << length << std::endl;
    if ( this->hasSimilarParametersWith(sc_vec[i]) ) intersections.push_back(i);
  }
}

void
cob_3d_mapping::ShapeCluster::merge(std::vector<ShapeCluster::Ptr>&sc_vec)
{
  int new_merged = 1;
  for(size_t i=0; i<sc_vec.size(); ++i)
  {
    this->shapes.insert(this->shapes.end(), sc_vec[i]->shapes.begin(), sc_vec[i]->shapes.end());
    new_merged += sc_vec[i]->merged;
  }
  this->merged = new_merged;
  this->computeAttributes();
}

void
cob_3d_mapping::ShapeCluster::insert(Shape::Ptr shape)
{
  shapes.push_back(shape);
}
