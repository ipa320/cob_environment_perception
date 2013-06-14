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
* Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 03/2012
*
* \brief
* Parent class representing geometric shapes
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

#ifndef SHAPE_H_
#define SHAPE_H_

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <vector>

namespace cob_3d_mapping
{

  /**
  * \brief Class, representing Shape objects.
  * \note Base class for Cylinder and Polygon classes
  */
  class Shape

  {
  public:
    /**
    * \brief Shape Pointer.
    * \details Boost shared pointer to shape object.
    */
    typedef boost::shared_ptr<Shape> Ptr;

    /**
    * \brief Constructor of shape object.
    */
    Shape()
      : id_(0)
      , merged_(1)
      , frame_stamp_(0)
      , pose_(Eigen::Affine3f::Identity())
      , color_(4,1)
    { }

    /**
    * \brief Destructor of shape.
    */
    virtual ~Shape() { }

    /**
    * \brief Transform shape to target frame.
    * \param[in] trafo Transformatuon, which is applied.
    */
    virtual void transform(const Eigen::Affine3f& trafo)=0;

    double computeDistanceFromViewpoint() {return pose_.translation().norm();}

    unsigned int id_;/**< ID of shape.*/
    unsigned int merged_;/**< Number of times, shape has been merged.*/
    unsigned int frame_stamp_;/**< Frame, shape was created or merged the last time.*/
    //Eigen::Vector4f centroid;/**< Centroid of shape. */
    Eigen::Affine3f pose_; /**<The pose of the shape. Transforms points into the shape coordinate system. */
    std::vector<float> color_;/**< Color of shape as RGB vector. */
  };

  typedef boost::shared_ptr<Shape> ShapePtr;/**< Boosted shared pointer to shape. */

}


#endif /* SHAPE_H_ */
