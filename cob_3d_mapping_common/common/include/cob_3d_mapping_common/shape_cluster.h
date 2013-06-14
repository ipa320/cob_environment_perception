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
* Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 08/2012
*
* \brief
* Class representing ShapeCluster shapes
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

#ifndef SHAPE_CLUSTER_H_
#define SHAPE_CLUSTER_H_

#include "cob_3d_mapping_common/shape.h"
#include <list>

namespace cob_3d_mapping
{
  /**
  * \brief Class representing ShapeCluster shapes.
  */
  class ShapeCluster : public Shape
  {
  public:
   /**
   * \brief ShapeCluster pointer.
   * \details  Boost shared pointer to ShapeCluster.
   */
    typedef boost::shared_ptr<ShapeCluster> Ptr;


   /**
   * \brief ConstShapeCluster pointer.
   * \details  Const Boost shared pointer to ShapeCluster.
   */
    typedef boost::shared_ptr<const ShapeCluster> ConstPtr;

  public:

    /**
    * \brief Constructor of ShapeCluster object.
    */
    ShapeCluster()
      : shapes()
      , scale(Eigen::Vector3f::Zero())
      , d_(0.1)
    { }

    /**
    * @brief Compute attributes of ShapeCluster
    */
    void computeAttributes();


   /**
  * \brief Transform ShapeCluster to target frame.
  *
  * \param[in] trafo Transformation from source frame to target frame.
  */
    void transform2tf(const Eigen::Affine3f& trafo);


    /**
    * \brief Check for merge candidates.
    *
    * \param[in] sc_vec Vector of ShapeClusters, that are checked.
    * \param[out] intersections Vector with indices of merge candidates.
    */
    void getMergeCandidates(const std::vector<ShapeCluster::Ptr>& sc_vec,
                            std::vector<int>& intersections) const;


    /**
    * \brief Merging of ShapeClusters.
    *
    * Complete merge process for this polygon with a vector of merge candidates.
    * Merging is performed relative to an average ShapeCluster.
    * \param[in] poly_vec Vector with merge candidate ShapeClusters
    */
    void merge(std::vector<ShapeCluster::Ptr>& sc_vec);

    /**
    * \@brief Insert shape to ShapeCluster.
    */
    void insert(Shape::Ptr shape);

    /**
    * \brief Check for similar ShapeCluster parameters.
    *
    * \details Check is performed based on merge settings of ShapeClusters.
    * \param[in] poly ShapeCluster, whose parameters are compared to this ShapeCluster.
    */
    inline bool hasSimilarParametersWith(ShapeCluster::Ptr sc) const
    {
      return ( (this->pose_.translation() - sc->pose_.translation()).norm() < d_ );
    }

    /**
    * @brief List with shapes
    */
    std::list<Shape::Ptr> shapes;
    /**
    * @brief A vector, containing the scale.
    */
    Eigen::Vector3f scale;

  private:
    float d_;
  };
}

#endif
