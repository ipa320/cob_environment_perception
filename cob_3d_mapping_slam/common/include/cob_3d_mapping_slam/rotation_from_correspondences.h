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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
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
/*
 * rotation_from_correspondences.h
 *
 *  Created on: 28.04.2012
 *      Author: josh
 */

#ifndef ROTATION_FROM_CORRESPONDENCES_H_
#define ROTATION_FROM_CORRESPONDENCES_H_

#include <Eigen/Core>

namespace pcl
{
  /**
   * \brief Calculates a transformation based on corresponding 3D points
   * \author Bastian Steder
   * \ingroup common
   */
  class RotationFromCorrespondences
  {
  public:
    //-----CONSTRUCTOR&DESTRUCTOR-----
    /** Constructor - dimension gives the size of the vectors to work with. */
    RotationFromCorrespondences ();

    /** Destructor */
    ~RotationFromCorrespondences ();

    //-----METHODS-----
    /** Reset the object to work with a new data set */
    inline void
    reset ();

    /** Get the summed up weight of all added vectors */
    inline float
    getAccumulatedWeight () const { return accumulated_weight_;}

    /** Get the number of added vectors */
    inline unsigned int
    getNoOfSamples () { return no_of_samples_;}

    /** Add a new sample */
    inline void
    add (const Eigen::Vector3f& point, const Eigen::Vector3f& corresponding_point,
         const Eigen::Vector3f& n1, const Eigen::Vector3f& n2,
         const Eigen::Vector3f& cn1, const Eigen::Vector3f& cn2,
         float weight=1.0,float weight2=1.0);

    /** Calculate the transformation that will best transform the points into their correspondences */
    inline Eigen::Matrix3f
    getTransformation ();

    //-----VARIABLES-----

  //protected:
    //-----METHODS-----
    //-----VARIABLES-----
    unsigned int no_of_samples_;
    float accumulated_weight_,accumulated_weight2_;
    Eigen::Matrix<float, 3, 3> covariance_;
    Eigen::Matrix<float, 3, 3> var_;

    Eigen::Vector3f va_,vb_,vA_,vB_;
  };

}  // END namespace

#include "impl/rotation_from_correspondences.hpp"

#endif /* ROTATION_FROM_CORRESPONDENCES_H_ */
