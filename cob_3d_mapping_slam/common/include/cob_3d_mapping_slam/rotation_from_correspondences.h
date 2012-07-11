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
