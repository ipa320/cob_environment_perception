/*
 * kinect_error.h
 *
 *  Created on: Nov 11, 2011
 *      Author: goa-jh
 */

#ifndef KINECT_ERROR_H_
#define KINECT_ERROR_H_


//##################
//#### includes ####

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace preprocessing
{

  template <typename PointT>
  class KinectErrorGenerator : public pcl::Filter<PointT>
  {
    using pcl::Filter<PointT>::input_;

  public:
    typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    /** \constructor */
    KinectErrorGenerator ()
    : standard_deviation_ (0.25)
    { };

    inline void setStandardDeviation(float f) {standard_deviation_=f;}
    inline float getStandardDeviation() {return standard_deviation_;}

    /** \Points with confidence values greater the filter limit will be discarded
     *   \Points with in filter limits will be the output PointCloud
     */
    void
    applyFilter (PointCloud &output);


  protected:
    double standard_deviation_;

  };

} // end namespace cob_3d_mapping_filters

#include "impl/kinect_error.hpp"
#endif /* KINECT_ERROR_H_ */
