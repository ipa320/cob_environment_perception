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
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2011

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

#ifndef AMPLITUDE_FILTER_H_
#define AMPLITUDE_FILTER_H_

//##################
//#### includes ####

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace cob_env_model
{
  /**
   * \Amplitude Filter uses intensity values of points to filter out pointcloud data
   */
  template <typename PointT>
  class AmplitudeFilter : public pcl::Filter<PointT>
  {
    using pcl::Filter<PointT>::input_;

  public:
    typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    /** \constructor */
    AmplitudeFilter ()
    : amplitude_min_threshold_ (1000),
      amplitude_max_threshold_ (60000)
    { };

    //virtual ~AmplitudeFilter();

    /** \sets the filter limits */
    inline void
    setFilterLimits (double lim_min,double lim_max)
    {
      amplitude_min_threshold_ = lim_min;
      amplitude_max_threshold_ = lim_max;
    }

    /** \gets the filter minimum limit */
    inline double
    getFilterMinLimit ()
    {
      return amplitude_min_threshold_;
    }

    /** \gets the filter maximum limit */
    inline double
    getFilterMaxLimit ()
    {
      return amplitude_max_threshold_;
    }

    /** \Points with confidence values greater the filter limit will be discarded
     *   \Points with in filter limits will be the output PointCloud
     */
    void
    applyFilter (PointCloud &output);

    /* Points that are inside the filter limits will be discarded
     * Points outside the filter limits will be output PointCloud
     */
    void
    negativeApplyFilter (PointCloud &output);

  protected:
    /** \minimum limit for the filter */
    double amplitude_min_threshold_;

    /** \maximum limit for the filter */
    double amplitude_max_threshold_;
  };

  template <>
  class AmplitudeFilter<sensor_msgs::PointCloud2> : public pcl::Filter<sensor_msgs::PointCloud2>
  {
    using pcl::Filter<sensor_msgs::PointCloud2>::input_;

  public:
    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    /** \constructor */
    AmplitudeFilter()
    : amplitude_min_threshold_ (1000),
      amplitude_max_threshold_ (60000)
    { };

    //virtual ~AmplitudeFilter();

    /** \sets the filter limits */
    inline void
    setFilterLimits (double lim_min,double lim_max)
    {
      amplitude_min_threshold_ = lim_min;
      amplitude_max_threshold_ = lim_max;
    }

    /** \gets the filter minimum limit */
    inline double
    getFilterMinLimit ()
    {
      return amplitude_min_threshold_;
    }

    /** \gets the filter maximum limit */
    inline double
    getFilterMaxLimit ()
    {
      return amplitude_max_threshold_;
    }

  protected:

    /** \Points that are outside the filter limits will be discarded
     *  \Points with in filter limits will be the output PointCloud2
     */
    void
    applyFilter (PointCloud2 &output);

    /** \Points that are inside the filter limits will be discarded
     *  \Points outside the filter limits will be output PointCloud2
     */
    void
    negativeApplyFilter (PointCloud2 &output);

    /** \minimum limit for the filter */
    double amplitude_min_threshold_;

    /** \maximum limit for the filter */
    double amplitude_max_threshold_;

  };
} // end namespace cob_env_model

#endif /* AMPLITUDE_FILTER_H_ */
