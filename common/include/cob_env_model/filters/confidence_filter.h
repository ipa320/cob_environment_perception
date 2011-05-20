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

#ifndef CONFIDENCE_FILTER_H_
#define CONFIDENCE_FILTER_H_

//##################
//#### includes ####

// PCL includes
#include "pcl/pcl_base.h"
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace cob_env_model
{
  /**
   * \Confidence Filter uses confidence values of points to filter out pointcloud data
   */
  template <typename PointT>
  class ConfidenceFilter : public pcl::Filter<PointT>
  {
    using pcl::Filter<PointT>::input_;

    public:
      typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \constructor */
      ConfidenceFilter()
      : confidence_threshold_ (68000)
      {};

      //virtual ~ConfidenceFilter();

      /** \sets the filter limit */
      inline void
      setFilterLimit (double lim)
      {
        confidence_threshold_ = lim;
      }

      /** \gets the filter limit */
      inline double
      getFilterLimit ()
      {
        return confidence_threshold_;
      }

      /** \Points with confidence values above the filter limit will be discarded
       *  \Points with confidence values below the filter limit will be the output PointCloud
       */
      void
      applyFilter (PointCloud &output);

      /** \Points with confidence values below the filter limit will be discarded
       *  \Points with confidence values above the filter limit will be the output PointCloud
       */
      void
      negativeApplyFilter (PointCloud &output);

    protected:
      /** \filter limit */
      int confidence_threshold_;
  };

  template <>
  class ConfidenceFilter<sensor_msgs::PointCloud2> : public pcl::Filter<sensor_msgs::PointCloud2>
  {
    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
      /** \constructor */
      ConfidenceFilter()
      : confidence_threshold_(68000)
      { };

      //virtual ~ConfidenceFilter();

      /** \sets the filter limit */
      inline void
      setFilterLimit (double lim)
      {
        confidence_threshold_ = lim;
      }

      /** \gets the filter limit */
      inline double
      getFilterLimit ()
      {
        return confidence_threshold_;
      }

      /** \Points with confidence values above the filter limit will be discarded
       *  \Points with confidence values below the filter limit will be the output PointCloud
       */
      void
      applyFilter (PointCloud2 &output);

      /** \Points with confidence values below the filter limit will be discarded
       *  \Points with confidence values above the filter limit will be the output PointCloud
       */
      void
      negativeApplyFilter (PointCloud2 &output);

    protected:
      /** \filter limit */
      int confidence_threshold_;
  };
} // end namespace cob_env_model

#endif /* CONFIDENCE_FILTER_H_ */
