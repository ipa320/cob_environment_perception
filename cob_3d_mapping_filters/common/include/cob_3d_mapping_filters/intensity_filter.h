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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_filters
 *
 * \author
 *  Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 05/2011
 *
 * \brief
 * Description:
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

#ifndef INTENSITY_FILTER_H_
#define INTENSITY_FILTER_H_

//##################
//#### includes ####

// PCL includes
#include "pcl/pcl_base.h"
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace cob_3d_mapping_filters
{
  /**
   * \Intensity Filter uses Intensity values of points to filter out pointcloud data
   */
  template<typename PointT>
    class IntensityFilter : public pcl::Filter<PointT>
    {
      using pcl::Filter<PointT>::input_;

    public:
      typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \constructor */
      IntensityFilter () :
          intensity_max_threshold_ (65000), intensity_min_threshold_ (0), negative_ (false)
      {
      }
      ;

      //virtual ~IntensityFilter();

      /** \sets the filter limit */
      inline void
      setFilterLimits (double lim_min, double lim_max)
      {
        intensity_min_threshold_ = lim_min;
        intensity_max_threshold_ = lim_max;
      }

      /** \gets the filter minimum limit */
      inline double
      getFilterMinLimit ()
      {
        return intensity_min_threshold_;
      }

      /** \gets the filter maximum limit */
      inline double
      getFilterMaxLimit ()
      {
        return intensity_max_threshold_;
      }
      inline void
      setNegative (bool negative)
      {
        negative_ = negative;
      }

      inline bool
      getNegative ()
      {
        return (negative_);
      }

    public:

      /** \Points with Intensity values above the filter limit will be discarded
       *  \Points with Intensity values below the filter limit will be the output PointCloud
       */
      void
      applyFilter (PointCloud &output);

      /** \Points with Intensity values below the filter limit will be discarded
       *  \Points with Intensity values above the filter limit will be the output PointCloud
       */
      void
      negativeApplyFilter (PointCloud &output);

      /** \filter limit */
      int intensity_max_threshold_;
      int intensity_min_threshold_;

      bool negative_;
    };

  template<>
    class IntensityFilter<sensor_msgs::PointCloud2> : public pcl::Filter<sensor_msgs::PointCloud2>
    {
      typedef sensor_msgs::PointCloud2 PointCloud2;
      typedef PointCloud2::Ptr PointCloud2Ptr;
      typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
      /** \constructor */
      IntensityFilter () :
          intensity_max_threshold_ (65000), intensity_min_threshold_ (0)
      {
      }
      ;

      //virtual ~IntensityFilter();

      /** \sets the filter limit */
      inline void
      setFilterLimits (double lim_min, double lim_max)
      {
        intensity_min_threshold_ = lim_min;
        intensity_max_threshold_ = lim_max;
      }

      /** \gets the filter minimum limit */
      inline double
      getFilterMinLimit ()
      {
        return intensity_min_threshold_;
      }

      /** \gets the filter maximum limit */
      inline double
      getFilterMaxLimit ()
      {
        return intensity_max_threshold_;
      }

      /** \Points with Intensity values above the filter limit will be discarded
       *  \Points with Intensity values below the filter limit will be the output PointCloud
       */
      void
      applyFilter (PointCloud2 &output);

      /** \Points with Intensity values below the filter limit will be discarded
       *  \Points with Intensity values above the filter limit will be the output PointCloud
       */
      void
      negativeApplyFilter (PointCloud2 &output);

    protected:
      /** \filter limit */
      int intensity_max_threshold_;
      int intensity_min_threshold_;
    };
} // end namespace cob_3d_mapping_filters

#endif /* INTENSITY_FILTER_H_ */
