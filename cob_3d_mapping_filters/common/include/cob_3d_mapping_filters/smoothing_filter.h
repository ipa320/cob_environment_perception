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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_filters
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
    class SmoothingFilter : public pcl::Filter<PointT>
    {
      using pcl::Filter<PointT>::input_;

    public:
      typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \constructor */
      SmoothingFilter () :
          edge_threshold_ (0.05), smoothing_factor_ (0.25), integral_factor_ (0.25)
      {
      }
      ;

      //virtual ~IntensityFilter();

      /** \sets the filter limit */
      inline void
      setFilterLimits (double edge_threshold, double smoothing_factor, double integral_factor)
      {
        edge_threshold_ = edge_threshold;
        smoothing_factor_ = smoothing_factor;
        integral_factor_ = integral_factor;
      }

      /** \gets the filter minimum limit */
      inline double
      getEdgeThr ()
      {
        return edge_threshold_;
      }

      /** \gets the filter maximum limit */
      inline double
      getSmoothingFactor ()
      {
        return smoothing_factor_;
      }

      /** \gets the filter integral */
      inline double
      getIntegralFactor ()
      {
        return integral_factor_;
      }

    private:

      PointCloud last_pc_;

    public:

      /** \Points with Intensity values above the filter limit will be discarded
       *  \Points with Intensity values below the filter limit will be the output PointCloud
       */
      void
      applyFilter (PointCloud &output);

      /** \filter limit */
      float edge_threshold_;
      float smoothing_factor_;
      float integral_factor_;

    };

} // end namespace cob_3d_mapping_filters

#endif /* INTENSITY_FILTER_H_ */
