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

#ifndef SPECKLE_FILTER_H_
#define SPECKLE_FILTER_H_

//##################
//#### includes ####

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace cob_3d_mapping_filters
{
  /**
   * \brief Outlier removal filter based on region growing (speckles).
   * \details See Fischer, Jan ; Arbeiter, Georg ; Verl, Alexander:
   * Combination of Time-of-Flight Depth and Stereo using Semiglobal Optimization
   * In: IEEE / Robotics and Automation Society: 2011 IEEE International Conference on Robotics and Automation ICRA : Better Robots, Better Life.
   * May 9-13, 2011, Shanghai, China. Piscataway, NJ, USA : IEEE Press, 2011, S. 3548-3553. - URN urn:nbn:de:0011-n-1901013
   * for details.
   */
  template<typename PointT>
    class SpeckleFilter : public pcl::Filter<PointT>
    {
      using pcl::Filter<PointT>::input_;

    public:
      typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      SpeckleFilter () :
          speckle_size_ (40), speckle_range_ (0.2)
      {
        points_to_remove_ = pcl::PointIndices::Ptr (new pcl::PointIndices ());
      }
      ;

      //virtual ~SpeckleFilter();

      /**
       * \brief Set the filter parameters.
       *
       * \param[in] speckle_s The minimum speckle size.
       * \param[in] speckle_r The minimum distance between speckles.
       */
      inline void
      setFilterParam (int speckle_s, double speckle_r)
      {
        speckle_size_ = speckle_s;
        speckle_range_ = speckle_r;
      }

      /**
       * \brief Get the speckle size parameter.
       *
       * \return The minimum speckle size.
       */
      inline int
      getSpeckleSize ()
      {
        return speckle_size_;
      }

      /**
       * \brief Get the speckle range parameter.
       *
       * \return The maximum distance between speckles.
       */
      inline double
      getSpeckleRange ()
      {
        return speckle_range_;
      }

      /**
       * \brief Get the indices of the removed points.
       *
       * \return The indices.
       */
      pcl::PointIndices::Ptr
      getRemovedIndices ()
      {
        return points_to_remove_;
      }

    protected:

      /**
       * \brief Apply the filter
       *
       * \param[out] output The filtered point cloud.
       */
      void
      applyFilter (PointCloud &output);

      int speckle_size_; ///< The minimum speckle size.
      double speckle_range_; ///< The minimum distance between two speckles.
      pcl::PointIndices::Ptr points_to_remove_; ///< The indices of the removed points.
    };

} // end namespace cob_3d_mapping_filters

#endif /* SPECKLE_FILTER_H_ */
