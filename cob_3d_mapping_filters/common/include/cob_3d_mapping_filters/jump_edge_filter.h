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

#ifndef JUMP_EDGE_FILTER_H_
#define JUMP_EDGE_FILTER_H_

//##################
//#### includes ####

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace cob_3d_mapping_filters
{
  /**
   * \brief Filter for removing jump edges that occur in TOF point clouds or during fast camera movement.
   * \details Implements the method described in Stefan Fuchs and Stefan May: Calibration and Registration for Precise Surface Reconstruction,
   * In Proceedings of the DAGM Dyn3D Workshop, September, 2007, Heidelberg, Germany
   */
  template<typename PointT>
    class JumpEdgeFilter : public pcl::Filter<PointT>
    {
      using pcl::Filter<PointT>::input_;

    public:
      typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** Empty Constructor */
      JumpEdgeFilter () :
          angle_deg_ (170.0), keep_organized_(false)
      {
      }
      ;

      //virtual ~JumpEdgeFilter();

      /**
       * \brief Set the angle threshold.
       *
       * \param[in] angle The angle in degrees.
       */
      inline void
      setAngleThreshold (double angle)
      {
        angle_deg_ = angle;
      }

      /**
       * \brief Get the angle threshold.
       *
       * \return The angle threshold.
       */
      inline double
      getAngleThreshold () const
      {
        return angle_deg_;
      }
      
      inline bool
      getKeepOrganized() const
      {
		  return keep_organized_;
	  }
      
      inline void
      setKeepOrganized(bool keep_organized)
      {
		  keep_organized_ = keep_organized;
	  }

    protected:

      /**
       * \brief Apply the filter
       *
       * \param[out] output The filtered point cloud.
       */
      void
      applyFilter (PointCloud &output);

      /** Angle threshold in degrees  */
      double angle_deg_;
      
      bool keep_organized_;
    };

} // end namespace cob_3d_mapping_filters

#endif /* JUMP_EDGE_FILTER_H_ */
