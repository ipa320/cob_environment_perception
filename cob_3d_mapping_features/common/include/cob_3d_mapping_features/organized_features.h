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
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 12/2011
 * ToDo:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
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

#ifndef __ORGANIZED_FEATURES_H__
#define __ORGANIZED_FEATURES_H__

#include <pcl/pcl_base.h>

namespace cob_3d_mapping_features
{
  template <typename PointInT, typename PointOutT>
    class OrganizedFeatures : public pcl::PCLBase<PointInT>
  {
    public:

      using pcl::PCLBase<PointInT>::input_;
      using pcl::PCLBase<PointInT>::indices_;

      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;

    public:
      /** \brief Empty constructor. */
      OrganizedFeatures () : pixel_search_radius_(2)
	,pixel_steps_(1)
	,circle_steps_(1)
	,distance_threshold_modifier_(4.0)
      { };

      inline void
	setSearchSurface(const PointCloudInConstPtr cloud)
      {
	surface_ = cloud;
	fake_surface_ = false;
      }

      //TODO: set radius in m instead of pixel
      inline void
	setPixelSearchRadius(int pixel_radius, int pixel_step_size=1, int circle_step_size=1)
      {
	pixel_search_radius_ = pixel_radius;
	pixel_steps_ = pixel_step_size;
	circle_steps_ = circle_step_size;
      }

      inline void
	setDistanceThresholdModifier(float mod)
      {
	distance_threshold_modifier_ = mod;
      }

      void
	compute(PointCloudOut &output);

      int
	searchForNeighbors(const PointCloudIn &cloud, int index, std::vector<int>& indices);

      int
	searchForNeighborsInRange(const PointCloudIn &cloud, int index, std::vector<int>& indices);


    protected:

      virtual inline bool
	initCompute();

      virtual inline bool
	deinitCompute();

      inline bool
	isNaN (const Eigen::Vector3f &pt)
      {
	return (pcl_isnan(pt[0]) || pcl_isnan(pt[1]) || pcl_isnan(pt[2]));
      }

      inline bool
	isNaN (const PointInT &pt)
      {
	return (pcl_isnan(pt.x) || pcl_isnan(pt.y) || pcl_isnan(pt.z));
      }

      inline bool
	isInRange (const PointInT &pi, const PointInT &pq, float distance_th_sqr)
      {
	float dx = pi.x - pq.x;
	float dy = pi.y - pq.y;
	float dz = pi.z - pq.z;
	return ( (dx*dx + dy*dy + dz*dz) < distance_th_sqr);
      }

      inline bool
	isInImage (int u, int v)
      {
	return ( v >= 0 && v < (int)input_->height && u >= 0 && u < (int)input_->width );
      }

      inline const std::string&
	getClassName () const { return (feature_name_); }

      PointCloudInConstPtr surface_;
      bool fake_surface_;

      int pixel_search_radius_;
      int pixel_steps_;
      int circle_steps_;
      std::vector<std::vector<int> > mask_;
      float inv_width_;
      float distance_threshold_modifier_;

      std::string feature_name_;

    private:
      virtual void
	computeFeature (PointCloudOut &output) = 0;
  };
}

#include "cob_3d_mapping_features/impl/organized_features.hpp"

#endif  //#ifndef __ORGANIZED_FEATURES_H__
