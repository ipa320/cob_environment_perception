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
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#ifndef __DOWNSAMPLE_FILTER_H__
#define __DOWNSAMPLE_FILTER_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cob_3d_mapping_common/sensor_model.h"

namespace cob_3d_mapping_filters
{
  template <typename PointT, typename SensorT = cob_3d_mapping::PrimeSense>
  class DownsampleFilter
  {
    public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    public:
    DownsampleFilter() { }
    ~DownsampleFilter() { }

    void setInputCloud(const PointCloudConstPtr& points) { surface_ = points; }


    PointT convolve(size_t il, size_t i, size_t ir, const PointCloud& pc)
    {
      PointT p;
      if( pc[i].z != pc[i].z )
      {
        if ( pc[ir].z == pc[ir].z ) p = pc[ir];
        else p = pc[il];
      }
      else if( pc[ir].z != pc[ir].z || !SensorT::areNeighbors(pc[i].z, pc[ir].z) )
      {
        if( pc[il].z != pc[il].z || !SensorT::areNeighbors(pc[i].z, pc[il].z) ) p = pc[i];
        else
        {
          p = pc[i];
          p.x = (2.0f / 3.0f) * pc[i].x + pc[il].x / 3.0f;
          p.y = (2.0f / 3.0f) * pc[i].y + pc[il].y / 3.0f;
          p.z = (2.0f / 3.0f) * pc[i].z + pc[il].z / 3.0f;
        }
      }
      else if( pc[il].z != pc[il].z || !SensorT::areNeighbors(pc[i].z, pc[il].z) )
      {
        p = pc[i];
        p.x = (2.0f / 3.0f) * pc[i].x + pc[ir].x / 3.0f;
        p.y = (2.0f / 3.0f) * pc[i].y + pc[ir].y / 3.0f;
        p.z = (2.0f / 3.0f) * pc[i].z + pc[ir].z / 3.0f;
      }
      else
      {
        p = pc[i];
        p.x = 0.25f * pc[il].x + 0.5f * pc[i].x + 0.25f * pc[ir].x;
        p.y = 0.25f * pc[il].y + 0.5f * pc[i].y + 0.25f * pc[ir].y;
        p.z = 0.25f * pc[il].z + 0.5f * pc[i].z + 0.25f * pc[ir].z;
      }
      return p;
    }

    void filter(PointCloud& out)
    {
      out.width = 0.5 * surface_->width;
      out.height = 0.5 * surface_->height;
      out.resize(out.width * out.height);
      PointCloud tmp;
      tmp.width = out.width;
      tmp.height = surface_->height;
      tmp.resize(tmp.width * tmp.height);

      for (size_t y = 2*tmp.width; y< tmp.size() - tmp.width; y+=tmp.width)
      {
        for (size_t i = y+1; i< y+tmp.width-1; ++i)
        {
          int idx = 2*i;
          tmp[i] = convolve(idx-1, idx, idx+1, *surface_);
        }
      }

      for (size_t y = 1; y<out.height-1; ++y)
      {
        for (size_t x = 1; x<out.width-1; ++x)
        {
          int idx = (2*y+1)*out.width + x;
          out[y*out.width+x] = convolve(idx-out.width, idx, idx+out.width, tmp);
        }
      }

      for(size_t i = 0; i<out.width; ++i)
        out[i].x = out[i].y = out[i].z = std::numeric_limits<float>::quiet_NaN();
      for(size_t i = out.size() - out.width; i<out.size(); ++i)
        out[i].x = out[i].y = out[i].z = std::numeric_limits<float>::quiet_NaN();
      for(size_t i = out.width; i<out.size(); i+=out.width) {
        int i_end = i+out.width-1;
        out[i].x = out[i].y = out[i].z = std::numeric_limits<float>::quiet_NaN();
        out[i_end].x = out[i_end].y = out[i_end].z = std::numeric_limits<float>::quiet_NaN();
      }
    }

    private:
    PointCloudConstPtr surface_;
  };
}

#endif
