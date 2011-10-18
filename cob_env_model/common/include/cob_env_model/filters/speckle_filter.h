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

#ifndef SPECKLE_FILTER_H_
#define SPECKLE_FILTER_H_

//##################
//#### includes ####

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <opencv/cv.h>
//#define __LINUX__
//#include <cob_vision_utils/VisionUtils.h>

namespace cob_env_model
{
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
      //
    }
    ;

    //virtual ~SpeckleFilter();

    inline void
    setFilterParam (int speckle_s, double speckle_r)
    {
      speckle_size_ = speckle_s;
      speckle_range_ = speckle_r;
    }

    inline int
    getSpeckleSize ()
    {
      return speckle_size_;
    }

    inline double
    getSpeckleRange ()
    {
      return speckle_range_;
    }

    void applyFilter (pcl::PointIndices::Ptr points_to_remove);

  protected:

    void
    applyFilter (PointCloud &output);

    int speckle_size_;
    double speckle_range_;
  };

  template<>
  class SpeckleFilter<sensor_msgs::PointCloud2> : public pcl::Filter<sensor_msgs::PointCloud2>
  {
    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

  public:
    SpeckleFilter () :
      speckle_size_ (40), speckle_range_ (0.2)
    {
      //
    }
    ;
    //virtual ~SpeckleFilter();

    inline void
    setFilterParam (int speckle_s, float speckle_r)
    {
      speckle_size_ = speckle_s;
      speckle_range_ = speckle_r;
    }

    inline int
    getSpeckleSize ()
    {
      return speckle_size_;
    }

    inline int
    getSpeckleRange ()
    {
      return speckle_range_;
    }

  protected:

    void
    applyFilter (PointCloud2 &output);

    int speckle_size_;
    double speckle_range_;

  };
} // end namespace cob_env_model

#endif /* SPECKLE_FILTER_H_ */
