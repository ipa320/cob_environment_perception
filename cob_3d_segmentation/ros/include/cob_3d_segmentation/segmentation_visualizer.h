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
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 02/2012
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

#ifndef __SEGMENTATION_VISUALIZER_H__
#define __SEGMENTATION_VISUALIZER_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace cob_3d_segmentation
{
  class SegmentationVisualizer
  {
    public:
    SegmentationVisualizer() : nh_(), v_(), v1_(0), v2_(0)
    { }

    ~SegmentationVisualizer() 
    { }


    inline void
      spinOnce() { v_.spinOnce(100); }

    inline bool
      isRunning() { return !v_.wasStopped(); }

    void
      init();


    
    protected:

    void
      update_v1_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

    void
      update_v2_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

    ros::NodeHandle nh_;
    ros::Subscriber sub_v1_;
    ros::Subscriber sub_v2_;

    pcl::visualization::PCLVisualizer v_;
    int v1_;
    int v2_;
  };
}


#endif
