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
 * ROS package name: cob_3d_segmentation
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

#include "cob_3d_segmentation/segmentation_visualizer.h"

#include <pcl/visualization/point_cloud_handlers.h>


using namespace cob_3d_segmentation;

void
SegmentationVisualizer::init()
{

  /* --- Viewports: ---
   *  1y
   *    | 1 | 3 |
   * .5 ----+----
   *    | 2 | 4 |
   *  0    .5    1x
   * 1:
   */
  // xmin, ymin, xmax, ymax

  v_.createViewPort(0.0, 0.0, 0.5, 1.0, v1_);
  v_.setBackgroundColor(0, 1.0, 1.0, v1_);

  v_.createViewPort(0.5, 0.0, 1.0, 1.0, v2_);
  v_.setBackgroundColor(0, 1.0, 1.0, v2_);

  sub_v1_ = nh_.subscribe<sensor_msgs::PointCloud2>
    ("segmentation_cloud", 1, boost::bind(&SegmentationVisualizer::update_v1_cb, this, _1));
  sub_v2_ = nh_.subscribe<sensor_msgs::PointCloud2>
    ("classified_cloud", 1, boost::bind(&SegmentationVisualizer::update_v2_cb, this, _1));
}

void
SegmentationVisualizer::update_v1_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  std::cout << "Received Segmented Cloud" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr blob(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg<pcl::PointXYZRGB>(*cloud_in,*blob);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> col_hdl(blob);
  if (!v_.updatePointCloud<pcl::PointXYZRGB>(blob, col_hdl, "segmented"))
    v_.addPointCloud<pcl::PointXYZRGB>(blob, col_hdl, "segmented", v1_);
}


void
SegmentationVisualizer::update_v2_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  std::cout << "Received Classified Cloud" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr blob(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg<pcl::PointXYZRGB>(*cloud_in,*blob);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> col_hdl(blob);
  if (!v_.updatePointCloud<pcl::PointXYZRGB>(blob, col_hdl, "classified"))
    v_.addPointCloud<pcl::PointXYZRGB>(blob, col_hdl, "classified", v2_);
}


int 
main(int argc, char **argv)
{
  ros::init(argc,argv, "SegmentationVisualizer");
  SegmentationVisualizer v;
  v.init();
  
  while (ros::ok() && v.isRunning())
  {
    ros::spinOnce();
    v.spinOnce();
    usleep(100000);
  }
  return 0;
}
