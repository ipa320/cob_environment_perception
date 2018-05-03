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
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_mapping_point_map
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2011
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


//##################
//#### includes ####

// ROS includes
//#include <ros/ros.h>
#include <ros/console.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
//#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>


/**
 * \brief Segments the difference of two point clouds (e.g. a point map and new sensor data) and publishes the difference.
 */
class DifferenceSegmentation : public nodelet::Nodelet//, protected Reconfigurable_Node<cob_3d_mapping_point_map::point_map_nodeletConfig>
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2 > MySyncPolicy;

public:
  // Constructor
  DifferenceSegmentation()
  {
  }

  // Destructor
  ~DifferenceSegmentation()
  {
    /// void
  }

  /**
   * @brief initializes parameters
   *
   * initializes parameters
   *
   * @return nothing
   */
  void
  onInit()
  {
    n_ = getNodeHandle();

    map_diff_pub_ = n_.advertise<PointCloud>("output",1);
    map_sub_.subscribe(n_,"target",10);
    pc_aligned_sub_.subscribe(n_,"input",10);
    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), map_sub_, pc_aligned_sub_));
    sync_->registerCallback(boost::bind(&DifferenceSegmentation::pointCloudSubCallback, this, _1, _2));

    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
    sd_.setSearchMethod(tree);
    //TODO: set to a value derived from the map resolution
    sd_.setDistanceThreshold(0.01);
  }

  /**
   * @brief callback for point cloud subroutine
   *
   * callback for point cloud subroutine which stores the point cloud
   * for further calculation
   *
   * @param pc_in  new point cloud
   *
   * @return nothing
   */
  void
  pointCloudSubCallback(const sensor_msgs::PointCloud2::ConstPtr& map_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_aligned_msg)
  {
    if(pc_aligned_msg->header.frame_id != map_msg->header.frame_id)
    {
      ROS_ERROR("Frame ID of incoming point cloud does not match map frame ID, aborting...");
      return;
    }
    PointCloud::Ptr pc_diff(new PointCloud);
    PointCloud::Ptr pc_aligned(new PointCloud);
    PointCloud::Ptr map(new PointCloud);

    pcl::PCLPointCloud2 pc_aligned2;
    pcl_conversions::toPCL(*pc_aligned_msg, pc_aligned2);
    pcl::fromPCLPointCloud2(pc_aligned2, *pc_aligned);
    pcl::PCLPointCloud2 map2;
    pcl_conversions::toPCL(*map_msg, map2);
    pcl::fromPCLPointCloud2(map2, *map);

    sd_.setTargetCloud(map_.makeShared());
    sd_.setInputCloud(pc_aligned);
    sd_.segment(*pc_diff);
    pc_diff->header = map->header;
    std::cout << pc_diff->size() << std::endl;
    if(pc_diff->size()>0)
    {
      map_diff_pub_.publish(pc_diff);
    }
    pcl::copyPointCloud(*map, map_);
  }

protected:
  ros::NodeHandle n_;
  message_filters::Subscriber<sensor_msgs::PointCloud2 > map_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2 > pc_aligned_sub_;
  ros::Publisher map_diff_pub_;		//publisher for map
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_;

  pcl::PointCloud<Point> map_;
  pcl::SegmentDifferences<Point> sd_;
};

PLUGINLIB_EXPORT_CLASS(DifferenceSegmentation, nodelet::Nodelet);

