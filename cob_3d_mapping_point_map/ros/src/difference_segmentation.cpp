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
 *  ROS package name: cob_3d_mapping_point_cloud
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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
//#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/kdtree/kdtree_flann.h>


//####################
//#### node class ####
class DifferenceSegmentation : public nodelet::Nodelet//, protected Reconfigurable_Node<cob_3d_mapping_point_map::point_map_nodeletConfig>
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef message_filters::sync_policies::ExactTime<PointCloud, PointCloud > MySyncPolicy;

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
    //diff_maps_pub_ = n_.advertise<PointCloud >("diff_maps",1);
    map_sub_.subscribe(n_,"target",10);
    pc_aligned_sub_.subscribe(n_,"input",10);
    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), map_sub_, pc_aligned_sub_));
    //sync_->connectInput(map_sub_, pc_aligned_sub_);
    sync_->registerCallback(boost::bind(&DifferenceSegmentation::pointCloudSubCallback, this, _1, _2));

#ifdef PCL_VERSION_COMPARE //fuerte
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
#else //electric
    pcl::KdTreeFLANN<Point>::Ptr tree (new pcl::KdTreeFLANN<Point> ());
#endif
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
  pointCloudSubCallback(const PointCloud::ConstPtr& map, const PointCloud::ConstPtr& pc_aligned)
  {
    if(pc_aligned->header.frame_id != map->header.frame_id)
    {
      ROS_ERROR("Frame ID of incoming point cloud does not match map frame ID, aborting...");
      return;
    }
    PointCloud::Ptr pc_diff(new PointCloud);
    /*std::cout << "map size:" << map->size() << std::endl;
    std::cout << map->header.stamp << std::endl;
    std::cout << "pc_aligned size:" << pc_aligned->size() << std::endl;
    std::cout << pc_aligned->header.stamp << std::endl;*/
    /*if(map->header.frame_id != pc_aligned->header.frame_id)
    {
      ROS_ERROR("Frame IDs do not match, aborting...");
      return;
    }*/
    //diff_maps_.header.frame_id=map->header.frame_id;
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
  message_filters::Subscriber<PointCloud > map_sub_;
  message_filters::Subscriber<PointCloud > pc_aligned_sub_;
  ros::Publisher map_diff_pub_;		//publisher for map
  //ros::Publisher diff_maps_pub_;
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_;

  pcl::PointCloud<Point> map_;
  //pcl::PointCloud<Point> diff_maps_;
  pcl::SegmentDifferences<Point> sd_;
};

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_point_map, DifferenceSegmentation, DifferenceSegmentation, nodelet::Nodelet)

