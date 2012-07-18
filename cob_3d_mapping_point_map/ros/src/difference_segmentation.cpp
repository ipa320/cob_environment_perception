/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception
 * ROS package name: cob_3d_mapping_point_cloud
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
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


//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <fstream>


// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cob_3d_mapping_common/point_types.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_3d_mapping_point_map/impl/field_of_view_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
//#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetFieldOfView.h>
#include "cob_3d_mapping_msgs/TriggerMappingAction.h"
#include <cob_3d_mapping_msgs/SetReferenceMap.h>
#include <cob_3d_mapping_msgs/GetPointMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "cob_3d_mapping_point_map/point_map.h"

#include "cob_3d_mapping_common/reconfigureable_node.h"
#include <cob_3d_mapping_point_map/point_map_nodeletConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/segmentation/segment_differences.h>




using namespace tf;

//####################
//#### node class ####
class DifferenceSegmentation : public pcl_ros::PCLNodelet//, protected Reconfigurable_Node<cob_3d_mapping_point_map::point_map_nodeletConfig>
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef message_filters::sync_policies::ExactTime<PointCloud, PointCloud > MySyncPolicy;

public:
  // Constructor
  DifferenceSegmentation()
  //: Reconfigurable_Node<cob_3d_mapping_point_map::point_map_nodeletConfig>("PointMapNodelet")
    {
    //setReconfigureCallback2(boost::bind(&callback, this, _1, _2), boost::bind(&callback_get, this, _1));
    }


  // Destructor
  ~DifferenceSegmentation()
  {
    /// void
  }

  /**
   * @brief callback for dynamic reconfigure
   *
   * everytime the dynamic reconfiguration changes this function will be called
   *
   * @param inst instance of PointMapNodelet which parameters should be changed
   * @param config data of configuration
   * @param level bit descriptor which notifies which parameter changed
   *
   * @return nothing
   */
  static void callback(DifferenceSegmentation *inst, cob_3d_mapping_point_map::point_map_nodeletConfig &config, uint32_t level)
  {
    if(!inst)
      return;

    inst->file_path_ = config.file_path;
    inst->save_ = config.save;

  }

  // callback for dynamic reconfigure
  static void callback_get(DifferenceSegmentation *inst, cob_3d_mapping_point_map::point_map_nodeletConfig &config)
  {
    if(!inst)
      return;

    config.file_path=inst->file_path_;
    config.save = inst->save_;

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
    PCLNodelet::onInit();
    n_ = getNodeHandle();

    map_diff_pub_ = n_.advertise<PointCloud >("output",1);
    diff_maps_pub_ = n_.advertise<PointCloud >("diff_maps",1);
    map_sub_.subscribe(n_,"target",1);
    pc_aligned_sub_.subscribe(n_,"input",1);
    sync_ = boost::make_shared <message_filters::Synchronizer<MySyncPolicy> >(10);
    sync_->connectInput(map_sub_, pc_aligned_sub_);
    sync_->registerCallback(boost::bind(&DifferenceSegmentation::pointCloudSubCallback, this, _1, _2));

    n_.param("aggregate_point_map/file_path" ,file_path_ ,std::string("~/pcl_daten/table/icp/map_"));
    n_.param("aggregate_point_map/save",save_ , false);

    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
    sd_.setSearchMethod(tree);
    sd_.setDistanceThreshold(0.001);
    diff_maps_.header.frame_id="/map";
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
    static int ctr=0;
    PointCloud pc_diff;
    std::cout << "map size:" << map->size() << std::endl;
    std::cout << map->header.stamp << std::endl;
    std::cout << "pc_aligned size:" << pc_aligned->size() << std::endl;
    std::cout << pc_aligned->header.stamp << std::endl;
    //pcl::fromROSMsg(pc_aligned_msg, pc_aligned);
    //pcl::SegmentDifferences<Point> sd;
    sd_.setTargetCloud(map_.makeShared());
    sd_.setInputCloud(pc_aligned);
    sd_.segment(pc_diff);
    std::cout << pc_diff.size() << std::endl;
    if(pc_diff.size()>0)
    {
      /*std::stringstream ss;
      ss << "/home/goa/pcl_daten/diff_map/diff_" << ctr << ".pcd";
      pcl::io::savePCDFileASCII (ss.str(), pc_diff);
      std::stringstream ss1;
      ss1 << "/home/goa/pcl_daten/diff_map/map_" << ctr << ".pcd";
      if(map_.size()>0)
        pcl::io::savePCDFileASCII (ss1.str(), map_);
      std::stringstream ss2;
      ss2 << "/home/goa/pcl_daten/diff_map/pc_aligned_" << ctr << ".pcd";
      pcl::io::savePCDFileASCII (ss2.str(), *pc_aligned);
      ctr++;*/
      //pc_in_ = *pc_in;
      map_diff_pub_.publish(pc_diff);
      //diff_maps_ += pc_diff;
      //diff_maps_pub_.publish(diff_maps_);
    }
    pcl::copyPointCloud(*map, map_);
  }


  ros::NodeHandle n_;
  ros::Time stamp_;


protected:
  message_filters::Subscriber<PointCloud > map_sub_;
  message_filters::Subscriber<PointCloud > pc_aligned_sub_;
  ros::Publisher map_diff_pub_;		//publisher for map
  ros::Publisher diff_maps_pub_;
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_;

  TransformListener tf_listener_;

  pcl::PointCloud<Point> map_;
  pcl::PointCloud<Point> diff_maps_;
  pcl::SegmentDifferences<Point> sd_;


  // Parameters for file saving
  std::string file_path_;
  bool save_;



};

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_point_map, DifferenceSegmentation, DifferenceSegmentation, nodelet::Nodelet)

