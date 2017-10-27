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
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: cob_env_model
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2011
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

// standard includes
//--
#include <sstream>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/point_types.h>

#include <dynamic_reconfigure/server.h>
#include <cob_table_object_cluster/table_object_cluster_nodeletConfig.h>



// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetPlane.h>
#include <cob_3d_mapping_msgs/GetBoundingBoxes.h>

// external includes
#include <boost/timer.hpp>
#include <Eigen/StdVector>

#include "cob_table_object_cluster/table_object_cluster.h"
#include "cob_3d_mapping_msgs/TableObjectClusterAction.h"

using namespace cob_table_object_cluster;


//####################
//#### nodelet class ####
class TableObjectClusterNodelet : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZRGB Point;
  // Constructor
  TableObjectClusterNodelet()
  : as_(0)
  {
    save_to_file_ = table_object_cluster_nodeletConfig::__getDefault__().save;
    file_path_ = table_object_cluster_nodeletConfig::__getDefault__().file_path;
  }

  // Destructor
  ~TableObjectClusterNodelet()
  {
    /// void
    if(as_) delete as_;
  }

  /**
   * @brief Callback for dynamic reconfigure server
   *
   * Callback for dynamic reconfigure server
   *
   * @return nothing
   */
  void
  dynReconfCallback(table_object_cluster_nodeletConfig &config, uint32_t level)
  {
    save_to_file_ = config.save;
    file_path_ = config.file_path;
    /*height_min_ = config.height_min;
    height_max_ = config.height_max;
    min_cluster_size_ = config.min_cluster_size;
    cluster_tolerance_ = config.cluster_tolerance;*/
    toc.setPrismHeight(config.height_min, config.height_max);
    toc.setClusterParams(config.min_cluster_size, config.cluster_tolerance);
  }

  /**
   * @brief initializes parameters
   *
   * initializes parameters
   *
   * @return nothing
   */
  void onInit()
  {
    //PCLNodelet::onInit();
    n_ = getNodeHandle();

    config_server_.setCallback(boost::bind(&TableObjectClusterNodelet::dynReconfCallback, this, _1, _2));

    get_plane_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetPlane>("get_plane");
    get_bb_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetBoundingBoxes>("get_known_objects");
    as_= new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TableObjectClusterAction>(n_, "table_object_cluster", boost::bind(&TableObjectClusterNodelet::actionCallback, this, _1), false);
    as_->start();
  }


  /**
   * @brief detects objects on table and recognizes them
   *
   * detects objects on table and recognizes them
   *
   * @param goal unused
   *
   * @return nothing
   */
  void
  actionCallback(const cob_3d_mapping_msgs::TableObjectClusterGoalConstPtr &goal)
  {
    ROS_INFO("action callback");
    cob_3d_mapping_msgs::TableObjectClusterFeedback feedback;
    cob_3d_mapping_msgs::TableObjectClusterResult result;
    cob_3d_mapping_msgs::GetPlane srv;
    if(!get_plane_client_.call(srv))
    {
      ROS_ERROR("Failed to call service get_plane");
      as_->setAborted();
      return;
    }
    pcl::PointCloud<Point>::Ptr pc(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr hull(new pcl::PointCloud<Point>);
    pcl::fromROSMsg(srv.response.pc, *pc);
    pcl::fromROSMsg(srv.response.hull, *hull);
    Eigen::Vector4f plane_coeffs(srv.response.plane_coeffs[0].data,
                                 srv.response.plane_coeffs[1].data,
                                 srv.response.plane_coeffs[2].data,
                                 srv.response.plane_coeffs[3].data);
    ROS_INFO("Hull size: %d", hull->size());

    pcl::PointCloud<Point>::Ptr pc_roi(new pcl::PointCloud<Point>);
    toc.extractTableRoi(pc, hull, *pc_roi);
    //toc.extractTableRoi2(pc, hull, plane_coeffs, *pc_roi);
    ROS_INFO("ROI size: %d", pc_roi->size());
    //TODO: proceed also if no bbs are sent
    pcl::PointCloud<Point>::Ptr pc_roi_red(new pcl::PointCloud<Point>);
    cob_3d_mapping_msgs::GetBoundingBoxes srv2;
    if(get_bb_client_.call(srv2))
    {
      std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > known_objs;
      for(unsigned int i=0; i<srv2.response.bounding_boxes.size(); i++)
      {
        pcl::PointCloud<pcl::PointXYZ> obj;
        pcl::fromROSMsg(srv2.response.bounding_boxes[i], obj);
        known_objs.push_back(obj);
      }
      /*pcl::PointCloud<Point> obj;
      Point p;
      p.x = -1.5012188;
      p.y = 0.069459468;
      p.z = 0.88345075;
      obj.points.push_back(p);
      p.x = -1.4262178;
      p.y = 0.18113546;
      p.z = 1.0654262;
      obj.points.push_back(p);
      known_objs.push_back(obj);*/
      toc.removeKnownObjects(pc_roi, known_objs, *pc_roi_red);
    }
    else
    {
      ROS_WARN("Failed to call service get_bounding_boxes");
      pc_roi_red = pc_roi;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > bounding_boxes;
    toc.calculateBoundingBoxes(pc_roi_red,bounding_boxes);
    for(unsigned int i=0; i< bounding_boxes.size(); i++)
    {
      sensor_msgs::PointCloud2 bb;
      pcl::toROSMsg(bounding_boxes[i], bb);
      result.bounding_boxes.push_back(bb);
    }

    if(save_to_file_)
    {
      std::stringstream ss;
      ss << file_path_ << "/pc.pcd";
      pcl::io::savePCDFileASCII (ss.str(), *pc);
      ss.str("");
      ss.clear();
      ss << file_path_ << "/hull.pcd";
      pcl::io::savePCDFileASCII (ss.str(), *hull);
      ss.str("");
      ss.clear();
      ss << file_path_ << "/table_roi.pcd";
      pcl::io::savePCDFileASCII (ss.str(), *pc_roi);
      ss.str("");
      ss.clear();
      ss << file_path_ << "/table_roi_red.pcd";
      pcl::io::savePCDFileASCII (ss.str(), *pc_roi_red);
      ss.str("");
      ss.clear();
      for(unsigned int i=0; i< bounding_boxes.size(); i++)
      {
        ss << file_path_ << "/bb_" << i << ".pcd";
        pcl::io::savePCDFileASCII (ss.str(), bounding_boxes[i]);
        ss.str("");
        ss.clear();
      }
    }
    as_->setSucceeded(result);
  }


  ros::NodeHandle n_;


protected:
  actionlib::SimpleActionServer<cob_3d_mapping_msgs::TableObjectClusterAction>* as_;
  ros::ServiceClient get_plane_client_;
  ros::ServiceClient get_bb_client_;
  dynamic_reconfigure::Server<table_object_cluster_nodeletConfig> config_server_;
  boost::mutex mutex_;

  TableObjectCluster toc;       /// class for actual calculation

  bool save_to_file_;
  std::string file_path_;
  /*double height_min_;           /// paramter for object detection
  double height_max_;           /// paramter for object detection
  int min_cluster_size_;        /// paramter for object detection
  double cluster_tolerance_;    /// paramter for object detection*/

};

PLUGINLIB_EXPORT_CLASS(TableObjectClusterNodelet, nodelet::Nodelet);
