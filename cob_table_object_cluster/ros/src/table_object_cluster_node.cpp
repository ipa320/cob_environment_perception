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
 *  ROS package name: cob_table_object_cluster
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 03/2012
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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
//#include <pluginlib/class_list_macros.h>
//#include <pcl_ros/pcl_nodelet.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/point_types.h>

#include <dynamic_reconfigure/server.h>
//#include <cob_3d_mapping_common/reconfigureable_node.h>
#include <cob_table_object_cluster/table_object_cluster_nodeletConfig.h>
#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetPointMap.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_perception_msgs/PointCloud2Array.h>
#include <visualization_msgs/Marker.h>
//#include <cob_3d_mapping_msgs/GetBoundingBoxes.h>

// external includes
#include <boost/timer.hpp>
#include <Eigen/StdVector>

#include "cob_table_object_cluster/table_object_cluster.h"
#include "cob_3d_mapping_msgs/TableObjectClusterAction.h"

using namespace cob_table_object_cluster;
using namespace cob_3d_mapping;


//####################
//#### nodelet class ####
class TableObjectClusterNode //: protected Reconfigurable_Node<table_object_cluster_nodeletConfig>
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef message_filters::sync_policies::ExactTime<PointCloud, cob_3d_mapping_msgs::ShapeArray > MySyncPolicy;
  // Constructor
  TableObjectClusterNode()
  : as_(0),
    ctr_(0)
  {
    config_server_.setCallback(boost::bind(&TableObjectClusterNode::dynReconfCallback, this, _1, _2));

    //n_ = getNodeHandle();
    bb_pub_ = n_.advertise<visualization_msgs::Marker>("bb_marker",100);
    object_cluster_pub_ = n_.advertise<cob_perception_msgs::PointCloud2Array>("cluster_array",1);
    get_point_map_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetPointMap>("get_point_map");
    //get_bb_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetBoundingBoxes>("get_known_objects");
    as_= new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TableObjectClusterAction>(n_, "table_object_cluster", boost::bind(&TableObjectClusterNode::actionCallback, this, _1), false);
    as_->start();

    pc_sub_.subscribe(n_,"point_cloud",10);
    sa_sub_.subscribe(n_,"table_array",10);
    sync_ = boost::make_shared <message_filters::Synchronizer<MySyncPolicy> >(100);
    sync_->connectInput(pc_sub_, sa_sub_);
    sync_->registerCallback(boost::bind(&TableObjectClusterNode::topicCallback, this, _1, _2));
  }

  // Destructor
  ~TableObjectClusterNode()
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
    if(ctr_) deleteMarker();
    ctr_= 0;
    cob_3d_mapping_msgs::TableObjectClusterFeedback feedback;
    cob_3d_mapping_msgs::TableObjectClusterResult result;
    cob_3d_mapping_msgs::GetPointMap srv;
    if(!get_point_map_client_.call(srv))
    {
      ROS_ERROR("Failed to call service get_point_map");
      as_->setAborted();
      return;
    }
    pcl::PointCloud<Point>::Ptr map(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr hull(new pcl::PointCloud<Point>);
    pcl::fromROSMsg(srv.response.map, *map);
    pcl::fromROSMsg(goal->table_hull, *hull);
    /*Eigen::Vector4f plane_coeffs(srv.response.plane_coeffs[0].data,
                                 srv.response.plane_coeffs[1].data,
                                 srv.response.plane_coeffs[2].data,
                                 srv.response.plane_coeffs[3].data);*/
    ROS_INFO("Hull size: %d", hull->size());

    pcl::PointCloud<Point>::Ptr pc_roi(new pcl::PointCloud<Point>);
    toc.extractTableRoi(map, hull, *pc_roi);
    //toc.extractTableRoi2(pc, hull, plane_coeffs, *pc_roi);
    ROS_INFO("ROI size: %d", pc_roi->size());
    //TODO: proceed also if no bbs are sent
    pcl::PointCloud<Point>::Ptr pc_roi_red(new pcl::PointCloud<Point>);
    /*cob_3d_mapping_msgs::GetBoundingBoxes srv2;
    if(get_bb_client_.call(srv2))
    {
      std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > > known_objs;
      for(unsigned int i=0; i<srv2.response.bounding_boxes.size(); i++)
      {
        pcl::PointCloud<Point> obj;
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
      known_objs.push_back(obj);
      toc.removeKnownObjects(pc_roi, known_objs, *pc_roi_red);
    }
    else
    {
      ROS_WARN("Failed to call service get_bounding_boxes");
      pc_roi_red = pc_roi;
    }*/

    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > bounding_boxes;
    std::vector<PointCloud::Ptr> object_clusters;
    toc.calculateBoundingBoxes(pc_roi, object_clusters, bounding_boxes);
    for(unsigned int i=0; i< bounding_boxes.size(); i++)
    {
      sensor_msgs::PointCloud2 bb;
      pcl::toROSMsg(bounding_boxes[i], bb);
      result.bounding_boxes.push_back(bb);
      publishMarker(bounding_boxes[i]);
    }

    if(save_to_file_)
    {
      std::stringstream ss;
      ss << file_path_ << "/pc.pcd";
      pcl::io::savePCDFileASCII (ss.str(), *map);
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

  void
  topicCallback(const PointCloud::ConstPtr& pc, const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa)
  {
    //PointCloud pc_in = *pc;
    //PointCloud::Ptr pc_in_ptr = pc->makeShared();
    for( unsigned int i=0; i< sa->shapes.size(); i++)
    {
      Polygon::Ptr p(new Polygon());
      fromROSMsg(sa->shapes[i], *p);
      //TODO: check holes
      int no_contour=-1;
      for( unsigned int j=0; j<p->contours.size(); j++)
      {
        if( p->holes[j] == 0 )
        {
          no_contour = j;
          break;
        }
      }
      if( no_contour == -1)
      {
        ROS_ERROR("Polygon has no positive contour, aborting...");
        return;
      }
      PointCloud::Ptr hull(new PointCloud);
      for( unsigned int j=0; j<p->contours[no_contour].size(); j++)
      {
        Point pt;
        pt.x = p->contours[no_contour][j][0];
        pt.y = p->contours[no_contour][j][1];
        pt.z = p->contours[no_contour][j][2];
        hull->points.push_back(pt);
      }
      hull->width = hull->size();
      hull->height = 1;
      PointCloud::Ptr pc_roi(new PointCloud);
      toc.extractTableRoi(pc, hull, *pc_roi);
      std::stringstream ss;
      if(save_to_file_)
      {
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
      }
      std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > bounding_boxes;
      std::vector<PointCloud::Ptr> object_clusters;
      toc.calculateBoundingBoxes(pc_roi, object_clusters, bounding_boxes);
      ROS_INFO("Computed %d bounding boxes", object_clusters.size());
      cob_perception_msgs::PointCloud2Array pca;
      pca.header = pc->header;
      for(unsigned int j=0; j<object_clusters.size(); j++)
      {
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*object_clusters[j], pc_msg);
        pc_msg.header = pc->header;
        pca.segments.push_back(pc_msg);
        if(save_to_file_)
        {
          ss << file_path_ << "/bb_" << j << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), *object_clusters[j]);
          ss.str("");
          ss.clear();
        }
      }
      object_cluster_pub_.publish(pca);
    }
  }

  /**
   * @brief publishes a bounding box
   *
   * publishes a bounding box
   *
   * @return nothing
   */
  void
  publishMarker(pcl::PointCloud<pcl::PointXYZ>& bb)
  {
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.lifetime = ros::Duration();
    marker.header.frame_id = "/map";
    marker.id = ctr_;

    //marker.header.stamp = stamp;

    //create the marker in the table reference frame
    //the caller is responsible for setting the pose of the marker to match

    marker.scale.x = 0.02;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1.0;

    marker.points.resize(24);
    geometry_msgs::Point pt;
    pt.x = bb.points[0].x;
    pt.y = bb.points[0].y;
    pt.z = bb.points[0].z;
    marker.points[0] = marker.points[2] = marker.points[4] = pt;
    pt.x = bb.points[1].x;
    marker.points[1] = marker.points[12] = marker.points[14] = pt;
    pt.y = bb.points[1].y;
    marker.points[11] = marker.points[15] = marker.points[19] = pt;
    pt.z = bb.points[1].z;
    marker.points[6] = marker.points[8] = marker.points[10] = pt;
    pt.x = bb.points[0].x;
    marker.points[7] = marker.points[17] = marker.points[23] = pt;
    pt.y = bb.points[0].y;
    marker.points[5] = marker.points[20] = marker.points[22] = pt;
    pt.x = bb.points[1].x;
    marker.points[9] = marker.points[13] = marker.points[21] = pt;
    pt.x = bb.points[0].x;
    pt.y = bb.points[1].y;
    pt.z = bb.points[0].z;
    marker.points[3] = marker.points[16] = marker.points[18] = pt;
    bb_pub_.publish(marker);
    ctr_++;
  }

  void
  deleteMarker()
  {
    for( unsigned int i=0; i<ctr_; i++)
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.id = i;
      bb_pub_.publish(marker);
    }
  }


  ros::NodeHandle n_;


protected:
  actionlib::SimpleActionServer<cob_3d_mapping_msgs::TableObjectClusterAction>* as_;
  ros::ServiceClient get_point_map_client_;
  ros::Publisher bb_pub_;
  ros::Publisher object_cluster_pub_;
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_;
  message_filters::Subscriber<PointCloud> pc_sub_;
  message_filters::Subscriber<cob_3d_mapping_msgs::ShapeArray> sa_sub_;
  dynamic_reconfigure::Server<table_object_cluster_nodeletConfig> config_server_;
  //ros::ServiceClient get_bb_client_;
  boost::mutex mutex_;

  TableObjectCluster toc;       /// class for actual calculation
  unsigned int ctr_;

  bool save_to_file_;
  std::string file_path_;
  /*double height_min_;           /// paramter for object detection
  double height_max_;           /// paramter for object detection
  int min_cluster_size_;        /// paramter for object detection
  double cluster_tolerance_;    /// paramter for object detection*/

};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "table_object_cluster_node");

  TableObjectClusterNode toc;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep();
  }
}

