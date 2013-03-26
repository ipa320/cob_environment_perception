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
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
//#include <pluginlib/class_list_macros.h>
//#include <nodelet/nodelet.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#ifdef PCL_VERSION_COMPARE
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif

//#include <cob_3d_mapping_common/reconfigureable_node.h>
#include <cob_table_object_cluster/table_object_cluster_nodeletConfig.h>
#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetPointMap.h>
#include <cob_3d_mapping_msgs/GetBoundingBoxes.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_perception_msgs/PointCloud2Array.h>
#include <visualization_msgs/Marker.h>

// external includes
#include <boost/timer.hpp>
#include <Eigen/StdVector>

#include "cob_table_object_cluster/table_object_cluster.h"
#include "cob_3d_mapping_msgs/TableObjectClusterAction.h"

#include <cob_3d_mapping_common/stop_watch.h>
#include <pcl/filters/extract_indices.h>

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
  : as_(0)
  , ctr_(0)
  , enable_action_mode_(false)
  , enable_tf_(false)
  , frame_id_("/map")
  , last_pc_(new PointCloud)
  , last_tf_(Eigen::Affine3f::Identity())
  {
    config_server_.setCallback(boost::bind(&TableObjectClusterNode::dynReconfCallback, this, _1, _2));

    //n_ = getNodeHandle();
    bb_pub_ = n_.advertise<visualization_msgs::Marker>("bb_marker",100);
    object_cluster_pub_ = n_.advertise<cob_perception_msgs::PointCloud2Array>("cluster_array",1);
    get_point_map_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetPointMap>("get_point_map");
    get_bb_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetBoundingBoxes>("get_known_objects");
    as_= new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TableObjectClusterAction>(n_, "table_object_cluster", boost::bind(&TableObjectClusterNode::actionCallback, this, _1), false);
    as_->start();

    pc_sub_.subscribe(n_,"point_cloud",1);
    sa_sub_.subscribe(n_,"shape_array",1);
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
    enable_action_mode_ = config.enable_action_mode;
    enable_tf_ = config.enable_tf;
    frame_id_ = config.map_frame_id;
    toc.setPrismHeight(config.height_min, config.height_max);
    toc.setClusterParams(config.min_cluster_size, config.cluster_tolerance);
  }

  void
  compute(std::vector<pcl::PointCloud<pcl::PointXYZ> >& bounding_boxes,
          cob_perception_msgs::PointCloud2Array& pca)
  {
    toc.setInputCloud(last_pc_);
    for(size_t i=0; i<last_sa_->shapes.size(); ++i)
    {
      pcl::PointCloud<Point>::Ptr hull(new pcl::PointCloud<Point>);
      shape2hull<Point>(last_sa_->shapes[i], *hull);

      if(enable_tf_) pcl::transformPointCloud(*hull, *hull, last_tf_);

      pcl::PointIndices::Ptr pc_roi(new pcl::PointIndices);
      PrecisionStopWatch sw;
      sw.precisionStart();
      toc.extractTableRoi(hull, *pc_roi);
      ROS_DEBUG("ROI took %f seconds", sw.precisionStop());
      ROS_DEBUG("ROI has %d points", pc_roi->indices.size());
      if(!pc_roi->indices.size()) return;
      std::stringstream ss;
      if(save_to_file_)
      {
        ss << file_path_ << "/pc.pcd";
        pcl::io::savePCDFileASCII (ss.str(), *last_pc_);
        ss.str("");
        ss.clear();
        ss << file_path_ << "/hull.pcd";
        pcl::io::savePCDFileASCII (ss.str(), *hull);
        ss.str("");
        ss.clear();
        ss << file_path_ << "/table_roi.pcd";
        PointCloud roi;
        pcl::ExtractIndices<Point> extract_roi;
        extract_roi.setInputCloud (last_pc_);
        extract_roi.setIndices (pc_roi);
        extract_roi.filter (roi);
        pcl::io::savePCDFileASCII (ss.str(), roi);
        ss.str("");
        ss.clear();
      }

      //std::vector<pcl::PointCloud<pcl::PointXYZ> > bounding_boxes;
      bounding_boxes.clear();
      std::vector<PointCloud::Ptr> object_clusters;
      sw.precisionStart();
      toc.calculateBoundingBoxes(pc_roi, object_clusters, bounding_boxes);
      ROS_DEBUG("BB took %f seconds", sw.precisionStop());
      ROS_INFO("Computed %d bounding boxes", object_clusters.size());

      //cob_perception_msgs::PointCloud2Array pca;
      pca.segments.clear();
      pca.header = last_pc_->header;
      for(unsigned int j=0; j<object_clusters.size(); j++)
      {
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*object_clusters[j], pc_msg);
        pc_msg.header = last_pc_->header;
        pca.segments.push_back(pc_msg);
        if(save_to_file_)
        {
          ss << file_path_ << "/cl_" << j << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), *object_clusters[j]);
          ss.str("");
          ss.clear();
          ss << file_path_ << "/bb_" << j << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), bounding_boxes[j]);
          ss.str("");
          ss.clear();
        }
      }
    }
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
    boost::lock_guard<boost::mutex> guard(mutex_);
    ROS_INFO("action callback");
    if(ctr_) deleteMarker(); // there is no assignment of ctr_ anywhere else
    ctr_= 0;
    cob_3d_mapping_msgs::TableObjectClusterFeedback feedback;
    cob_3d_mapping_msgs::TableObjectClusterResult result;

    cob_3d_mapping_msgs::GetBoundingBoxes srv;
    if(get_bb_client_.call(srv))
    {
      std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > known_objs;
      for(unsigned int i=0; i<srv.response.bounding_boxes.size(); i++)
      {
        pcl::PointCloud<pcl::PointXYZ> obj;
        pcl::fromROSMsg(srv.response.bounding_boxes[i], obj);
        known_objs.push_back(obj);
      }
      pcl::PointCloud<Point>::Ptr pc_red(new pcl::PointCloud<Point>);
      toc.removeKnownObjects(last_pc_, known_objs, *pc_red);
      *last_pc_ = *pc_red;
    }
    else
    {
      ROS_WARN("Failed to call service get_bounding_boxes");
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ> > bba;
    cob_perception_msgs::PointCloud2Array pca;
    compute(bba, pca);

    publishMarker(bba);
    //object_cluster_pub_.publish(pca);

    for(unsigned int i=0; i< bba.size(); ++i)
    {
      sensor_msgs::PointCloud2 bb;
      pcl::toROSMsg(bba[i], bb);
      result.bounding_boxes.push_back(bb);
    }

    as_->setSucceeded(result);
  }

  void
  topicCallback(const PointCloud::ConstPtr& pc, const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa)
  {
    boost::lock_guard<boost::mutex> guard(mutex_);

    if(enable_tf_)
    {
      tf::StampedTransform trf_map;
      try
      {
        tf_listener_.waitForTransform(frame_id_, sa->header.frame_id, sa->header.stamp, ros::Duration(2));
        tf_listener_.lookupTransform(frame_id_, sa->header.frame_id, sa->header.stamp, trf_map);
      }
      catch (tf::TransformException ex) { ROS_ERROR("[geometry map node] : %s",ex.what()); return; }

      Eigen::Affine3d ad;
      tf::TransformTFToEigen(trf_map, ad);
      last_tf_ = ad.cast<float>();
      pcl::transformPointCloud(*pc, *last_pc_, last_tf_);
    }
    else
    {
      frame_id_ = sa->header.frame_id;
      *last_pc_ = *pc; // deep copy, required for removing points later on
    }

    last_sa_ = sa; // copy shape array pointer

    if(enable_action_mode_) { return; }

    std::vector<pcl::PointCloud<pcl::PointXYZ> > bba;
    cob_perception_msgs::PointCloud2Array pca;
    compute(bba, pca);

    publishMarker(bba);
    object_cluster_pub_.publish(pca);
  }

  /**
   * @brief publishes a bounding box
   *
   * publishes a bounding box
   *
   * @return nothing
   */
  void
  publishMarker(std::vector<pcl::PointCloud<pcl::PointXYZ> >& bb)
  {
    for(unsigned int i=0; i<bb.size(); i++)
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.lifetime = ros::Duration();
      marker.header.frame_id = frame_id_;
      marker.id = i;

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
      pt.x = bb[i].points[0].x;
      pt.y = bb[i].points[0].y;
      pt.z = bb[i].points[0].z;
      marker.points[0] = marker.points[2] = marker.points[4] = pt;
      pt.x = bb[i].points[1].x;
      marker.points[1] = marker.points[12] = marker.points[14] = pt;
      pt.y = bb[i].points[1].y;
      marker.points[11] = marker.points[15] = marker.points[19] = pt;
      pt.z = bb[i].points[1].z;
      marker.points[6] = marker.points[8] = marker.points[10] = pt;
      pt.x = bb[i].points[0].x;
      marker.points[7] = marker.points[17] = marker.points[23] = pt;
      pt.y = bb[i].points[0].y;
      marker.points[5] = marker.points[20] = marker.points[22] = pt;
      pt.x = bb[i].points[1].x;
      marker.points[9] = marker.points[13] = marker.points[21] = pt;
      pt.x = bb[i].points[0].x;
      pt.y = bb[i].points[1].y;
      pt.z = bb[i].points[0].z;
      marker.points[3] = marker.points[16] = marker.points[18] = pt;
      bb_pub_.publish(marker);
    }
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
  ros::ServiceClient get_bb_client_;
  ros::Publisher bb_pub_;
  ros::Publisher object_cluster_pub_;
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_;
  message_filters::Subscriber<PointCloud> pc_sub_;
  message_filters::Subscriber<cob_3d_mapping_msgs::ShapeArray> sa_sub_;
  dynamic_reconfigure::Server<table_object_cluster_nodeletConfig> config_server_;
  tf::TransformListener tf_listener_;

  boost::mutex mutex_;

  TableObjectCluster toc;       /// class for actual calculation
  unsigned int ctr_;

  bool save_to_file_;
  std::string file_path_;
  /*double height_min_;           /// paramter for object detection
  double height_max_;           /// paramter for object detection
  int min_cluster_size_;        /// paramter for object detection
  double cluster_tolerance_;    /// paramter for object detection*/
  bool enable_action_mode_;
  bool enable_tf_;
  std::string frame_id_;

  PointCloud::Ptr last_pc_;
  cob_3d_mapping_msgs::ShapeArray::ConstPtr last_sa_;
  Eigen::Affine3f last_tf_;
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

