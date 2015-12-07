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
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PointIndices.h>
#include <eigen_conversions/eigen_msg.h>

//#include <cob_3d_mapping_common/reconfigureable_node.h>
#include <cob_table_object_cluster/table_object_cluster_nodeletConfig.h>
#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetPointMap.h>
#include <cob_3d_mapping_msgs/SetBoundingBoxes.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_perception_msgs/PointCloud2Array.h>
#include <cob_object_detection_msgs/Detection.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <cob_vision_utils/ros_msg_conversions.h>
#include <visualization_msgs/Marker.h>

// external includes
#include <boost/timer.hpp>
#include <Eigen/StdVector>

#include "cob_table_object_cluster/table_object_cluster.h"
#include "cob_3d_mapping_msgs/TableObjectClusterAction.h"

#include <cob_3d_mapping_common/stop_watch.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cob_table_object_cluster;
using namespace cob_3d_mapping;


//####################
//#### nodelet class ####
class TableObjectClusterNode //: protected Reconfigurable_Node<table_object_cluster_nodeletConfig>
{
public:
  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef message_filters::sync_policies::ExactTime<PointCloud, cob_3d_mapping_msgs::ShapeArray > MySyncPolicy;
  // Constructor
  TableObjectClusterNode()
  : as_(0)
  , ctr_(0)
  , enable_action_mode_(false)
  , last_pc_(new PointCloud)
  {
    config_server_.setCallback(boost::bind(&TableObjectClusterNode::dynReconfCallback, this, _1, _2));

    // subscriber:
    pc_sub_.subscribe(n_,"point_cloud",1);
    sa_sub_.subscribe(n_,"shape_array",1);
    sync_ = boost::make_shared <message_filters::Synchronizer<MySyncPolicy> >(100);
    sync_->connectInput(pc_sub_, sa_sub_);
    sync_->registerCallback(boost::bind(&TableObjectClusterNode::topicCallback, this, _1, _2));

    // publisher:
    bba_pub_ = n_.advertise<cob_object_detection_msgs::DetectionArray>("bounding_box_array", 1);
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("bb_marker",100);
    object_cluster_pub_ = n_.advertise<cob_perception_msgs::PointCloud2Array>("cluster_array",1);

    // Services and Actions
    //get_bb_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetBoundingBoxes>("get_known_objects");
    set_known_objects_server_ = n_.advertiseService("set_known_objects", &TableObjectClusterNode::setKnownObjects, this);
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
    if(enable_action_mode_)
    {
      std::cout << "action mode not supported" << std::endl;
      //as_= new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TableObjectClusterAction>(n_, "table_object_cluster", boost::bind(&TableObjectClusterNode::actionCallback, this, _1), false);
      //as_->start();
    }
    toc.setPrismHeight(config.height_min, config.height_max);
    toc.setClusterParams(config.min_cluster_size, config.cluster_tolerance);
  }

  bool
  setKnownObjects(cob_3d_mapping_msgs::SetBoundingBoxes::Request& req, cob_3d_mapping_msgs::SetBoundingBoxes::Response& res)
  {
    known_objects_.clear();
    for(unsigned int i=0; i<req.bounding_boxes.detections.size(); i++)
    {
      BoundingBox bb;
      bb.min_pt = Eigen::Vector4f(-req.bounding_boxes.detections[i].bounding_box_lwh.x,
                                  -req.bounding_boxes.detections[i].bounding_box_lwh.y, 0, 1.0);
      bb.max_pt = Eigen::Vector4f(req.bounding_boxes.detections[i].bounding_box_lwh.x,
                                  req.bounding_boxes.detections[i].bounding_box_lwh.y,
                                  req.bounding_boxes.detections[i].bounding_box_lwh.z, 1.0);
      Eigen::Affine3d tf;
      tf::poseMsgToEigen(req.bounding_boxes.detections[i].pose.pose , tf);
      bb.pose = tf.cast<float>();
      known_objects_.push_back(bb);
    }
    return true;
  }

  void
  compute(cob_object_detection_msgs::DetectionArray& bba,
          cob_perception_msgs::PointCloud2Array& pca)
  {
    pcl::PointCloud<Point>::Ptr pc_red(new pcl::PointCloud<Point>);
    toc.removeKnownObjects(last_pc_, known_objects_, pc_red);
    //*last_pc_ = *pc_red;
    toc.setInputCloud(pc_red);
    for(size_t i=0; i<last_sa_->shapes.size(); ++i)
    {
      //convert shape msgs to point cloud
      pcl::PointCloud<Point>::Ptr hull(new pcl::PointCloud<Point>);
      shape2hull<Point>(last_sa_->shapes[i], *hull);

      pcl::PointIndices::Ptr pc_roi(new pcl::PointIndices);
      PrecisionStopWatch sw;
      sw.precisionStart();

      // get points above table
      toc.extractTableRoi(hull, *pc_roi);
      ROS_INFO("ROI took %f seconds", sw.precisionStop());
      ROS_INFO("ROI has %d points", (int)pc_roi->indices.size());
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

      std::vector<PointCloud::Ptr> object_clusters;
      std::vector<pcl::PointIndices> object_cluster_indices;
      sw.precisionStart();
      toc.extractClusters(pc_roi, object_clusters, object_cluster_indices);
      ROS_INFO("extract clusters took %f seconds", sw.precisionStop());
      sw.precisionStart();
      pca.segments.clear();
      pcl_conversions::fromPCL(last_pc_->header, pca.header);
      //pca.header = last_pc_->header;

      Eigen::Vector3f normal(last_sa_->shapes[i].params[0],
                             last_sa_->shapes[i].params[1],
                             last_sa_->shapes[i].params[2]);
      Eigen::Vector3f point(last_sa_->shapes[i].pose.position.x,
                            last_sa_->shapes[i].pose.position.y,
                            last_sa_->shapes[i].pose.position.z);

      bba.detections.clear();
      pcl_conversions::fromPCL(last_pc_->header, bba.header);
      //bba.header.stamp = last_pc_->header.stamp;
      //bba.header.frame_id = last_pc_->header.frame_id;
      pca.segments.resize(object_clusters.size());
      for(unsigned int j=0; j<object_clusters.size(); j++)
      {
        Eigen::Vector3f pos;
        Eigen::Quaternion<float> rot;
        Eigen::Vector3f size;

        toc.calculateBoundingBox(last_pc_, object_cluster_indices[j], normal, point, pos, rot, size);

        bba.detections.push_back(cob_object_detection_msgs::Detection());
        pcl_conversions::fromPCL(last_pc_->header, bba.detections.back().header);
        //bba.detections.back().header.stamp = last_pc_->header.stamp;
        //bba.detections.back().header.frame_id = last_pc_->header.frame_id;
        bba.detections.back().label = "Object Cluster";
        bba.detections.back().detector = "BoundingBoxDetector";
        bba.detections.back().score = 0;
        //bba.detections.back().mask;
        pcl_conversions::fromPCL(last_pc_->header, bba.detections.back().pose.header);
        //bba.detections.back().pose.header.stamp = last_pc_->header.stamp;
        //bba.detections.back().pose.header.frame_id = last_pc_->header.frame_id;
        cob_perception_common::EigenToROSMsg(pos,rot,bba.detections.back().pose.pose);
        cob_perception_common::EigenToROSMsg(size, bba.detections.back().bounding_box_lwh);

        pcl::toROSMsg(*object_clusters[j], pca.segments[j]);
        pcl_conversions::fromPCL(last_pc_->header, pca.segments[j].header);
        //pca.segments[j].header = last_pc_->header;


        if(save_to_file_)
        {
          ss << file_path_ << "/cl_" << j << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), *object_clusters[j]);
          ss.str("");
          ss.clear();
          /*
          ss << file_path_ << "/bb_" << j << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), bba[j]);
          ss.str("");
          ss.clear();
          */
        }
      }
      ROS_INFO("BB took %f seconds", sw.precisionStop());
      ROS_INFO("Computed %d bounding boxes", (int)object_clusters.size());
    }
  }


  /**
   * @brief detects objects on table and recognizes them
   *
   * DEPRECATED
   *
   * @param goal unused
   *
   * @return nothing
   */
  /*void
  actionCallback(const cob_3d_mapping_msgs::TableObjectClusterGoalConstPtr &goal)
  {
    //boost::lock_guard<boost::mutex> guard(mutex_);
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

    //std::vector<pcl::PointCloud<pcl::PointXYZ> > bba;
    cob_object_detection_msgs::DetectionArray bba;
    cob_perception_msgs::PointCloud2Array pca;
    compute(bba, pca);

    bba_pub_.publish(bba);
    publishMarker(bba);
    object_cluster_pub_.publish(pca);
    result.bounding_boxes = bba;
    as_->setSucceeded(result);
  }*/

  void
  topicCallback(const PointCloud::ConstPtr& pc, const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa)
  {
    //boost::lock_guard<boost::mutex> guard(mutex_);

    *last_pc_ = *pc; // deep copy, required for removing points later on
    last_sa_ = sa; // copy shape array pointer

    ROS_INFO("Saved new PointCloud and %d potential tables", (int)sa->shapes.size());

    if(enable_action_mode_) { return; }

    cob_object_detection_msgs::DetectionArray bba;
    cob_perception_msgs::PointCloud2Array pca;
    compute(bba, pca);
    publishMarker(bba);
    bba_pub_.publish(bba);
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
  publishMarker(cob_object_detection_msgs::DetectionArray& bba)
  {
    for(unsigned int i=0; i<bba.detections.size(); i++)
    {
      visualization_msgs::Marker marker;
      cob_perception_common::boundingBoxToMarker(bba.detections[i], marker);

      marker.lifetime = ros::Duration();
      marker.header.frame_id = bba.header.frame_id;
      marker.id = i;

      marker_pub_.publish(marker);
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
      marker_pub_.publish(marker);
    }
  }


  ros::NodeHandle n_;


protected:
  actionlib::SimpleActionServer<cob_3d_mapping_msgs::TableObjectClusterAction>* as_;
  //ros::ServiceClient get_bb_client_;
	ros::ServiceServer set_known_objects_server_;
  ros::Publisher bba_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher object_cluster_pub_;
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_;
  message_filters::Subscriber<PointCloud> pc_sub_;
  message_filters::Subscriber<cob_3d_mapping_msgs::ShapeArray> sa_sub_;
  dynamic_reconfigure::Server<table_object_cluster_nodeletConfig> config_server_;

  boost::mutex mutex_;

  TableObjectCluster<Point> toc;       /// class for actual calculation
  unsigned int ctr_;

  bool save_to_file_;
  std::string file_path_;
  /*double height_min_;           /// paramter for object detection
  double height_max_;           /// paramter for object detection
  int min_cluster_size_;        /// paramter for object detection
  double cluster_tolerance_;    /// paramter for object detection*/
  bool enable_action_mode_;

  PointCloud::Ptr last_pc_;
  cob_3d_mapping_msgs::ShapeArray::ConstPtr last_sa_;
	std::vector<BoundingBox> known_objects_;
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

