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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2011
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
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/point_types.h>



// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model/GetPlane.h>

// external includes
#include <boost/timer.hpp>
#include <Eigen/StdVector>

#include "cob_env_model/table_object_cluster.h"
#include "cob_env_model/TableObjectClusterAction.h"


//####################
//#### nodelet class ####
class TableObjectClusterNodelet : public pcl_ros::PCLNodelet
{
public:
  typedef pcl::PointXYZ Point;
  // Constructor
  TableObjectClusterNodelet()
  : as_(0)
  {
  }

  // Destructor
  ~TableObjectClusterNodelet()
  {
    /// void
    if(as_) delete as_;
  }


  void onInit()
  {
    PCLNodelet::onInit();
    n_ = getNodeHandle();

    get_plane_client_ = n_.serviceClient<cob_env_model::GetPlane>("get_plane");
    as_= new actionlib::SimpleActionServer<cob_env_model::TableObjectClusterAction>(n_, "table_object_cluster", boost::bind(&TableObjectClusterNodelet::actionCallback, this, _1), false);
    as_->start();
  }


  void
  actionCallback(const cob_env_model::TableObjectClusterGoalConstPtr &goal)
  {
    ROS_INFO("action callback");
    cob_env_model::GetPlane srv;
    if(!get_plane_client_.call(srv))
    {
      ROS_ERROR("Failed to call service get_plane");
      as_->setAborted();
      return;
    }
    pcl::PointCloud<Point>::Ptr pc(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr hull(new pcl::PointCloud<Point>);
    ROS_INFO("Hull size: %d", srv.response.hull.width*srv.response.hull.height);
    pcl::fromROSMsg(srv.response.pc, *pc);
    pcl::fromROSMsg(srv.response.hull, *hull);
    ROS_INFO("Hull size: %d", hull->size());
    pcl::io::savePCDFileASCII ("/home/goa/tmp/pc.pcd", *pc);
    pcl::io::savePCDFileASCII ("/home/goa/tmp/hull.pcd", *hull);

    pcl::PointCloud<Point>::Ptr pc_roi(new pcl::PointCloud<Point>);
    toc.extractTableRoi(pc, hull, *pc_roi);
    pcl::io::savePCDFileASCII ("/home/goa/tmp/table_roi.pcd", *pc_roi);
    std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > > known_objs;
    pcl::PointCloud<Point> obj;
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
    pcl::PointCloud<Point>::Ptr pc_roi_red(new pcl::PointCloud<Point>);
    toc.removeKnownObjects(pc_roi, known_objs, *pc_roi_red);
    pcl::io::savePCDFileASCII ("/home/goa/tmp/table_roi_red.pcd", *pc_roi_red);
    std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > > bounding_boxes;
    toc.calculateBoundingBoxes(pc_roi_red,bounding_boxes);
    for(unsigned int i=0; i< bounding_boxes.size(); i++)
    {
      std::stringstream ss;
      ss << "/home/goa/tmp/bb_" << i << ".pcd";
      pcl::io::savePCDFileASCII (ss.str(), bounding_boxes[i]);
      sensor_msgs::PointCloud2 bb;
      pcl::toROSMsg(bounding_boxes[i], bb);
      result_.bounding_boxes.push_back(bb);
    }
    /*if(!lock)
    //if(!lock.owns_lock())
    {
      ROS_INFO(" actionCallback not owning lock");
      return;
    }
    else
      ROS_INFO(" actionCallback owns lock");*/
    /*feedback_.currentStep.data = std::string("plane extraction");
    std::vector<pcl::PointCloud<Point> > v_cloud_hull;
    std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
    std::vector<pcl::ModelCoefficients> v_coefficients_plane;
    extractPlane(pc_cur_.makeShared(), v_cloud_hull, v_hull_polygons, v_coefficients_plane);
    pcl::copyPointCloud(pc_cur_, pc_plane_);
    // only save dominant plane
    pcl::copyPointCloud(v_cloud_hull[0], hull_);*/
    as_->setSucceeded(result_);
  }


  ros::NodeHandle n_;


protected:
  actionlib::SimpleActionServer<cob_env_model::TableObjectClusterAction>* as_;
  ros::ServiceClient get_plane_client_;
  ros::ServiceClient get_bb_client_;
  // create messages that are used to published feedback/result
  cob_env_model::TableObjectClusterFeedback feedback_;
  cob_env_model::TableObjectClusterResult result_;
  boost::mutex mutex_;

  TableObjectCluster toc;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, TableObjectClusterNodelet, TableObjectClusterNodelet, nodelet::Nodelet)

/// Old code

/*void publishMarker(pcl::PointCloud<Point>& cloud_hull, std::string& frame_id)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.lifetime = ros::Duration();
  marker.header.frame_id = frame_id;
  marker.id = ctr_;


  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  geometry_msgs::Point pt1, pt2, pt3;
  pt1.x = cloud_hull.points[0].x;
  pt1.y = cloud_hull.points[0].y;
  pt1.z = cloud_hull.points[0].z;

  for(unsigned int i = 1; i < cloud_hull.points.size()-1; ++i)
  {
    pt2.x = cloud_hull.points[i].x;
    pt2.y = cloud_hull.points[i].y;
    pt2.z = cloud_hull.points[i].z;

    pt3.x = cloud_hull.points[i+1].x;
    pt3.y = cloud_hull.points[i+1].y;
    pt3.z = cloud_hull.points[i+1].z;

    marker.points.push_back(pt1);
    marker.points.push_back(pt2);
    marker.points.push_back(pt3);
  }

  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1.0;

  table_marker_pub_.publish(marker);
}*/
