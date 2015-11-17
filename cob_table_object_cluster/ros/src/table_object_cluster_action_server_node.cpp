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
 *  ROS package name: cob_table_object_clsuter
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 04/2013
 *
 * \brief
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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <std_srvs/Trigger.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <cob_3d_mapping_msgs/SetBoundingBoxes.h>
#include <cob_3d_mapping_msgs/TableObjectClusterAction.h>

class TableObjectClusterActionServerNode
{
public:
  typedef cob_object_detection_msgs::DetectionArray BoundingBoxes;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  TableObjectClusterActionServerNode(const std::string& name)
    : action_name(name)
    , action_started(false)
    , as(nh, name, false)
    , subscribed_(false)
  { onInit(); }

  void onInit()
  {
    //sub_pc = nh.subscribe<PointCloud>("point_cloud", 1, boost::bind(&TableObjectClusterActionServerNode::inputCallback,this,_1));
    sub_bba = nh.subscribe<BoundingBoxes>("bounding_box_array", 1,
                                          boost::bind(&TableObjectClusterActionServerNode::outputCallback,this,_1));

    pub_pc = nh.advertise<PointCloud>("triggered_point_cloud", 1);

    set_bb_client_ = nh.serviceClient<cob_3d_mapping_msgs::SetBoundingBoxes>("set_known_objects");
    start_pc_sub_ = nh.advertiseService("start_pc_sub", &TableObjectClusterActionServerNode::startPCSub, this);

    as.registerGoalCallback(boost::bind(&TableObjectClusterActionServerNode::goalCallback, this));
    as.registerPreemptCallback(boost::bind(&TableObjectClusterActionServerNode::preemptCallback, this));
    as.start();
  }

  bool startPCSub(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    if(!subscribed_)
    {
      sub_pc = nh.subscribe<PointCloud>("point_cloud", 1, boost::bind(&TableObjectClusterActionServerNode::inputCallback,this,_1));
      subscribed_ = true;
    }
    else
    {
      sub_pc.shutdown();
      subscribed_ = false;
    }
    return true;
  }

  void goalCallback()
  {
    if(!last_pc)
    {
      ROS_INFO("%s: Aborted! No point cloud available for table detection.", action_name.c_str());
      as.setAborted();
      return;
    }
    action_timeout = ros::Time::now() + ros::Duration(5.0);
    action_started = true;
    cob_3d_mapping_msgs::SetBoundingBoxes srv;
    srv.request.bounding_boxes = as.acceptNewGoal()->known_objects;
    if(set_bb_client_.call(srv))
    {
      pub_pc.publish(last_pc);
    }
    else
    {
      ROS_WARN("Failed to set bounding boxes of known objects");
    }
  }

  void preemptCallback()
  {
    ROS_INFO("%s: Preempted", action_name.c_str());
    as.setPreempted();
  }

  // buffer last camera point cloud
  void inputCallback(const PointCloud::ConstPtr& pc)
  {
    //ROS_INFO("PC received");
    last_pc = pc;
  }

  // wait for incomming bouning boxes and redirect
  void outputCallback(const BoundingBoxes::ConstPtr& bba)
  {
    if(as.isActive())
    {
      cob_3d_mapping_msgs::TableObjectClusterResult result;
      result.bounding_boxes = *bba;
      as.setSucceeded(result);
    }
  }

  // check for timeout
  void spinOnce()
  {
    if( as.isActive() && ros::Time::now() > action_timeout )
    {
      ROS_INFO("%s: Timeout! No BoundingBoxes received after 5.0s", action_name.c_str());
      as.setAborted();
      action_started = false;
    }
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub_pc;
  ros::Subscriber sub_bba;
  ros::Publisher pub_pc;
  ros::ServiceClient set_bb_client_;
  ros::ServiceServer start_pc_sub_;

  actionlib::SimpleActionServer<cob_3d_mapping_msgs::TableObjectClusterAction> as;

  PointCloud::ConstPtr last_pc;

  std::string action_name;
  bool action_started;
  ros::Time action_timeout;
  bool subscribed_;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "table_object_cluster_action_server");

  TableObjectClusterActionServerNode tocasn("tabletop_object_cluster_trigger");

  ros::Rate loop_rate(10); // in hz
  while (ros::ok())
  {
    ros::spinOnce();
    tocasn.spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
