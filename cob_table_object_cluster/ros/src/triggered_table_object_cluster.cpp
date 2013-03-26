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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
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
/*
 * plane_extraction_action_client.cpp
 *
 *  Created on: 26.08.2011
 *      Author: goa
 */


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_3d_mapping_msgs/TriggerSegmentationAction.h>
#include <cob_perception_msgs/PointCloud2Array.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Dummy
{
public:
  void cb(const cob_perception_msgs::PointCloud2ArrayConstPtr& pca)
    {
      ROS_INFO("received clusters");
    }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "triggered_table_object_cluster");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  Dummy dummy;
  sub = nh.subscribe<cob_perception_msgs::PointCloud2Array>(
    "/tabletop_object_cluster/cluster_array", 1, boost::bind(&Dummy::cb, &dummy, _1));


  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<cob_3d_mapping_msgs::TriggerSegmentationAction> ac("trigger_segmentation", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  while(ros::ok())
  {
    cob_3d_mapping_msgs::TriggerSegmentationGoal goal;
    goal.next = true;

    ROS_INFO("Sending goal.");
    // send a goal to the action
    ac.sendGoal(goal);
    ros::spinOnce();
    sleep(1);
  }

  return 0;
}
