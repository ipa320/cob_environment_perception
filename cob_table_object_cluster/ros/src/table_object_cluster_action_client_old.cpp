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
#include <cob_3d_mapping_msgs/TableObjectClusterAction.h>
#include <cob_3d_mapping_msgs/GetObjectsOfClass.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_table_object_cluster");
  ros::NodeHandle nh;

  ros::ServiceClient get_tables_client_;
  get_tables_client_ = nh.serviceClient<cob_3d_mapping_msgs::GetObjectsOfClass> ("get_objects_of_class");
  ROS_INFO("waiting for server to start");
  ros::service::waitForService ("get_objects_of_class");
  //build message
  cob_3d_mapping_msgs::GetObjectsOfClassRequest req;
  cob_3d_mapping_msgs::GetObjectsOfClassResponse res;
  get_tables_client_.call (req, res);
  ROS_INFO("Service call finished, found %d tables", res.objects.shapes.size());

  for(unsigned int i=0; i<res.objects.shapes.size(); i++)
  {
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<cob_3d_mapping_msgs::TableObjectClusterAction> ac("table_object_cluster", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    cob_3d_mapping_msgs::TableObjectClusterGoal goal;
    goal.table_hull = res.objects.shapes[i].points[0];
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
      return 0;
    }
    cob_3d_mapping_msgs::TableObjectClusterResultConstPtr res = ac.getResult();
    for( unsigned int j=0; j < res->bounding_boxes.size(); j++)
    {
      pcl::PointCloud<pcl::PointXYZ> pc;
      pcl::fromROSMsg(res->bounding_boxes[j], pc);
      std::stringstream ss;
      ss << "/tmp/cluster_" << i << "_" << j << ".pcd";
      pcl::io::savePCDFile(ss.str(),pc, false);
    }
  }
  //exit
  return 0;
}

