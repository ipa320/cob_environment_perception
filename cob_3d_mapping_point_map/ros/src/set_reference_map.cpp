/*
 * set_reference_map.cpp
 *
 *  Created on: Sep 22, 2011
 *      Author: goa-jh
 */



#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_3d_mapping_msgs/SetReferenceMap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "cob_3d_mapping_point_map/point_map.h"

int main (int argc, char **argv)
{
  if(argc<1) {
    ROS_ERROR("Please specify path to map file\nlile:\nrosrun cob_env_model set_reference_map myfile.pcd");
    return -1;
  }
  ros::init(argc, argv, "set_reference_map");

  ros::NodeHandle nh;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ros::service::waitForService("set_reference_map"); //will wait for infinite time

  ROS_INFO("Server started, sending map.");

  //build message
  cob_3d_mapping_msgs::SetReferenceMapRequest req;

  pcl::PointCloud<PointMap::Point> map;
  if(pcl::io::loadPCDFile(argv[1], map)!=0) {
    ROS_ERROR("Couldn't open pcd file. Sorry.");
    return -1;
  }

  pcl::toROSMsg(map,req.map);

  cob_3d_mapping_msgs::SetReferenceMapResponse resp;

  if (ros::service::call("set_reference_map", req,resp))
  {
    ROS_INFO("Action finished");
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

