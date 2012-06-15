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

