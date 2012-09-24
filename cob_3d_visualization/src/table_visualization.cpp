/*
 * table_visualization_node.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: goa-sn
 */
// ROS includes
#include <ros/ros.h>

#include <cob_3d_visualization/table_visualization.h>

void TableVisualization::tableVisualizationCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& ta) {
    cob_3d_mapping_msgs::GetObjectsOfClass::Request req;
    cob_3d_mapping_msgs::GetObjectsOfClass::Response res;

    ros::service::call("/table_extraction/get_objects_of_class",req,res) ;

//  cob_3d_mapping_msgs::GetTables::Request req ;
//  cob_3d_mapping_msgs::GetTables::Response res;
//
//  ros::service::call("/table_extraction/get_tables",req,res) ;
}


int main (int argc, char** argv){
  ros::init (argc, argv, "table_visualization");
  ROS_INFO("table_visualization node started....");
  TableVisualization tablevis;
  ros::spin();
}



