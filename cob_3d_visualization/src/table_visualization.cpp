/*
 * table_visualization_node.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: goa-sn
 */
// ROS includes
#include <ros/ros.h>

#include <cob_3d_visualization/table_visualization.h>
#include <cob_3d_mapping_msgs/GetObjectsOfClass.h>
#include <cob_3d_mapping_msgs/GetGeometricMap.h>


void TableVisualization::tableVisualizationCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& ta) {

  cob_3d_mapping_msgs::GetObjectsOfClass::Request req;
  cob_3d_mapping_msgs::GetObjectsOfClass::Response res;
  cob_3d_mapping_msgs::ShapeArrayPtr tables ;

  if (ros::service::call("/table_extraction/get_objects_of_class",req,res)){
    ROS_INFO("Service called...");
  }

  //  cob_3d_mapping_msgs::GetGeometricMapRequest req;
  //  cob_3d_mapping_msgs::GetGeometricMapResponse res;

  std::cout << "response size: "<< "\t" << res.objects.shapes.size() << "\n" ;
  if (!res.objects.shapes.empty()){
  for (unsigned int i=0;i<res.objects.shapes.size();i++){
    ROS_WARN("table id : %d", i) ;//res.objects.shapes[i].id) ;

//    tables->shapes[i] = res.objects.shapes[i] ;
    boost::shared_ptr<TableMarker> tm(new TableMarker(table_im_server_,res.objects.shapes[i],i)) ;//,ctr_));
    v_tm_.push_back(tm) ;
//    ctr_ ++ ;
  }
  }
}


  int main (int argc, char** argv){
    ros::init (argc, argv, "table_visualization");
    ROS_INFO("table_visualization node started....");
    TableVisualization tablevis;
    ros::spin();
  }



