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
#include <cob_3d_mapping_msgs/MoveToTable.h>
#include <cob_3d_mapping_msgs/GetTables.h>


#include <cob_3d_mapping_msgs/GetGeometricMap.h>


void TableVisualization::tableVisualizationCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& ta) {

  // Service request and response
  cob_3d_mapping_msgs::GetObjectsOfClass::Request reqObject;
  cob_3d_mapping_msgs::GetObjectsOfClass::Response resObject;

  cob_3d_mapping_msgs::GetTables::Request reqTable ;
  cob_3d_mapping_msgs::GetTables::Response resTable;

  //
  if (ros::service::call("/table_extraction/get_objects_of_class",reqObject,resObject)){
    ROS_INFO("Service called...");
  }

  if (ros::service::call("/table_extraction/get_tables",reqTable,resTable)){
    ROS_INFO("Service called...");
  }

  //  std::cout << "response size: "<< "\t" << res.tables.size() << "\n" ;
  if (!resTable.tables.empty()){
    for (unsigned int i=0;i<resTable.tables.size();i++){
      ROS_WARN("table id : %d", i) ;//res.objects.shapes[i].id) ;
//      ROS_WARN("table x_min is: %f , table y_min is: %f", resTable.tables[i].table.x_min , resTable.tables[i].table.y_min);
    }
  }
  //    tables->shapes[i] = res.objects.shapes[i] ;
  //      boost::shared_ptr<TableMarker> tm(new TableMarker(table_im_server_,res.objects.shapes[i],i)) ;//,ctr_));
  //      v_tm_.push_back(tm) ;
  //    ctr_ ++ ;



  //  std::cout << "response size: "<< "\t" << res.objects.shapes.size() << "\n" ;
  if (!resObject.objects.shapes.empty()){
    for (unsigned int i=0;i<resObject.objects.shapes.size();i++){
      ROS_WARN("table id : %d", i) ;//res.objects.shapes[i].id) ;

      //    tables->shapes[i] = res.objects.shapes[i] ;
      boost::shared_ptr<TableMarker> tm(new TableMarker(table_im_server_,resObject.objects.shapes[i],i,resTable.tables[i].table)) ;//,ctr_));
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



