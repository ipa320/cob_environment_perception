/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 *\author
 * Author: Shaghayegh Nazari, email:georg.arbeiter@ipa.fhg.de
 * \author
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 09/2012
 * ToDo:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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



