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



