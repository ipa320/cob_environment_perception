/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_semantics
*
* \author
* Author: Shaghayegh Nazari, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 10/2012
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#ifndef MOVE_TO_TABLE_NODE_H_
#define MOVE_TO_TABLE_NODE_H_

//##################
//#### includes ####
// standard includes
#include <stdio.h>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
// ROS includes
#include <ros/ros.h>
#include <cob_3d_mapping_msgs/MoveToTable.h>
#include <cob_3d_mapping_msgs/GetTables.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/interactive_marker_server.h>
//#include <cob_script_server/ScriptAction.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigenvalues>

class MoveToTableNode
{
  public:
    // Constructor
    MoveToTableNode ()

    {
      move_to_table_server_ = n_.advertiseService ("move_to_table", &MoveToTableNode::moveToTableService, this);
      safe_dist_ = 0.7 ; // should be set to a predefined safe distance
      table_im_server_.reset (new interactive_markers::InteractiveMarkerServer ("geometry_map/map", "", false));
      navigation_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped> ("move_base_simple/goal", 1);

    }
    // Destructor
    ~MoveToTableNode ()
    {
      /// void
    }

    bool moveToTableService (cob_3d_mapping_msgs::MoveToTable::Request &req,cob_3d_mapping_msgs::MoveToTable::Response &res) ;

    geometry_msgs::Pose transformToTableCoordinateSystem(tabletop_object_detector::Table &table,geometry_msgs::Pose &Pose);
    bool doIntersect(float line) ;
    geometry_msgs::Pose findIntersectionPoint() ;
    geometry_msgs::Pose findSafeTargetPoint() ;
    void addMarkerForFinalPose(geometry_msgs::Pose finalPose) ;
    Eigen::Quaternionf faceTable (geometry_msgs::Pose finalPose);

    ros::NodeHandle n_;
    tabletop_object_detector::Table table_;
    geometry_msgs::Pose robotPoseInTableCoordinateSys_;
    geometry_msgs::Pose robotPose_;

    Eigen::Matrix4f transformToTableCoordinateSys_ ;
    float safe_dist_ ;

  protected:
    ros::ServiceServer move_to_table_server_ ;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> table_im_server_;
    ros::Publisher navigation_goal_pub_ ;

};





#endif /* MOVE_TO_TABLE_NODE_H_ */
