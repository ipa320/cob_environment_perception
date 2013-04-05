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


class MoveToTableNode
{
  public:
    // Constructor
    MoveToTableNode() ;
    // Destructor
    ~MoveToTableNode()
    {
      /// void
    }
    /**
     * @brief service callback for MoveToTable service
     * @param[in] req request  to move to table
     * @param[in] res empty response
     *
     * @return nothing
     */
    bool moveToTableService (cob_3d_mapping_msgs::MoveToTable::Request &req,cob_3d_mapping_msgs::MoveToTable::Response &res) ;
    /**
     * @brief transforms a point to table coordinate system
     *
     * @param[in] table table msg
     * @param[in] pose the pose which needs to be transformed ro table coordinate system
     * @param[out] transformed point
     *
     * @return the transformed point
     */
    geometry_msgs::Pose transformToTableCoordinateSystem(cob_3d_mapping_msgs::Table &table,geometry_msgs::Pose &Pose);
    /**
     * @brief finds whether there is an intersection between the line through the robot pose and table centroid and the boundies of the table
     * @param[in] line the line which needs to be checked for the intersection
     * @return true if there exists an Intersection
     */
    bool doIntersect(float line) ;
    /**
     * @brief finds the intersection between the line through the robot pose and table centroid and the boundies of the table
     * @param[out] intersection position
     *
     * @return the intersection point
     */
    geometry_msgs::Pose findIntersectionPoint() ;
    /**
     * @brief finds a safe position in the vicinity of the table as the target
     * @param[out] safe target position
     *
     * @return the position of the target point
     */
    geometry_msgs::Pose findSafeTargetPoint() ;
    /**
     * @brief adds a marker for showing the final target
     * @return nothing
     */
    void addMarkerForFinalPose(geometry_msgs::Pose finalPose) ;
    /**
     * @brief sets the orientation of the final target
     *
     * @param[in] finalPose target position
     * @param[out] quaternion representing the orientation of final pose
     *
     * @return the quaternion orientation
     */
    Eigen::Quaternionf faceTable (geometry_msgs::Pose finalPose);

    ros::NodeHandle n_;
    cob_3d_mapping_msgs::Table table_;
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
