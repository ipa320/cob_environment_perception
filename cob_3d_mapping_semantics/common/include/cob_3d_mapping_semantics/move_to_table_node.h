/*
 * move_to_table_node.h
 *
 *  Created on: Oct 9, 2012
 *      Author: goa-sn
 */

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

    geometry_msgs::Pose transformToTableCoordinateSystem(cob_3d_mapping_msgs::Table &table,geometry_msgs::Pose &Pose);
    bool doIntersect(float line) ;
    geometry_msgs::Pose findIntersectionPoint() ;
    geometry_msgs::Pose findSafeTargetPoint() ;
    void addMarkerForFinalPose(geometry_msgs::Pose finalPose) ;

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
