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
 * Author: Shaghayegh Nazari, email:georg.arbeiter@ipa.fhg.de
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


#ifndef TABLE_MARKER_H_
#define TABLE_MARKER_H_

//##################
//#### includes ####
// standard includes
#include <stdio.h>
#include <sstream>

// ROS includes
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
//#include <visualization_msgs/MenuEntry.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// PCL includes
#include <pcl/pcl_config.h>
#ifdef PCL_VERSION_COMPARE
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>

// external includes
#include <Eigen/Core>

//#include <cob_3d_visualization/polypartition.h>
//#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_visualization/shape_marker.h>
//#include <cob_3d_mapping_msgs/ModifyMap.h>

#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"
#include <cob_3d_mapping_msgs/MoveToTable.h>





class TableMarker
{
  public:
    // Constructor
    TableMarker (boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,cob_3d_mapping_msgs::Shape& table,int ctr,
        tabletop_object_detector::Table& tableMsg) ;
    // Destructor
    ~TableMarker ()
    {

    }
    /**
     * @brief Create marker for the table and add it to the interactive marker control
     *
     * @param[in] triangle_list triangulated list of poly points
     * @param[in] im_ctrl interactive marker control
     *
     */
    void createMarkerforTable (visualization_msgs::InteractiveMarkerControl& im_ctrl);
    /**
     * @brief Publish interactive markers for a table detected by table_extractiion node using interactive marker server
     */
    void createInteractiveMarkerForTable();
    /**
     * @brief Create menu entries for each shape
     * @param[in] point 3D point to be transformed
     * @param[out] transformed 2D TPPLPoint
     * @return return transformed 2D TPPLPoint
     */
    TPPLPoint msgToPoint2DforTable (const pcl::PointXYZ &point);
    /**
     * @brief callback function when there is a mouse click on interactive marker
     * @param[in] feedback feedback from rviz when there is a mouse click on interactive marker
     */
    void tableFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    /**
     * @brief Create menu entries for each table
     */
    void createTableMenu();
    /**
     * @brief feedback callback for Move to this table menu entry
     * @param[in] feedback feedback from rviz when Move to this table menu entry of a detected table is chosen
     */
    void MoveToTheTable(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  protected:
    visualization_msgs::InteractiveMarker table_int_marker_ ;
    visualization_msgs::Marker table_marker_;

    ros::NodeHandle nh_;

    cob_3d_mapping_msgs::Shape table_;

    ros::Publisher goal_pub_ ;


    visualization_msgs::InteractiveMarkerControl im_ctrl;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> table_im_server_;
    interactive_markers::MenuHandler table_menu_handler_;

    Eigen::Affine3f transformation_;
    Eigen::Affine3f transformation_inv_;

    int id_;

    // Table Parameters
    //tabletop_object_detector::Table table_msg_;

};


#endif /* TABLE_MARKER_H_ */
