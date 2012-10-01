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
 * shape_visualization.h
 *
 *  Created on: Jul 25, 2012
 *      Author: goa-sn
 */

#ifndef SHAPE_VISUALIZATION_H_
#define SHAPE_VISUALIZATION_H_

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
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <cob_3d_mapping_msgs/ModifyMap.h>
#include <cob_3d_visualization/shape_marker.h>
//#include <cob_3d_visualization/table_marker.h>
#include <cob_3d_mapping_msgs/GetTables.h>
#include <cob_3d_mapping_msgs/GetObjectsOfClass.h>



#include <boost/shared_ptr.hpp>


class ShapeVisualization
{
  public:
    // Constructor
    ShapeVisualization () :ctr_for_shape_indexes (0)

    {
      shape_array_sub_ = nh_.subscribe ("shape_array", 1, &ShapeVisualization::shapeArrayCallback, this);
      feedback_sub_ = nh_.subscribe("geometry_map/map/feedback",1,&ShapeVisualization::setShapePosition,this);
      //      shape_pub_ = nh_.advertise<cob_3d_mapping_msgs::ShapeArray> ("shape_array", 1);
      //      get_table_subscriber_ = nh_.subscribe("shape_array", 1, &ShapeVisualization::findTables,this);
      im_server_.reset (new interactive_markers::InteractiveMarkerServer ("geometry_map/map", "", false));
      moreOptions() ;
      //      findTables();//const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    }
    // Destructor
    ~ShapeVisualization ()
    {
      /// void
    }

    void shapeArrayCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& sa) ;
    void setShapePosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);//,const cob_3d_mapping_msgs::Shape& shape) ;
    void moreOptions();
    void displayAllNormals(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void displayAllCentroids (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void displayAllContours (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void resetAll(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void applyModifications (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void optionMenu() ;
//    void findTables(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  protected:

    ros::NodeHandle nh_;
    //    ros::Publisher shape_pub_ ;
    ros::Subscriber shape_array_sub_; // sub for shape array msgs
    ros::Subscriber feedback_sub_ ;
    std::vector<boost::shared_ptr<ShapeMarker> > v_sm_;
    cob_3d_mapping_msgs::ShapeArray sha ;
    interactive_markers::MenuHandler menu_handler_for_text_;
    //    ros::Subscriber get_table_subscriber_;
    int ctr_for_shape_indexes;
    std::vector<unsigned int> moved_shapes_indices_;
    std::vector<unsigned int> interacted_shapes_;
    std::vector<unsigned int> deleted_markers_indices_;
    cob_3d_mapping_msgs::ShapeArray modified_shapes_;


    Eigen::Quaternionf quatInit ;
    Eigen::Vector3f oldCentroid ;
    Eigen::Matrix4f transInit;
    Eigen::Affine3f affineInit;
    Eigen::Matrix4f transInitInv;
    cob_3d_mapping_msgs::ModifyMap::Request req ;
    cob_3d_mapping_msgs::ModifyMap::Response res;



    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_; // server for interactive markers

};


#endif /* SHAPE_VISUALIZATION_H_ */
