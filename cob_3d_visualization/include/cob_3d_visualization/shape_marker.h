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
 * shape_marker.h
 *
 *  Created on: Jul 25, 2012
 *      Author: goa-sn
 */

#ifndef SHAPE_MARKER_H_
#define SHAPE_MARKER_H_


//##################
//#### includes ####
// standard includes
#include <stdio.h>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
// ROS includes
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
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

//#include <boost/bind.hpp>
//#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

// external includes
#include <Eigen/Core>

//#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_visualization/polypartition.h>
#include <cob_3d_mapping_msgs/ModifyMap.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"
//#include <cob_3d_visualization/shape_visualization.h>

//#define PI 3.14159265

//using namespace cob_3d_mapping ;
class ShapeMarker
{
  public:

    ShapeMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server,
        cob_3d_mapping_msgs::Shape& shape,std::vector<unsigned int>& moved_shapes_indices,std::vector<unsigned int>& interacted_shapes,
        std::vector<unsigned int>& deleted_markers_indices_) :
          interacted_shapes_(interacted_shapes) , moved_shapes_indices_(moved_shapes_indices) , deleted_markers_indices_(deleted_markers_indices_)
    {

      im_server_ = im_server;
      shape_ = shape;
      id_ = shape.id;
//      feedback_sub_ = nh_.subscribe("geometry_map/map/feedback",1,&ShapeMarker::setShapePosition,this);
      createShapeMenu ();
      createInteractiveMarker();

    }

    ~ShapeMarker()
    {
      if(im_server_->erase(marker_.name)){
        //        ROS_INFO("Marker %s erased",marker_.name.c_str());
        stringstream ss;
        ss << "normal_" << shape_.id;
        im_server_->erase(ss.str());
        ss.str("");
        ss.clear();
        ss << "centroid_" << shape_.id;
        im_server_->erase(ss.str());
      }
    }


    void enableMovement (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void displayArrows() ;
    void hideArrows(int flag_untick) ;

    void createShapeMenu () ;
    void deleteMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;

    TPPLPoint msgToPoint2D (const pcl::PointXYZ &point) ;

    void createMarker (list<TPPLPoly>& triangle_list,visualization_msgs::InteractiveMarkerControl& im_ctrl);
    void createInteractiveMarker () ;

    void displayNormalCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void displayNormal();
    void hideNormal(int flag_untick);

    void displayCentroidCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void displayCentroid();
    void hideCentroid(int flag_untick);

    void displayContourCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void displayContour();
    void hideContour(int flag_untick);

    void resetMarker();//bool call_reset_marker,visualization_msgs::InteractiveMarker& imarker) ;

//    void setShapePosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);//,const cob_3d_mapping_msgs::Shape& shape) ;
    void getShape (cob_3d_mapping_msgs::Shape& shape) ;
    unsigned int getID() ;

    //    std::vector<int> getInteractedShapesNumber();


  protected:
    visualization_msgs::InteractiveMarker marker_ ;
    visualization_msgs::InteractiveMarker Imarker ;
    visualization_msgs::Marker marker;

    ros::NodeHandle nh_;
    ros::Subscriber feedback_sub_ ;

    visualization_msgs::InteractiveMarkerControl im_ctrl;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
    cob_3d_mapping_msgs::Shape shape_;


    interactive_markers::MenuHandler menu_handler_;

    Eigen::Affine3f transformation_;
    Eigen::Affine3f transformation_inv_;
    //    int shape_ctr_ ;


    //    int shape_ctr_ ;
    unsigned int id_;
    std::vector<unsigned int>& moved_shapes_indices_ ;
    std::vector<unsigned int>& interacted_shapes_ ;
    std::vector<unsigned int>& deleted_markers_indices_ ;


//    bool arrows_;
//    cob_3d_mapping_msgs::ModifyMap::Request req ;
//    cob_3d_mapping_msgs::ModifyMap::Response res;
//    //
//    Eigen::Quaternionf quatInit ;
//    Eigen::Vector3f oldCentroid ;
//    Eigen::Matrix4f transInit;
//    Eigen::Affine3f affineInit;
//    Eigen::Matrix4f transInitInv;

};




#endif /* SHAPE_MARKER_H_ */
