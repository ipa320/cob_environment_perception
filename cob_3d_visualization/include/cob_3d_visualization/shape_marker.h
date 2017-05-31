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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_mapping_common
 *
 * \author
 *  Author: Waqas Tanveer, email:Waqas.Tanveer@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 09/2012
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
//#include <cob_3d_mapping_common/polypartition.h>
#include <cob_3d_mapping_msgs/ModifyMap.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_common/cylinder.h"
//#include <cob_3d_visualization/shape_visualization.h>

//#define PI 3.14159265

//using namespace cob_3d_mapping ;
class ShapeMarker
{
  public:

    ShapeMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server,
        cob_3d_mapping_msgs::Shape& shape,std::vector<unsigned int>& moved_shapes_indices,std::vector<unsigned int>& interacted_shapes,
        std::vector<unsigned int>& deleted_markers_indices, bool arrows,bool deleted);
    ~ShapeMarker()
    {
      if(im_server_->erase(marker_.name)){
        //        ROS_INFO("Marker %s erased",marker_.name.c_str());
        std::stringstream ss;
        ss << "normal_" << shape_.id;
        im_server_->erase(ss.str());
        ss.str("");
        ss.clear();
        ss << "centroid_" << shape_.id;
        im_server_->erase(ss.str());
      }
    }

    /**
     * @brief subdivides a list of triangles.
     *
     * Based on a threshold in x-Direction, triangles are subdivided.
     * @param[in] i_list Input triangle list.
     * @param[out] o_list Output triangle list.
     * @return nothing
     */
    void triangle_refinement(std::list<TPPLPoly>& i_list, std::list<TPPLPoly>& o_list);
    void getShape (cob_3d_mapping_msgs::Shape& shape);
    /**
     * @brief returns the global variable arrows_
     * @param[out] global variable arrows_
     */
    bool getArrows();
    /**
     * @brief sets the global variable arrows_ to false
     * @param[out] global variable arrows_
     */
    bool setArrows();
    /**
     * @brief returns global variable deleted_
     * @param[out] global variable deleted_
     */
    bool getDeleted();
    /**
     * @brief sets the global variable deleted_ to false
     * @param[out] global variable deleted_
     */
    bool setDeleted();
    /**
     * @brief returns id of the shape msg
     * @param[out] shape msg id
     */
    unsigned int getID() ;
    /**
     * @brief Feedback callback for Enable Movement menu entry
     * @param[in] feedback feedback from rviz when the Enable Movement menu entry of a shape is changed
     */
    void enableMovement (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Display arrows for interactive marker movement
     */
    void displayArrows() ;
    /**
     * @brief Remove arrows for interactive marker movement
     * @param[in] flag_untick a flag which shows whether Enable Movement is unticked
     */
    void hideArrows(int flag_untick) ;
    /**
     * @brief Create menu entries for each shape
     */
    void createShapeMenu () ;
    /**
     * @brief feedback callback for Delete Marker menu entry
     * @param[in] feedback feedback from rviz when the Delete Marker menu entry of a shape is chosen
     */
    void deleteMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Create menu entries for each shape
     *
     * @param[in] point 3D point to be transformed
     * @return returns transformed 2D TPPLPoint
     */
    TPPLPoint msgToPoint2D (const pcl::PointXYZ &point) ;
    /**
     * @brief Create marker for the shape and add it to the interactive marker control
     *
     * @param[in] triangle_list triangulated list of poly points
     * @param[in] im_ctrl interactive marker control
     *
     */
    void createMarker (visualization_msgs::InteractiveMarkerControl& im_ctrl);
    /**
     * @brief Publish interactive markers for a shape message using interactive marker server
     */
    void createInteractiveMarker () ;
    /**
     * @brief Feedback callback for normal menu entry
     *
     * @param[in] feedback feedback from rviz when the normal menu entry of a shape is changed
     */
    void displayNormalCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Display the normal vector of a shape
     */
    void displayNormal();
    /**
     * @brief Remove the normal vector of a shape
     * param[in] flag_untick a flag which shows whether Enable Movement is unticked
     */
    void hideNormal(int flag_untick);
    /**
     * @brief Feedback callback for symmetry axis menu entry with cylinders
     *
     * @param[in] feedback feedback from rviz when the symmetry axis menu entry of a shape is changed
     */
    void displaySymAxisCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Display the Symmetry axis of a cylinder
     */
    void displaySymAxis();
    /**
     * @brief Remove the Symmetry axis of a cylinder
     * param[in] flag_untick a flag which shows whether Enable Movement is unticked
     */
    void hideSymAxis(int flag_untick);
    /**
     * @brief Feedback callback for Display Centroid menu entry
     *
     * @param[in] feedback feedback from rviz when the Display Centroid menu entry of a shape is changed
     */
    void displayCentroidCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Display the centroid of a shape
     */
    void displayCentroid();
    /**
     * @brief Remove the centroid of a shape
     */
    void hideCentroid(int flag_untick);
    /**
     * @brief Feedback callback for origin  menu entry
     *
     * @param[in] feedback feedback from rviz when the centroid menu entry of a shape is changed
     */
    void displayOriginCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Display the origin of a cylinder
     */
    void displayOrigin();
    /**
     * @brief Remove the origin of a cylinder
     */
    void hideOrigin(int flag_untick);
    /**
     * @brief Feedback callback for Display Contour menu entry
     *
     * @param[in] feedback feedback from rviz when the Display Contour menu entry of a shape is changed
     */
    void displayContourCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Display contour of a shape
     */
    void displayContour();
    /**
     * @brief Remove contour of a shape
     */
    void hideContour(int flag_untick);
    /**
     * @brief Resets all controls activated for Interactive marker
     *
     */
    void resetMarker();

    void displayIDCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void displayID();
    void hideID(int flag_untick);

    visualization_msgs::Marker& getMarker() {return marker;};
    visualization_msgs::MarkerArray& getContourMarker() {return contour_marker_;};
    visualization_msgs::MarkerArray& getDeleteContourMarker() {return delete_contour_marker_;};




  protected:
    visualization_msgs::InteractiveMarker marker_ ;
    visualization_msgs::InteractiveMarker imarker_ ;
    visualization_msgs::InteractiveMarker deleted_imarker_ ;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray contour_marker_;
    visualization_msgs::MarkerArray delete_contour_marker_;

    //    ros::NodeHandle nh_;
    //    ros::Subscriber feedback_sub_ ;
    //    ros::Publisher marker_pub_ ;

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
    //    unsigned int& deleted_ ;


    bool arrows_;
    bool deleted_ ;
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
