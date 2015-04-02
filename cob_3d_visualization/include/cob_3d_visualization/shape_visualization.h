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
 *  ROS package name: cob_3d_visualization
 *
 * \author
 *  Author: Waqas Tanveer, email:Waqas.Tanveer@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 04/2012
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
//#include <cob_3d_mapping_msgs/GetTables.h>
#include <cob_3d_mapping_msgs/GetObjectsOfClass.h>
#include <tf/transform_listener.h>




#include <boost/shared_ptr.hpp>


class ShapeVisualization
{
  public:
    // Constructor
    ShapeVisualization () ;
    // Destructor
    ~ShapeVisualization ()
    {
      /// void
    }
    /**
     * @brief Callback for shape array messages
     *
     * @param sa received shape array message
     */
    void shapeArrayCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& sa) ;
    /**
     * @brief Callback for feedback subscriber for getting the transformation of moved markers
     * @param[in] feedback subscribed from geometry_map/map/feedback
     */
    void setShapePosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    /**
     * @brief creats a text for applying controls on all of the markers
     **/
    void moreOptions();
    /**
     * @brief Feedback callback for All Normals Controls menu entry
     *
     * @param feedback feedback from rviz when the All Normals menu entry of the text is changed
     */
    void displayAllNormals(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    /**
     * @brief Feedback callback for All Centroids Controls menu entry
     *
     * @param feedback feedback from rviz when the All Centroids menu entry of the text is changed
     */
    void displayAllCentroids (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Feedback callback for All Contours Controls menu entry
     *
     * @param feedback feedback from rviz when the All Contours menu entry of the text is changed
     */
    void displayAllContours (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    /**
     * @brief Feedback callback for Reset all Controls menu entry
     * @param[in] feedback feedback from rviz when the Reset all Controls menu entry of the text is changed
     */
    void resetAll(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Feedback callback for Apply map modifications menu entry
     * @param[in] feedback feedback from rviz when the Apply map modifications menu entry of the text is changed
     */
    void applyModifications (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    /**
     * @brief Create menu entries for the text
     */
    void optionMenu() ;
//    void findTables(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  protected:

    ros::NodeHandle nh_;
    //    ros::Publisher shape_pub_ ;
    ros::Subscriber shape_array_sub_; // sub for shape array msgs
    ros::Subscriber feedback_sub_ ;
    ros::Publisher marker_pub_;
    std::vector<boost::shared_ptr<ShapeMarker> > v_sm_;
    cob_3d_mapping_msgs::ShapeArray sha ;
    interactive_markers::MenuHandler menu_handler_for_text_;

    int ctr_for_shape_indexes;
    std::vector<unsigned int> moved_shapes_indices_;
    std::vector<unsigned int> interacted_shapes_;
    std::vector<unsigned int> deleted_markers_indices_;
    cob_3d_mapping_msgs::ShapeArray modified_shapes_;
    std::string frame_id_;
    std::vector<unsigned int> marker_ids_;
    std::vector<unsigned int> contour_ids_;
    bool show_contours_;
//    unsigned int deleted_ ;


    Eigen::Quaternionf quatInit ;
    Eigen::Vector3f oldCentroid ;
    Eigen::Matrix4f transInit;
    Eigen::Affine3f affineInit;
    Eigen::Matrix4f transInitInv;
//    cob_3d_mapping_msgs::ModifyMap::Request req ;
//    cob_3d_mapping_msgs::ModifyMap::Response res;



    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_; // server for interactive markers

	void on_subscriber(const ros::SingleSubscriberPublisher& pub);
};


#endif /* SHAPE_VISUALIZATION_H_ */
