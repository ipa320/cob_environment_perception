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
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: cob_env_model
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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

#ifndef GEOMETRY_MAP_NODE_H__
#define GEOMETRY_MAP_NODE_H__

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ROS message includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_3d_mapping_msgs/GetGeometryMap.h>
#include <cob_3d_mapping_msgs/SetGeometryMap.h>
#include <cob_3d_mapping_msgs/ModifyMap.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_srvs/Trigger.h>

#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_geometry_map/geometry_map.h"

namespace cob_3d_mapping
{
  /**
  * @brief Ros node for GeometryMap.
  * @details Node handles input Shape arrays and provides output Shape arrays.\n
  * Input topics: /tf, /shape_array\n
  * Output topics: /map_array , /geometry_marker, /primitives
  */
  class GeometryMapNode //: protected Reconfigurable_Node<cob_3d_mapping_geometry_map::geometry_map_nodeConfig>
  {
  public:
    /**
    * @brief Constructor
    */
    GeometryMapNode();
    /**
    * brief Destructor
    */
    ~GeometryMapNode() { };

    /**
    * @brief callback for dynamic reconfigure
    *
    * everytime the dynamic reconfiguration changes this function will be called
    *
    * @param inst instance of AggregatePointMap which parameters should be changed
    * @param config data of configuration
    * @param level bit descriptor which notifies which parameter changed
    *
    * @return nothing
    */
    void dynReconfCallback(cob_3d_mapping_geometry_map::geometry_map_nodeConfig &config, uint32_t level);
    void shapeCallback(const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa);

    /**
     * @brief clears map
     *
     * deletes 3d map of the environment
     *
     * @param req not needed
     * @param res not needed
     *
     * @return nothing
     */
    bool clearMap(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);



    /**
     * @brief service callback for GetGeometricMap service
     *
     * Fills the service response of the GetGeometricMap service with the current point map
     *
     * @param req request to send map
     * @param res the current geometric map
     *
     * @return nothing
     */
    bool getMap(cob_3d_mapping_msgs::GetGeometryMap::Request &req, cob_3d_mapping_msgs::GetGeometryMap::Response &res);

    /**
     * @brief service callback for SetGeometryMap service
     *
     * Sets the GeometryMap with the service request
     *
     * @param req request to set map
     * @param res service response (empty)
     *
     * @return true if successful
     */
    bool
    setMap(cob_3d_mapping_msgs::SetGeometryMap::Request &req, cob_3d_mapping_msgs::SetGeometryMap::Response &res);

   /**
    * @brief service callback for MofiyMap service
    *
    * Fills the service response of the ModifyMap service with the modified map
    *
    * @param req request to modify map
    * @param res the modified geometric map
    *
    * @return nothing
    */
    bool modifyMap(cob_3d_mapping_msgs::ModifyMap::Request &req, cob_3d_mapping_msgs::ModifyMap::Response &res) ;



    /**
    * @brief Map arrays are published.
    *
    * Shape message is generated and published to specified topic.
    */
    void publishMap();


    /**
     * @brief publishes the contour of the polygons
     *
     * publishes the contour of the polygons
     *
     * @return nothing
     */
    void publishMapMarker();


    /**
    * @brief Cylinder primitives are published
    *
    * Visualization markers of Cylinder shapes are created and published.
    */
    void publishPrimitives();

    /**
    * @brief Polygon marker is filled out.
    * @note Method is not yet implemented.
    * @param[in] p Polygon, whose marker is to be filled.
    * @param[in] m Corresponding marker.
    * @param[out] m_t Filled marker.
    */
    void fillMarker(Polygon::Ptr p, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t);


    /**
    * @brief ShapeCluster marker is filled out.
    * @note Method is not yet implemented.
    * @param[in] p Cylinder, whose marker is to be filled.
    * @param[in] m Corresponding marker.
    * @param[out] m_t Filled marker.
    */
    void fillMarker(Cylinder::Ptr c, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t);


    /**
    * @brief ShapeCluster marker is filled out.
    * @note Method is not yet implemented.
    * @param[in] p ShapeCluster, whose marker is to be filled.
    * @param[in] m Corresponding marker.
    * @param[out] m_t Filled marker.
    */
    void fillMarker(ShapeCluster::Ptr sc, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t);

    ros::NodeHandle n_;                         ///< Ros node handle.

  protected:
    ros::Subscriber shape_sub_;                 ///< Subscription to shape message to be processed.
    ros::Publisher map_pub_;                    ///< Publish Map array as shape message.
    ros::Publisher marker_pub_;                 ///< Publish Map array as visualization markers.
    ros::Publisher primitive_pub_;              ///< Publish Cylinder primitive visualization markers.
    ros::ServiceServer clear_map_server_;
    ros::ServiceServer get_map_server_;
    ros::ServiceServer set_map_server_;
    ros::ServiceServer modify_map_server_ ;


    tf::TransformListener tf_listener_;         ///< Retrieves transformations.
    bool enable_tf_;                            ///< If true transformation to target frame is performed.
    bool enable_cyl_;                           ///< If true , processing of cylinders is activated.
    bool enable_poly_;                          ///< If true , processing of polygons is activated.

    /**
    * @brief Dynamic Reconfigure server
    */
    dynamic_reconfigure::Server<cob_3d_mapping_geometry_map::geometry_map_nodeConfig> config_server_;

    GeometryMap geometry_map_;                   ///< Map containing geometrys (polygons,cylinders)

    unsigned int ctr_;                          ///< Counter how many polygons are received.
    std::string file_path_;                     ///< Path out output file.
    bool save_to_file_;                         ///< True if file output is enabled.
    std::string map_frame_id_;                  ///< Name of target frame.
  };
}

#endif // GEOMETRY_MAP_NODE_H__
