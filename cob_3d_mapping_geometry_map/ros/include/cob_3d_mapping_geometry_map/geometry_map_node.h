/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
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
#include <cob_3d_mapping_msgs/GetGeometricMap.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_srvs/Trigger.h>

#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_geometry_map/geometry_map.h"

namespace cob_3d_mapping
{
  class GeometryMapNode //: protected Reconfigurable_Node<cob_3d_mapping_geometry_map::geometry_map_nodeConfig>
  {
  public:
    // Constructor
    GeometryMapNode();
    // Destructor
    ~GeometryMapNode() { };

    void dynReconfCallback(cob_3d_mapping_geometry_map::geometry_map_nodeConfig &config, uint32_t level);
    void shapeCallback(const cob_3d_mapping_msgs::ShapeArray::ConstPtr sa);

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
    bool getMap(cob_3d_mapping_msgs::GetGeometricMap::Request &req, cob_3d_mapping_msgs::GetGeometricMap::Response &res);

    /**
     * @brief output featuremap to dump file
     *
     * output featuremap to dump file, path is hard coded
     *
     * @param m feature map
     *
     * @return nothing
     */
    void dumpPolygonToFile(Polygon& m);

    void publishMap();

    /**
     * @brief publishes the contour of the polygons
     *
     * publishes the contour of the polygons
     *
     * @return nothing
     */
    void publishMapMarker();

    void fillMarker(Polygon::Ptr p, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t);
    void fillMarker(Cylinder::Ptr c, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t);
    void fillMarker(ShapeCluster::Ptr sc, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t);

    ros::NodeHandle n_;

  protected:
    ros::Subscriber shape_sub_;
    ros::Publisher map_pub_;
    ros::Publisher marker_pub_;
    ros::ServiceServer clear_map_server_;
    ros::ServiceServer get_map_server_;

    tf::TransformListener tf_listener_;
    bool enable_tf_;

    dynamic_reconfigure::Server<cob_3d_mapping_geometry_map::geometry_map_nodeConfig> config_server_;

    GeometryMap geometry_map_;      /// map containing geometrys (polygons)

    unsigned int ctr_;            /// counter how many polygons are received
    std::string file_path_;
    bool save_to_file_;
    std::string map_frame_id_;
  };
}

#endif // GEOMETRY_MAP_NODE_H__
