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
 *  ROS package name: cob_3d_mapping_semantics
 *
 * \author
 *  Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2011
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

//##################
//#### includes ####
#include <algorithm>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <dynamic_reconfigure/server.h>

// ros message includes
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

//internal includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/GetGeometryMap.h>
#include <cob_3d_mapping_msgs/GetTables.h>
#include <cob_3d_mapping_semantics/table_extraction.h>
#include <cob_3d_mapping_semantics/table_extraction_nodeConfig.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>

using namespace cob_3d_mapping;

class TableExtractionNode
{
public:

  // Constructor
  TableExtractionNode () :
    target_frame_id_ ("/map")
  {
    config_server_.setCallback(boost::bind(&TableExtractionNode::dynReconfCallback, this, _1, _2));

    sa_sub_ = n_.subscribe ("shape_array", 10, &TableExtractionNode::callbackShapeArray, this);
    sa_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray> ("shape_array_pub", 1);

    get_tables_server_ = n_.advertiseService ("get_tables", &TableExtractionNode::getTablesService, this);
  }

  // Destructor
  ~TableExtractionNode ()
  {
    /// void
  }

  /**
   * @brief callback for dynamic reconfigure
   *
   * everytime the dynamic reconfiguration changes this function will be called
   *
   * @param config data of configuration
   * @param level bit descriptor which notifies which parameter changed
   *
   * @return nothing
   */
  void
  dynReconfCallback(cob_3d_mapping_semantics::table_extraction_nodeConfig &config, uint32_t level)
  {
    ROS_INFO("[table_extraction]: received new parameters");
    target_frame_id_ = config.target_frame_id;
    te_.setNormalBounds (config.tilt_angle);
    te_.setHeightMin (config.height_min);
    te_.setHeightMax (config.height_max);
    te_.setAreaMin (config.area_min);
    te_.setAreaMax (config.area_max);
  }

  /**
   * @brief callback for ShapeArray messages
   *
   * @param sa_ptr pointer to the message
   *
   * @return nothing
   */

  void
  callbackShapeArray (const cob_3d_mapping_msgs::ShapeArray::ConstPtr sa_ptr)
  {
    if(sa_ptr->header.frame_id != target_frame_id_)
    {
      ROS_ERROR("Frame IDs do not match, aborting...");
      return;
    }
    cob_3d_mapping_msgs::ShapeArray tables;
    tables.header = sa_ptr->header;
    tables.header.frame_id = target_frame_id_ ;
    processMap(*sa_ptr, tables);
    sa_pub_.publish (tables);
    ROS_INFO("Found %u tables", (unsigned int)tables.shapes.size());
  }


  /**
   * @brief service offering table object candidates
   *
   * @param req request for objects of a class (table objects in this case)
   * @param res response of the service which is possible table object candidates
   *
   * @return true if service successful
   */
  bool
  getTablesService (cob_3d_mapping_msgs::GetTablesRequest &req,
                    cob_3d_mapping_msgs::GetTablesResponse &res)
  {
    ROS_INFO("table detection started...");

    cob_3d_mapping_msgs::ShapeArray sa;
    if (getMapService (sa))
    {
    	if(sa.header.frame_id != target_frame_id_)
    	{
      		ROS_ERROR("Frame IDs do not match, aborting...");
      		return false;
    	}
      processMap(sa, res.tables);
      res.tables.header = sa.header;
      ROS_INFO("Found %zu tables", res.tables.shapes.size());
      return true;
    }
    else
      return false;
  }

  /**
   * @brief service offering geometry map of the scene
   *
   * @return true if service successful
   */
  bool
  getMapService (cob_3d_mapping_msgs::ShapeArray& sa)
  {
    ROS_INFO("Waiting for service server to start.");
    ros::service::waitForService ("get_geometry_map", 10); //will wait for infinite time

    ROS_INFO("Server started, polling map.");

    //build message
    cob_3d_mapping_msgs::GetGeometryMapRequest req;
    cob_3d_mapping_msgs::GetGeometryMapResponse res;

    if (ros::service::call ("get_geometry_map", req, res))
    {
      ROS_INFO("Service call finished.");
    }
    else
    {
      ROS_INFO("Service call failed.");
      return 0;
    }
    sa = res.map;
    return true;
  }

  /**
   * @brief processes a shape array in order to find tables
   *
   * @param sa input shape array
   * @param tables output shape array containing tables
   *
   * @return nothing
   */
  void
  processMap(const cob_3d_mapping_msgs::ShapeArray& sa, cob_3d_mapping_msgs::ShapeArray& tables)
  {
    for (unsigned int i = 0; i < sa.shapes.size (); i++)
    {
      Polygon::Ptr poly_ptr (new Polygon());
      fromROSMsg(sa.shapes[i], *poly_ptr);
      te_.setInputPolygon(poly_ptr);
      if (te_.isTable())
      {
        poly_ptr->color_[0] = 1;
        poly_ptr->color_[1] = 0;
        poly_ptr->color_[2] = 0;
        poly_ptr->color_[3] = 1;
        cob_3d_mapping_msgs::Shape s;
        s.header = sa.header;
        s.header.frame_id = target_frame_id_;
        toROSMsg(*poly_ptr,s);
        ROS_INFO("getTablesService: Polygon[%d] converted to shape",i);
        tables.shapes.push_back (s);
      }
    }
  }

  ros::NodeHandle n_;

protected:
  ros::Subscriber sa_sub_;
  ros::Publisher sa_pub_;
  ros::ServiceServer get_tables_server_;

  /**
  * @brief Dynamic Reconfigure server
  */
  dynamic_reconfigure::Server<cob_3d_mapping_semantics::table_extraction_nodeConfig> config_server_;

  TableExtraction te_;

  std::string target_frame_id_;
};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "table_extraction_node");

  TableExtractionNode sem_exn_node;
  ros::spin ();
}
