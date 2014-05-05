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
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 01/2013
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
//#include <algorithm>

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ros message includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//internal includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_semantics/structure_extraction.h>
#include <cob_3d_mapping_semantics/structure_extraction_nodeConfig.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>

using namespace cob_3d_mapping;

class StructureExtractionNode
{
public:

  // Constructor
  StructureExtractionNode () :
    target_frame_id_ ("/map"),
    remove_floor_("false"),
    colorize_("false")
  {
    config_server_.setCallback(boost::bind(&StructureExtractionNode::dynReconfCallback, this, _1, _2));

    sa_sub_ = n_.subscribe ("shape_array", 10, &StructureExtractionNode::callbackShapeArray, this);
    sa_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray> ("shape_array_pub", 1);
  }

  // Destructor
  ~StructureExtractionNode ()
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
  dynReconfCallback(cob_3d_mapping_semantics::structure_extraction_nodeConfig &config, uint32_t level)
  {
    ROS_INFO("[table_extraction]: received new parameters");
    target_frame_id_ = config.target_frame_id;
    se_.setFloorHeight (config.floor_height);
    se_.setCeilingHeight (config.ceiling_height);
    remove_floor_ = config.remove_floor;
    colorize_ = config.colorize;
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
    cob_3d_mapping_msgs::ShapeArray sa_out;
    sa_out.header = sa_ptr->header;
    sa_out.header.frame_id = target_frame_id_ ;
    for (unsigned int i = 0; i < sa_ptr->shapes.size (); i++)
    {
      Polygon::Ptr poly_ptr (new Polygon());
      fromROSMsg(sa_ptr->shapes[i], *poly_ptr);
      se_.setInputPolygon(poly_ptr);
      unsigned int label;
      se_.classify(label);
      if(colorize_)
      {
        switch (label)
        {
          case 1:
            poly_ptr->color_[0] = 1;
            poly_ptr->color_[1] = 1;
            poly_ptr->color_[2] = 1;
            poly_ptr->color_[3] = 0.5;
            break;
          case 2:
            poly_ptr->color_[0] = 0.5;
            poly_ptr->color_[1] = 0.5;
            poly_ptr->color_[2] = 0.5;
            poly_ptr->color_[3] = 1;
            break;
          case 3:
            poly_ptr->color_[0] = 0;
            poly_ptr->color_[1] = 0;
            poly_ptr->color_[2] = 1;
            poly_ptr->color_[3] = 1;
            break;
        }
      }
      cob_3d_mapping_msgs::Shape s;
      s.header = sa_ptr->header;
      s.header.frame_id = target_frame_id_;
      toROSMsg(*poly_ptr,s);
      if(label == 2 && remove_floor_)
        {}//std::cout << "removing floor" << std::endl;
      else
        sa_out.shapes.push_back (s);
    }
    sa_pub_.publish (sa_out);
  }


  ros::NodeHandle n_;

protected:
  ros::Subscriber sa_sub_;
  ros::Publisher sa_pub_;

  /**
  * @brief Dynamic Reconfigure server
  */
  dynamic_reconfigure::Server<cob_3d_mapping_semantics::structure_extraction_nodeConfig> config_server_;

  StructureExtraction se_;

  std::string target_frame_id_;
  bool remove_floor_;
  bool colorize_;

};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "structure_extraction_node");

  StructureExtractionNode sem_exn_node;
  ros::spin ();
}
