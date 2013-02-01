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
 *  ROS stack name: cob_environment_perception_intern
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
//#include <rosbag/bag.h>
//#include <tf_conversions/tf_eigen.h>
//#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

// ros message includes
//#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL includes
/*#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>*/

//internal includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
//#include <cob_3d_mapping_msgs/MoveToTable.h>
#include <cob_3d_mapping_semantics/structure_extraction.h>
#include <cob_3d_mapping_semantics/structure_extraction_nodeConfig.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
//#include <tabletop_object_detector/TabletopDetection.h>

using namespace cob_3d_mapping;

class StructureExtractionNode
{
public:

  // Constructor
  StructureExtractionNode () :
    target_frame_id_ ("/map")
  {
    config_server_.setCallback(boost::bind(&StructureExtractionNode::dynReconfCallback, this, _1, _2));

    sa_sub_ = n_.subscribe ("shape_array", 10, &StructureExtractionNode::callbackShapeArray, this);
    sa_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray> ("shape_array_pub", 1); //10
    s_marker_pub_ = n_.advertise<visualization_msgs::Marker> ("marker", 10);
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
      //if(sa_ptr->header.frame_id!="/map")
      //  poly_ptr->transform2tf(af_target);
      //ROS_INFO("\n\tisTableObject....  : ");
      se_.setInputPolygon(poly_ptr);
      unsigned int label;
      se_.classify(label);
      switch (label)
      {
        case 1:
          poly_ptr->color[0] = 1;
          poly_ptr->color[1] = 1;
          poly_ptr->color[2] = 1;
          poly_ptr->color[3] = 0.5;
        case 2:
          poly_ptr->color[0] = 0.5;
          poly_ptr->color[1] = 0.5;
          poly_ptr->color[2] = 0.5;
          poly_ptr->color[3] = 1;
        case 3:
          poly_ptr->color[0] = 0;
          poly_ptr->color[1] = 0;
          poly_ptr->color[2] = 1;
          poly_ptr->color[3] = 1;
      }
      cob_3d_mapping_msgs::Shape s;
      s.header = sa_ptr->header;
      s.header.frame_id = target_frame_id_;
      toROSMsg(*poly_ptr,s);
      if(label == 2 && remove_floor_)
        std::cout << "removing floor" << std::endl;
      else
        sa_out.shapes.push_back (s);
    }
    //publishShapeMarker (tables);
    sa_pub_.publish (sa_out);
  }

  /**
   * @brief publishe markers to visualize shape in rviz
   *
   * @param s shape to be seen visually
   *
   * @return nothing
   */
  void
  publishShapeMarker (const cob_3d_mapping_msgs::ShapeArray& sa)
  {
    /*for(unsigned int i=0; i<table_ctr_old_; i++)
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.id = i;
      s_marker_pub_.publish (marker);
    }
    for(unsigned int i=0; i<sa_ptr->shapes.size(); i++)
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.lifetime = ros::Duration ();
      marker.header.frame_id = target_frame_id_;
      marker.ns = "table_marker";
      marker.header.stamp = ros::Time::now ();

      marker.id = i;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0;
      marker.color.r = 0;//1;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 1.0;


      // sensor_msgs::PointCloud2 pc2;
      pcl::PointCloud<pcl::PointXYZ> cloud;

      pcl::fromROSMsg (sa_ptr->shapes[i].points[0], cloud);

      geometry_msgs::Point p;
      //marker.points.resize (cloud.size()+1);
      for (unsigned int j = 0; j < cloud.size(); j++)
      {
        p.x = cloud[j].x;
        p.y = cloud[j].y;
        p.z = cloud[j].z;
        marker.points.push_back(p);
      }
      p.x = cloud[0].x;
      p.y = cloud[0].y;
      p.z = cloud[0].z;
      marker.points.push_back(p);
      s_marker_pub_.publish (marker);
    }*/
  }


  ros::NodeHandle n_;

protected:
  ros::Subscriber sa_sub_;
  ros::Publisher sa_pub_;
  ros::Publisher s_marker_pub_;

  /**
  * @brief Dynamic Reconfigure server
  */
  dynamic_reconfigure::Server<cob_3d_mapping_semantics::structure_extraction_nodeConfig> config_server_;

  StructureExtraction se_;

  std::string target_frame_id_;
  bool remove_floor_;

};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_extraction_node");

  StructureExtractionNode sem_exn_node;
  //ros::spin ();

  ros::Rate loop_rate (10);
  while (ros::ok ())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}
