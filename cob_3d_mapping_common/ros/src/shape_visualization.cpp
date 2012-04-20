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
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer, email:Waqas.Tanveer@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 04/2012
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

//##################
//#### includes ####
// standard includes
#include <stdio.h>
//--

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
//#include <visualization_msgs/MenuEntry.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
//#include <boost/bind.hpp>
//#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <cmath>

#include <cob_3d_mapping_common/ros_msg_conversions.h>
//####################
//#### node class ####
class ShapeVisualization
{
public:
  // Constructor
  ShapeVisualization () :
      marker_id_ (0)
  {
    shape_array_sub_ = nh_.subscribe ("shape_array", 1, &ShapeVisualization::shapeArrayCallback, this);
    viz_msg_pub_ = nh_.advertise<visualization_msgs::Marker> ("marker", 10);
    viz_msg_im_pub_ = nh_.advertise<visualization_msgs::InteractiveMarker> ("interactive_marker", 1);

    shape_pub_ = nh_.advertise<cob_3d_mapping_msgs::Shape>("shape",1);
    im_server_.reset (new interactive_markers::InteractiveMarkerServer ("shape_server"));

  }

  // Destructor
  ~ShapeVisualization ()
  {
    /// void
  }

  void
  shapeArrayCallback (const cob_3d_mapping_msgs::ShapeArrayPtr &sa)
  {
    ROS_DEBUG("Shape Array Size: %d ", (int)sa->shapes.size());
    ROS_INFO_STREAM_ONCE("shape array message received");
    for (unsigned int i = 0; i < sa->shapes.size (); i++)
      publishInteractiveMarker (sa->shapes[i], sa->header); //,sa.shapes.size());

  }

  void
  menuCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    ROS_INFO(" Menu Feedback .....");
  }

  void
  imServerCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    ROS_INFO(" Server Feedback .....");
  }

  void
  createInteractiveMarker (visualization_msgs::InteractiveMarker& im,
                           visualization_msgs::InteractiveMarkerControl& im_ctrl, std_msgs::Header& header)
  {
    ROS_INFO(" creating interactive marker .....");
    im.name = "shape_visualizer";
    im.description = "shape details";
    im.header = header;
    //im.header.frame_id = header.frame_id;
    //im.header.stamp = ros::Time::now();
    std::cout << "  interactive marker HEADER:  " << im.header << std::endl;
    im_name_ = im.name;

    im_ctrl.always_visible = true;
    im_ctrl.name = "move";
    im_ctrl.description = "normal_vector";
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  }

  void
  createMarker (cob_3d_mapping::Polygon& p, visualization_msgs::InteractiveMarkerControl& im_ctrl,
                std_msgs::Header& header)
  {
    ROS_INFO(" creating markers .....");
    marker_id_ = 0;
    visualization_msgs::Marker marker;

    marker.header.frame_id = header.frame_id;
   // marker.header.stamp = ros::Time::now();
    marker.ns = "shape_marker";

    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    //marker.ns = "shape visualization";
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration ();

    //set color
    marker.color.r = p.color[0];
    marker.color.g = p.color[1];
    marker.color.b = p.color[2];
    marker.color.a = p.color[3];

    //set scale
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    for (unsigned int i = 0; i < p.contours.size (); i++)
    {
      marker.id = marker_id_++;
      marker.points.resize (p.contours[i].size ());
      for (unsigned int j = 0; j < p.contours[i].size (); j++)
      {
        marker.points[j].x = p.contours[i][j] (0);
        marker.points[j].y = p.contours[i][j] (1);
        marker.points[j].z = p.contours[i][j] (2);
      }
      im_ctrl.markers.push_back (marker);
      createMarkerMenu (marker);
      viz_msg_pub_.publish (marker);
    }
  }

  void
  createMarkerMenu (visualization_msgs::Marker& marker)
  {
    ROS_INFO(" creating marker menu .....");
    interactive_markers::MenuHandler menu_handler;
    //interactive_markers::MenuHandler::EntryHandle eh_1,eh_2,eh_3;
    interactive_markers::MenuHandler::EntryHandle eh_2,eh_3;
    eh_1 = menu_handler.insert ("first_entry", boost::bind (&ShapeVisualization::menuCB, this, _1));
    eh_2 = menu_handler.insert ("second_entry",boost::bind (&ShapeVisualization::menuCB, this, _1));
    eh_3 = menu_handler.insert (eh_2, "third_entry",boost::bind (&ShapeVisualization::menuCB, this, _1));
    menu_handler.setVisible (eh_1, true);
    menu_handler.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);
    menu_handler.setVisible (eh_2, true);
    menu_handler.setCheckState (eh_2, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler.setVisible (eh_2, true);
    menu_handler.setCheckState (eh_2, interactive_markers::MenuHandler::CHECKED);
    menu_handler.apply (*im_server_, im_name_);
    //menu_handler.
    //menu_handler.reApply(*im_server_);
    im_server_->applyChanges ();
  }

  void
  publishInteractiveMarker (const cob_3d_mapping_msgs::Shape& s, std_msgs::Header header) //,std::vector::size_type sa_size)
  {
    //im_server_->clear();
    //im_server_->applyChanges ();

    shape_pub_.publish(s);
    cob_3d_mapping::Polygon p;
    cob_3d_mapping::fromROSMsg (s, p);

    visualization_msgs::InteractiveMarker im;
    visualization_msgs::InteractiveMarkerControl im_ctrl;

    createInteractiveMarker (im, im_ctrl, header);
    createMarker (p, im_ctrl, header);
    im.controls.push_back(im_ctrl);

    im_server_->insert (im,boost::bind (&ShapeVisualization::imServerCB, this, _1),visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);

    im_server_->applyChanges ();
   // viz_msg_im_pub_.publish (im);
  }
    /*
     entry_handle_ = menu_handler_.insert("first_entry",boost::bind (&ShapeVisualization::menuCB, this, _1));
     interactive_markers::MenuHandler::EntryHandle eh_1;
     eh_1 = menu_handler_.insert(entry_handle_,"second_entry",boost::bind (&ShapeVisualization::imServerCB, this, _1));
     menu_handler_.setVisible(entry_handle_,true);
     menu_handler_.setCheckState(entry_handle_,interactive_markers::MenuHandler::UNCHECKED);
     menu_handler_.setCheckState(eh_1,interactive_markers::MenuHandler::CHECKED);
     menu_handler_.setVisible(eh_1,true);

     menu_handler_.apply(*im_server_,im.name);
     //menu_handler_.reApply(*im_server_);

     //im_server_->setCallback (im.name, boost::bind (&ShapeVisualization::imServerCB, this, _1),visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
     im_server_->applyChanges();

     viz_msg_im_pub_.publish(im);
     */
    //visualization_msgs::MarkerArray marker_arr;
    // createMarkerArray (p,marker_arr,header);
    // visualization_msgs::InteractiveMarker im;
    /*
     for(unsigned int i = 0;i<marker_arr.markers.size(); i++)
     {
     im_ctrl.markers.push_back(marker_arr.markers[i]);

     }

     im.controls.push_back(im_ctrl);
     im.scale = 0.5;
     */
    /*
     visualization_msgs::MenuEntry menu,sub_menu1,sub_menu2,sub_menu_sm1;
     menu.id = 1;
     menu.title = "menu";

     sub_menu1.id = 2;
     sub_menu1.title = "sub_menu1";
     sub_menu1.parent_id = menu.id;

     sub_menu2.id = 3;
     sub_menu2.title = "sub_menu2";
     sub_menu2.parent_id = menu.id;

     sub_menu_sm1.id = 4;
     sub_menu_sm1.title = "sub_menu_sm1";
     sub_menu_sm1.parent_id = sub_menu1.id;


     im.menu_entries.push_back(menu);
     im.menu_entries.push_back(sub_menu1);
     im.menu_entries.push_back(sub_menu2);
     im.menu_entries.push_back(sub_menu_sm1);


     im_server_->insert(im);
     entry_handle_ = menu_handler_.insert("first_entry",boost::bind (&ShapeVisualization::menuCB, this, _1));
     interactive_markers::MenuHandler::EntryHandle eh_1;
     eh_1 = menu_handler_.insert(entry_handle_,"second_entry",boost::bind (&ShapeVisualization::imServerCB, this, _1));

     menu_handler_.setVisible(entry_handle_,true);
     menu_handler_.setCheckState(entry_handle_,interactive_markers::MenuHandler::UNCHECKED);
     menu_handler_.setCheckState(eh_1,interactive_markers::MenuHandler::CHECKED);
     menu_handler_.setVisible(eh_1,true);


     menu_handler_.apply(*im_server_,im.name);
     //menu_handler_.reApply(*im_server_);

     //im_server_->setCallback (im.name, boost::bind (&ShapeVisualization::imServerCB, this, _1),visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
     im_server_->applyChanges();

     viz_msg_im_pub_.publish(im);
     */
    /*
     //std::cout << " point clouds : " << p.contours.size () << std::endl;

     for (unsigned int i = 0; i < p.contours.size (); i++)
     {


     sleep (1);
     visualization_msgs::Marker marker;
     marker.id = marker_id_++;
     marker.points.resize (p.contours[i].size ());

     marker.header = header;
     marker.ns = "shape_marker";

     marker.type = visualization_msgs::Marker::POINTS;
     //marker.ns = "shape visualization";
     marker.action = visualization_msgs::Marker::ADD;
     marker.lifetime = ros::Duration ();

     //set color
     marker.color.r = 1;
     marker.color.g = 0;
     marker.color.b = 0;
     marker.color.a = 1;

     //set color
     marker.color.r = p.color[0];
     marker.color.g = p.color[1];
     marker.color.b = p.color[2];
     marker.color.a = p.color[3];

     //set scale
     marker.scale.x = 0.1;
     marker.scale.y = 0.1;
     marker.scale.z = 0.1;

     for (unsigned int j = 0; j < p.contours[i].size (); j++)
     {
     marker.points[j].x = p.contours[i][j] (0);
     marker.points[j].y = p.contours[i][j] (1);
     marker.points[j].z = p.contours[i][j] (2);
     }
     viz_msg_pub_.publish (marker);
     // marker_arr_.markers.push_back (marker);
     }
     */


  /*
   void
   publishInteractiveMarker (const cob_3d_mapping_msgs::Shape& s, std_msgs::Header header) //,std::vector::size_type sa_size)
   {

   //std::cout << " marker array size :" << marker_arr_.markers.size () << std::endl;
   * */

  /*
   for (unsigned int idx = 0; idx < marker_id_; idx++)
   {

   if (marker.id == idx)
   {
   ROS_INFO(" Deleting Marker with id = %d", marker.id);
   marker.action = visualization_msgs::Marker::DELETE;
   }
   }
   */

protected:

  ros::NodeHandle nh_;
  ros::Subscriber shape_array_sub_;

  ros::Publisher viz_msg_pub_;
  ros::Publisher viz_msg_im_pub_;
  ros::Publisher shape_pub_;

  unsigned int marker_id_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
  std::string im_name_;
  //visualization_msgs::InteractiveMarkerControl im_control_;
  //interactive_markers::MenuHandler menu_handler;
  //interactive_markers::MenuHandler::EntryHandle entry_handle_;
  interactive_markers::MenuHandler::EntryHandle eh_1;
};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "shape_visualization");

  ShapeVisualization sv;
  //std::cout << argc << "        " << argv[0] << std::endl;

  ros::Rate loop_rate (1);
  while (ros::ok ())
  {
    loop_rate.sleep ();
    ros::spin ();

  }
}

