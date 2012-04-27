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
#include <sstream>
//--

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
//#include <visualization_msgs/MenuEntry.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
//#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/common/transform.h>
#include <pcl/common/rigid_transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <cmath>
//#include <rviz/selection/selection_manager.h>
//#include <rviz/selection/selection_handler.h>
//#include <rviz/selection/forwards.h>
#include <Eigen/Core>

#include <cob_3d_mapping_rviz_plugins/polypartition.h>
//#include <cob_3d_mapping_rviz_plugins/shape_marker.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>

//####################
//#### node class ####
class ShapeVisualization
{
public:
  // Constructor
  ShapeVisualization () :
      marker_id_ (0), display_marker_ (true)
  {
    shape_array_sub_ = nh_.subscribe ("shape_array", 1, &ShapeVisualization::shapeArrayCallback, this);
    viz_msg_pub_ = nh_.advertise < visualization_msgs::Marker > ("marker", 10);
    viz_msg_im_pub_ = nh_.advertise < visualization_msgs::InteractiveMarker > ("interactive_marker", 1);
    shape_pub_ = nh_.advertise < cob_3d_mapping_msgs::Shape > ("shape", 1);
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
    ROS_INFO_ONCE("shape array message received");
    //ROS_INFO("Shape Array Size: %d ", (int)sa->shapes.size());
    if (display_marker_ == true)
    {
      ROS_INFO("displaying shape array message ");
      display_marker_ = false;
      for (unsigned int i = 0; i < sa->shapes.size (); i++)
      {
        boost::shared_ptr<cob_3d_mapping_msgs::Shape> s_ptr = boost::make_shared<cob_3d_mapping_msgs::Shape> (
            sa->shapes[i]);
        publishInteractiveMarker (s_ptr, sa->header);
      }
    }
  }

  void
  menuCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    ROS_INFO(" Menu Feedback .....");
    /*
     std::cout << "  feedback->__connection_header : " << feedback->__connection_header << cout << endl;
     std::cout << "  feedback->client_id : " << feedback->client_id << cout << endl;
     std::cout << "  feedback->control_name : " << feedback->control_name << cout << endl;
     std::cout << "  feedback->event_type : " << feedback->event_type << cout << endl;
     std::cout << "  feedback->header : " << feedback->header << cout << endl;
     std::cout << "  feedback->marker_name : " << feedback->marker_name << cout << endl;
     std::cout << "  feedback->menu_entry_id : " << feedback->menu_entry_id << cout << endl;
     std::cout << "  feedback->mouse_point : " << feedback->mouse_point << cout << endl;
     std::cout << "  feedback->mouse_point_valid : " << feedback->mouse_point_valid << cout << endl;
     std::cout << "  feedback->pose : " << feedback->pose << cout << endl;
     */
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED)
    {
      ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.reApply (*im_server_);
      im_server_->applyChanges ();
    }
    if (check_state == interactive_markers::MenuHandler::CHECKED)
    {
      ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.reApply (*im_server_);
      im_server_->applyChanges ();
    }
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
    ROS_INFO_ONCE(" creating interactive marker .....");
    im.name = "shape_visualizer";
    im.description = "shape details";
    im.header = header;
    //im.header.frame_id = header.frame_id;
    //im.header.stamp = ros::Time::now();
    //std::cout << "  interactive marker HEADER:  " << im.header << std::endl;
    // im_name_ = im.name;

    im_ctrl.always_visible = true;
    im_ctrl.name = "move";
    im_ctrl.description = "normal_vector";
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  }

  TPPLPoint
  MsgToPoint2D (const pcl::PointXYZ &point, const cob_3d_mapping_msgs::Shape::ConstPtr& new_message)
  {
    TPPLPoint pt;

    if (new_message->params.size () == 4)
    {
      Eigen::Vector3f u, v, normal, origin;
      Eigen::Affine3f transformation;
      normal (0) = new_message->params[0];
      normal (1) = new_message->params[1];
      normal (2) = new_message->params[2];
      origin (0) = new_message->centroid.x;
      origin (1) = new_message->centroid.y;
      origin (2) = new_message->centroid.z;
      //std::cout << "normal: " << normal << std::endl;
      //std::cout << "centroid: " << origin << std::endl;
      v = normal.unitOrthogonal ();
      u = normal.cross (v);
      pcl::getTransformationFromTwoUnitVectorsAndOrigin (v, normal, origin, transformation);

      Eigen::Vector3f p3 = transformation * point.getVector3fMap ();
      pt.x = p3 (0);
      pt.y = p3 (1);
    }
    else if (new_message->params.size () == 5)
    {
      pt.x = point.x;
      pt.y = point.y;
    }

    return pt;
  }

  /*
   void
   createMarker (cob_3d_mapping::Polygon& p, visualization_msgs::InteractiveMarkerControl& im_ctrl,
   std_msgs::Header& header)
   {
   ROS_INFO_ONCE(" creating markers .....");
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
   // createMarkerMenu (marker);
   viz_msg_pub_.publish (marker);
   }
   }
   */
  void
  createMarker (list<TPPLPoly>& triangle_list, visualization_msgs::InteractiveMarker& im,
                visualization_msgs::InteractiveMarkerControl& im_ctrl, std_msgs::Header& header)
  {
    ROS_INFO_ONCE(" creating markers .....");
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
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    //set scale
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    TPPLPoint pt;

    for (std::list<TPPLPoly>::iterator it = triangle_list.begin (); it != triangle_list.end (); it++)
    {
      //draw each triangle
      marker.id = marker_id_++;
      marker.points.resize (it->GetNumPoints ());
      for (long i = 0; i < it->GetNumPoints (); i++)
      {
        pt = it->GetPoint (i);
        //std::cout << " result :: x = " << pt.x << "  y = " << pt.y << std::endl;
        marker.points[i].x = pt.x;
        marker.points[i].y = pt.y;
        marker.points[i].z = 0;
        //Eigen::Vector3f p3, normal;
        //MsgToPoint3D(p1,new_message,p3,normal);

        //polygon_->position(p3(0),p3(1),p3(2));  // start position
        //polygon_->normal(normal(0),normal(1),normal(2));
      }
      im_ctrl.markers.push_back (marker);
      createMarkerMenu (marker, im);
      //viz_msg_pub_.publish (marker);
    }
  }
  void
  createMarkerMenu (visualization_msgs::Marker& marker, visualization_msgs::InteractiveMarker& im)
  {
    ROS_INFO_ONCE(" creating marker menu .....");
    interactive_markers::MenuHandler menu_handler;
    interactive_markers::MenuHandler::EntryHandle eh_1, eh_2, eh_3;
    stringstream ss;
    if (marker.id == 0)
    {
      ss << marker.ns << "/" << marker.id;
      eh_1 = menu_handler_.insert (ss.str (), boost::bind (&ShapeVisualization::menuCB, this, _1));
      eh_2 = menu_handler_.insert (eh_1, "Normal vector", boost::bind (&ShapeVisualization::menuCB, this, _1));
      eh_3 = menu_handler_.insert (eh_1, "d", boost::bind (&ShapeVisualization::menuCB, this, _1));
      menu_handler_.setVisible (eh_1, true);
      menu_handler_.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);
      menu_handler_.setVisible (eh_2, true);
      menu_handler_.setCheckState (eh_2, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setVisible (eh_3, true);
      menu_handler_.setCheckState (eh_3, interactive_markers::MenuHandler::UNCHECKED);
      //menu_handler_.apply (*im_server_, im.name);
      //im_server_->applyChanges ();
    }
    if (marker.id == 1)
    {
      ss << marker.ns << "/" << marker.id;
      eh_1 = menu_handler_.insert (ss.str (), boost::bind (&ShapeVisualization::menuCB, this, _1));
      eh_2 = menu_handler_.insert (eh_1, "Normal vector", boost::bind (&ShapeVisualization::menuCB, this, _1));
      eh_3 = menu_handler_.insert (eh_1, "d", boost::bind (&ShapeVisualization::menuCB, this, _1));
      menu_handler_.setVisible (eh_1, true);
      menu_handler_.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);
      menu_handler_.setVisible (eh_2, true);
      menu_handler_.setCheckState (eh_2, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setVisible (eh_3, true);
      menu_handler_.setCheckState (eh_3, interactive_markers::MenuHandler::UNCHECKED);
      //menu_handler_.apply (*im_server_, im.name);
      //im_server_->applyChanges ();
    }

    /*
     if (marker.id == 0)
     {
     ss << marker.ns << "/" << marker.id;
     menu1_eh_1_ = menu_handler_.insert (ss.str (), boost::bind (&ShapeVisualization::menuCB, this, _1));
     menu1_eh_2_ = menu_handler_.insert (menu1_eh_1_, "Normal vector", boost::bind (&ShapeVisualization::menuCB, this, _1));
     menu1_eh_3_ = menu_handler_.insert (menu1_eh_1_, "d", boost::bind (&ShapeVisualization::menuCB, this, _1));
     menu_handler_.setVisible (menu1_eh_1_, true);
     menu_handler_.setCheckState (menu1_eh_1_, interactive_markers::MenuHandler::NO_CHECKBOX);
     menu_handler_.setVisible (menu1_eh_2_, true);
     menu_handler_.setCheckState (menu1_eh_2_, interactive_markers::MenuHandler::UNCHECKED);
     menu_handler_.setVisible (menu1_eh_3_, true);
     menu_handler_.setCheckState (menu1_eh_3_, interactive_markers::MenuHandler::UNCHECKED);
     //menu_handler_.apply (*im_server_, im.name);
     //im_server_->applyChanges ();
     }
     if (marker.id == 1)
     {
     ss << marker.ns << "/" << marker.id;
     menu2_eh_1_ = menu_handler_.insert (ss.str (), boost::bind (&ShapeVisualization::menuCB, this, _1));
     menu2_eh_2_ = menu_handler_.insert (menu2_eh_1_, "Normal vector", boost::bind (&ShapeVisualization::menuCB, this, _1));
     menu2_eh_3_ = menu_handler_.insert (menu2_eh_1_, "d", boost::bind (&ShapeVisualization::menuCB, this, _1));
     menu_handler_.setVisible (menu2_eh_1_, true);
     menu_handler_.setCheckState (menu2_eh_1_, interactive_markers::MenuHandler::NO_CHECKBOX);
     menu_handler_.setVisible (menu2_eh_2_, true);
     menu_handler_.setCheckState (menu2_eh_2_, interactive_markers::MenuHandler::UNCHECKED);
     menu_handler_.setVisible (menu2_eh_3_, true);
     menu_handler_.setCheckState (menu2_eh_3_, interactive_markers::MenuHandler::UNCHECKED);
     //menu_handler_.apply (*im_server_, im.name);
     //im_server_->applyChanges ();
     }
     */
  }

/*
  void
  displayNormal (visualization_msgs::InteractiveMarkerControl& im_ctrl)
  {
    ROS_INFO_ONCE(" Displaying normal vector .....");
    //marker_id_ = 0;
    visualization_msgs::Marker marker;

    marker.header.frame_id = header_.frame_id;
    // marker.header.stamp = ros::Time::now();
    marker.ns = "shape_marker";

    marker.type = visualization_msgs::Marker::ARROW;
    //marker.ns = "shape visualization";
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration ();

    //set color
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;

    //set scale
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.id = marker_id_++;

    std::cout<< " normal_[0]" <<normal_[0]<<std::endl;
    marker.points[0].x = normal_[0];
    marker.points[0].y = normal_[1];
    marker.points[0].z = normal_[2];

    im_ctrl.markers.push_back (marker);
    //createMarkerMenu (marker, im);
    viz_msg_pub_.publish (marker);
}
*/
void
publishInteractiveMarker (const cob_3d_mapping_msgs::Shape::ConstPtr& shape_msg, std_msgs::Header header) //,std::vector::size_type sa_size)
{
 // header_ = header;
  //shape_pub_.publish (*shape_msg);
  cob_3d_mapping::Polygon p;
  cob_3d_mapping::fromROSMsg (*shape_msg, p);
  normal_ = p.normal;
  centroid_ = p.centroid;
  //std::cout<< "normal" << normal_ <<std::endl;
  TPPLPartition pp;
  list<TPPLPoly> polys, result;

  for (size_t i = 0; i < shape_msg->points.size (); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> pc;
    TPPLPoly poly;
    pcl::fromROSMsg (shape_msg->points[i], pc);
    poly.Init (pc.points.size ());
    poly.SetHole (shape_msg->holes[i]);

    for (size_t j = 0; j < pc.points.size (); j++)
    {
      std::cout << " point cloud : " << pc[j] << std::endl;
      poly[j] = MsgToPoint2D (pc[j], shape_msg);
    }
    if (shape_msg->holes[i])
      poly.SetOrientation (TPPL_CW);
    else
      poly.SetOrientation (TPPL_CCW);

    polys.push_back (poly);
  }
  pp.Triangulate_EC (&polys, &result);

  visualization_msgs::InteractiveMarker im;
  visualization_msgs::InteractiveMarkerControl im_ctrl;

  createInteractiveMarker (im, im_ctrl, header);
  createMarker (result, im, im_ctrl, header);

  displayNormal(im_ctrl);
  im.controls.push_back (im_ctrl);
  //im_server_->insert(im);

  im_server_->insert (im, boost::bind (&ShapeVisualization::imServerCB, this, _1),
                      visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
  menu_handler_.apply (*im_server_, im.name);

  im_server_->applyChanges ();
  //viz_msg_im_pub_.publish (im);

}

protected:

ros::NodeHandle nh_;
ros::Subscriber shape_array_sub_;

ros::Publisher viz_msg_pub_;
ros::Publisher viz_msg_im_pub_;
ros::Publisher shape_pub_;

unsigned int marker_id_;
bool display_marker_;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
std_msgs::Header header_;

Eigen::Vector3f normal_;
Eigen::Vector4f centroid_;

interactive_markers::MenuHandler menu_handler_;
//interactive_markers::MenuHandler::EntryHandle menu1_eh_1_, menu1_eh_2_, menu1_eh_3_;
//interactive_markers::MenuHandler::EntryHandle menu2_eh_1_, menu2_eh_2_, menu2_eh_3_;
};

int
main (int argc, char** argv)
{
ros::init (argc, argv, "shape_visualization");
ROS_INFO(" shape_visualization node started....");
ShapeVisualization sv;
//std::cout << argc << "        " << argv[0] << std::endl;

ros::Rate loop_rate (1);
while (ros::ok ())
{
  loop_rate.sleep ();
  ros::spinOnce ();

}
}

