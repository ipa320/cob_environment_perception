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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_common
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
#include <visualization_msgs/MenuEntry.h>
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
      marker_id_ (0), shape_ctr_ (0), display_marker_ (true)
  {
    shape_array_sub_ = nh_.subscribe ("shape_array", 1, &ShapeVisualization::shapeArrayCallback, this);
    viz_msg_pub_ = nh_.advertise<visualization_msgs::Marker> ("marker", 10);
    //viz_msg_im_pub_ = nh_.advertise<visualization_msgs::InteractiveMarker> ("interactive_marker", 1);
    //shape_pub_ = nh_.advertise<cob_3d_mapping_msgs::Shape> ("shape", 1);
    im_server_.reset (new interactive_markers::InteractiveMarkerServer ("shapes", "shape_server", false));

  }

  // Destructor
  ~ShapeVisualization ()
  {
    /// void
  }

  // ahape_array msgs callback
  void
  shapeArrayCallback (const cob_3d_mapping_msgs::ShapeArrayPtr &sa)
  {
    ROS_INFO_ONCE("shape array message received");
    ROS_INFO_ONCE("Shape Array Size: %d ", (int)sa->shapes.size());
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
  }

  void
  displayNormalCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback, unsigned int shape_idx)
  {
    ROS_INFO_ONCE(" Feedback -> display Normal  of shape_%d.....", shape_idx);
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
    string title;

    //menu_handler_.getTitle (feedback->menu_entry_id, title);

    interactive_markers::MenuHandler::CheckState check_state;

    menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED)
    {
      //ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
      displayNormal (true, shape_idx);
      menu_handler_.reApply (*im_server_);
      im_server_->applyChanges ();

    }
    if (check_state == interactive_markers::MenuHandler::CHECKED)
    {
      //ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
      displayNormal (false, shape_idx);
      menu_handler_.reApply (*im_server_);
      im_server_->applyChanges ();
    }
  }

  void
  displayCentroidCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback, unsigned int shape_idx)
  {
    ROS_INFO_ONCE(" Feedback : display centroid of shape_%d .....", shape_idx);

    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED)
    {
      //ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
      displayCentroid (true, shape_idx);
      menu_handler_.reApply (*im_server_);
      im_server_->applyChanges ();
    }
    if (check_state == interactive_markers::MenuHandler::CHECKED)
    {
      //ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
      displayCentroid (false, shape_idx);
      menu_handler_.reApply (*im_server_);
      im_server_->applyChanges ();
    }
  }
  void
  imServerCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    ROS_INFO(" Server Feedback .....");
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

  void
  displayNormal (bool display, unsigned int shape_idx)
  {
    stringstream ss;
    if (display == true)
    {
      ROS_INFO_ONCE(" Displaying normal vector .....");
      //marker_id_ = 0;
      visualization_msgs::Marker marker;

      marker.header.frame_id = header_.frame_id;
      marker.header.stamp = ros::Time::now ();
      marker.ns = "normal_marker";

      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration ();

      //set color
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 1;

      //set scale
      marker.scale.x = 1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      //set pose
      for (unsigned int k = 0; k < centroid_.size (); k++)
      {
        if (k == shape_idx)
        {
          marker.pose.position.x = centroid_[k][0] + 0.2;
          marker.pose.position.y = centroid_[k][1] + 0.2;
          marker.pose.position.z = 0;

          marker.pose.orientation.x = normal_[k][0];
          marker.pose.orientation.y = normal_[k][1];
          marker.pose.orientation.z = normal_[k][2];
          marker.pose.orientation.w = 1;
          break;
        }
      }

      marker.id = 0;
      /*
       std::cout << " normal vector marker id " << marker_id_ << std::endl;
       std::cout << " normal_[0] " << normal_[0][0] << std::endl;
       std::cout << " normal_[1] " << normal_[0][1] << std::endl;
       std::cout << " normal_[2] " << normal_[0][2] << std::endl;
       */

      visualization_msgs::InteractiveMarkerControl im_ctrl_n;
      im_ctrl_n.always_visible = true;
      ss.str("");
      ss << "normal_ctrl_" << shape_idx;
      im_ctrl_n.name = ss.str ();
      im_ctrl_n.description = "display_normal";

      //im_ctrl_n.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      im_ctrl_n.markers.push_back (marker);
      im.controls.push_back (im_ctrl_n);
      im_server_->insert (im);
    }

    else
    {
      ss << "normal_ctrl_" << shape_idx;
      ROS_INFO_ONCE(" removing marker.....");
      for (unsigned int i = 0; i < im.controls.size (); i++)
      {
        if (im.controls[i].name == ss.str ())
        {
          im.controls.erase (im.controls.begin () + i);
          break;
          //marker_id_--;
        }
      }
      im_server_->insert (im);

    }

  }

  void
  displayCentroid (bool display, int shape_idx)
  {
    stringstream ss;
    if (display == true)
    {
      ROS_INFO_ONCE(" Displaying centroid .....");
      //marker_id_ = 0;
      visualization_msgs::Marker marker;

      marker.header.frame_id = header_.frame_id;
      marker.header.stamp = ros::Time::now ();
      marker.ns = "centroid_marker";

      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration ();

      //set color
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 1;

      //set pose
      for (int k = 0; k < centroid_.size (); k++)
      {
        if (k == shape_idx)
        {
          /*
           std::cout<<" centroid_[k][0] <<"<<k<<" : "<<centroid_[k][0]<<std::endl;
           std::cout<<" centroid_[k][1] <<"<<k<<" : "<<centroid_[k][1]<<std::endl;
           std::cout<<" centroid_[k][2] <<"<<k<<" : "<<centroid_[k][2]<<std::endl;
           */
          marker.pose.position.x = centroid_[k][0];
          marker.pose.position.y = centroid_[k][1];
          marker.pose.position.z = centroid_[k][2];
          break;
        }
      }

      //set scale
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.id = 0; //marker_id_++;
      /*
       std::cout << " centroid marker id " << marker_id_ << std::endl;
       std::cout << " centroid_[0] " << centroid_[0][0] << std::endl;
       std::cout << " centroid_[1] " << centroid_[0][1] << std::endl;
       std::cout << " centroid_[2] " << centroid_[0][2] << std::endl;
        */

      visualization_msgs::InteractiveMarkerControl im_ctrl_c;
      im_ctrl_c.always_visible = true;
      ss.str("");
      ss << "centroid_ctrl_" << shape_idx;
      im_ctrl_c.name = ss.str ();
      //im_ctrl_c.description = "display centroid";
      //im_ctrl_c.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      im_ctrl_c.markers.push_back (marker);

      im.controls.push_back (im_ctrl_c);

      im_server_->insert (im);
    }


    else
    {
      ROS_INFO_ONCE(" removing centroid.....");
      ss << "centroid_ctrl_" << shape_idx;

      for (unsigned int i = 0; i < im.controls.size (); i++)
      {
        if (im.controls[i].name == ss.str ())
        {
          im.controls.erase (im.controls.begin () + i);
          break;
        }

      }
      im_server_->insert (im);

    }

  }

  void
  createMarker (list<TPPLPoly>& triangle_list, visualization_msgs::InteractiveMarkerControl& im_ctrl,
                const std_msgs::ColorRGBA& clr)
  {
    ROS_INFO_ONCE(" creating markers .....");
    marker_id_ = 0;
    visualization_msgs::Marker marker;

    marker.header.frame_id = header_.frame_id;
    marker.header.stamp = ros::Time::now ();
    marker.ns = "shape_marker";

    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    //marker.ns = "shape visualization";
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration ();

    //set color
    marker.color.r = clr.r;
    marker.color.g = clr.g;
    marker.color.b = clr.b;
    marker.color.a = clr.a;

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
      //viz_msg_pub_.publish (marker);
    }
  }
  void
  createShapeMenu ()
  {
    ROS_INFO(" creating shape menu ....shape_%d..,", shape_ctr_);
    // interactive_markers::MenuHandler menu_handler;
    interactive_markers::MenuHandler::EntryHandle eh_1, eh_2, eh_3;
    std::string n_title, c_title;
    stringstream ss;
    //ss << im.name << "/menu_shape_" << shape_ctr_;
    ss << "menu_shape_" << shape_ctr_;
    /*
     eh_1 = menu_handler_.insert (ss.str (), boost::bind (&ShapeVisualization::menuCB, this, _1));
     ss.str ("");
     ss << "Normal vector" << shape_ctr_;
     eh_2 = menu_handler_.insert (eh_1, ss.str (), boost::bind (&ShapeVisualization::displayNormalCB, this, _1));
     ss.str ("");
     ss << "Centroid" << shape_ctr_;
     eh_3 = menu_handler_.insert (eh_1, ss.str(), boost::bind (&ShapeVisualization::displayCentroidCB, this, _1));

     menu_handler_.setVisible (eh_1, true);
     menu_handler_.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);
     menu_handler_.setVisible (eh_2, true);
     menu_handler_.setCheckState (eh_2, interactive_markers::MenuHandler::UNCHECKED);
     menu_handler_.setVisible (eh_3, true);
     menu_handler_.setCheckState (eh_3, interactive_markers::MenuHandler::UNCHECKED);
     */
    eh_1 = menu_handler_.insert (ss.str (), boost::bind (&ShapeVisualization::menuCB, this, _1));
    ss.str ("");
    ss << "Normal_" << shape_ctr_;
    n_title = ss.str ();
    eh_2 = menu_handler_.insert (eh_1, n_title, boost::bind (&ShapeVisualization::displayNormalCB, this, _1, shape_ctr_));
    ss.str ("");
    ss << "Centroid_" << shape_ctr_;
    c_title = ss.str ();
    eh_3 = menu_handler_.insert (eh_1, c_title, boost::bind (&ShapeVisualization::displayCentroidCB, this, _1, shape_ctr_));


    std::cout<<" eh_1 : "<<eh_1<<std::endl;
    std::cout<<" eh_2 : "<<eh_2<<std::endl;
    std::cout<<" eh_3 : "<<eh_3<<std::endl;

    shape_ctr_++;

    menu_handler_.setVisible (eh_1, true);
    menu_handler_.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);
    menu_handler_.setVisible (eh_2, true);
    menu_handler_.setCheckState (eh_2, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler_.setVisible (eh_3, true);
    menu_handler_.setCheckState (eh_3, interactive_markers::MenuHandler::UNCHECKED);

    /*
     visualization_msgs::MenuEntry menu_entry;
     menu_entry.title = n_title;
     //menu_entry.id = shape_ctr_++;
     im.menu_entries.push_back(menu_entry);
     menu_entry.title = c_title;
     im.menu_entries.push_back(menu_entry);
     im_server_->insert(im);
     */

  }

  void
  publishInteractiveMarker (const cob_3d_mapping_msgs::Shape::ConstPtr& shape_msg, std_msgs::Header header) //,std::vector::size_type sa_size)
  {
    header_ = header;
    //shape_pub_.publish (*shape_msg);
    cob_3d_mapping::Polygon p;
    cob_3d_mapping::fromROSMsg (*shape_msg, p);
    normal_.push_back (p.normal);
    centroid_.push_back (p.centroid);
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
        //std::cout << " point cloud : " << pc[j] << std::endl;
        poly[j] = MsgToPoint2D (pc[j], shape_msg);
      }
      if (shape_msg->holes[i])
        poly.SetOrientation (TPPL_CW);
      else
        poly.SetOrientation (TPPL_CCW);

      polys.push_back (poly);
    }
    pp.Triangulate_EC (&polys, &result);
    /*
     ///////////////////////////////////////
     visualization_msgs::InteractiveMarker im_l;
     ROS_INFO_ONCE(" creating interactive marker .....");
     stringstream ss;
     ss << "shape_visualizer" << shape_ctr_;
     im_l.name = ss.str();
     ss.str("");
     im_l.description = "shape details";
     im_l.header = header;


     interactive_markers::MenuHandler menu_h;
     //////////////////////
     */
   //visualization_msgs::InteractiveMarker im;
    //interactive_markers::MenuHandler menu_h;
    stringstream ss;

    ROS_INFO_ONCE(" creating interactive marker .....");
    ss << "shape_visualizer";
    im.name = ss.str ();
    im.description = "shape details";
    im.header = header;

    ss.str ("");
    visualization_msgs::InteractiveMarkerControl im_ctrl;
    im_ctrl.always_visible = true;
    ss << "shape_" << shape_ctr_ << "control";
    im_ctrl.name = ss.str ();
    im_ctrl.description = "shape_markers";
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    createMarker (result, im_ctrl, shape_msg->color);

    im.controls.push_back (im_ctrl);

    //im_server_->insert (im, boost::bind (&ShapeVisualization::imServerCB, this, _1),visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
//////////////////////////////////////

    // im_l.controls.push_back (im_ctrl);
    //im_server_->insert (im_l);
//////////////////

    //im_server_->insert (im);
    // createShapeMenu ();
    //menu_handler_.apply (*im_server_, im.name);

    im_server_->insert (im);

    createShapeMenu ();

    //menu_h.apply (*im_server_, im.name);
    menu_handler_.apply (*im_server_, im.name);
    im_server_->applyChanges ();

    //viz_msg_im_pub_.publish (im);
    // shape_ctr_++;

  }

protected:

  ros::NodeHandle nh_;
  ros::Subscriber shape_array_sub_;

  ros::Publisher viz_msg_pub_;
  ros::Publisher viz_msg_im_pub_;
  //ros::Publisher shape_pub_;

  unsigned int marker_id_;
  bool display_marker_;unsigned int shape_ctr_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

  std_msgs::Header header_;

  visualization_msgs::InteractiveMarker im ;

  std::vector<Eigen::Vector3f> normal_;
  std::vector<Eigen::Vector4f> centroid_;

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

