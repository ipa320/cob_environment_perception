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
#include <cob_3d_visualization/polypartition.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"


#define PI 3.14159265

using namespace cob_3d_mapping;


class ShapeMarker
{
public:

  ShapeMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& im_server,
              cob_3d_mapping_msgs::Shape& shape)
  {
    im_server_ = im_server;
    shape_ = shape;
    createShapeMenu ();
    createInteractiveMarker();
  }

  ~ShapeMarker()
  {
    if(im_server_->erase(marker_.name))
      ROS_INFO("Marker %s erased",marker_.name.c_str());
    stringstream ss;
    ss << "normal_" << shape_.id;
    im_server_->erase(ss.str());
    ss.str("");
    ss.clear();
    ss << "centroid_" << shape_.id;
    im_server_->erase(ss.str());
  }

  /**
   * @brief Create menu entries for each shape
   *
   * @param menu_h_ptr menu entry handler for current shape
   * @param transformation transformation matrix for this shape
   *
   */
  void
  createShapeMenu ()
  {
    //ROS_INFO(" creating menu .....");

    interactive_markers::MenuHandler::EntryHandle eh_1, eh_2, eh_3;

    eh_1 = menu_handler_.insert ("Display");
    eh_2 = menu_handler_.insert (eh_1, "Normal",boost::bind (&ShapeMarker::displayNormalCB, this, _1));
    //eh_2 = menu_handler_.insert (eh_1, "Normal");
    eh_3 = menu_handler_.insert (eh_1, "Centroid",boost::bind (&ShapeMarker::displayCentroidCB, this, _1));

    menu_handler_.setVisible (eh_1, true);
    menu_handler_.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);
    menu_handler_.setVisible (eh_2, true);
    menu_handler_.setCheckState (eh_2, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler_.setVisible (eh_3, true);
    menu_handler_.setCheckState (eh_3, interactive_markers::MenuHandler::UNCHECKED);

  }

  /**
   * @brief Create marker for the shape and add it to the interactive marker control
   *
   * @param triangle_list triangulated list of poly points
   * @param im_ctrl interactive marker control
   * @param transformation transformation matrix for this shape
   *
   */
  void
  createMarker (list<TPPLPoly>& triangle_list, visualization_msgs::InteractiveMarkerControl& im_ctrl)
  {
    //ROS_INFO(" creating markers for this shape.....");
    int ctr = 0;
    //std::cout << "triangle list " << triangle_list.size () << std::endl;
    //transformation = transformation.inverse ();

    TPPLPoint pt;
    for (std::list<TPPLPoly>::iterator it = triangle_list.begin (); it != triangle_list.end (); it++)
    {
      visualization_msgs::Marker marker;
      marker.id = ctr;
      ctr++;

      marker.header = shape_.header;
      //        marker.ns = "shape_marker";

      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker.ns = "shape visualization";
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration ();

      //set color
      marker.color.r = shape_.color.r;
      marker.color.g = shape_.color.g;
      marker.color.b = shape_.color.b;
      marker.color.a = shape_.color.a;

      //set scale
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;

      //set pose
      Eigen::Quaternionf quat (transformation_inv_.rotation ());
      Eigen::Vector3f trans (transformation_inv_.translation ());

      marker.pose.position.x = trans (0);
      marker.pose.position.y = trans (1);
      marker.pose.position.z = trans (2);

      marker.pose.orientation.x = quat.x ();
      marker.pose.orientation.y = quat.y ();
      marker.pose.orientation.z = quat.z ();
      marker.pose.orientation.w = quat.w ();

      //std::cout << "marker pose position: " << marker.pose.position << std::endl;
      //std::cout << "marker pose orientation: " << marker.pose.orientation << std::endl;
      //draw each triangle
      marker.points.resize (it->GetNumPoints ());
      //std::cout << "marker points = " << marker.points.size () << std::endl;
      for (long i = 0; i < it->GetNumPoints (); i++)
      {
        pt = it->GetPoint (i);
        marker.points[i].x = pt.x;
        marker.points[i].y = pt.y;
        marker.points[i].z = 0;
      }
      im_ctrl.markers.push_back (marker);
      //viz_msg_pub_.publish (marker);
    }

  }

  /**
   * @brief Create menu entries for each shape
   *
   * @param point 3D point to be transformed
   * @param transformation transformation matrix for this shape
   *
   * @return return transformed 2D TPPLPoint
   */
  TPPLPoint
  MsgToPoint2D (const pcl::PointXYZ &point)
  {
    //ROS_INFO(" transform 3D point to 2D ");
    TPPLPoint pt;
    Eigen::Vector3f p = transformation_ * point.getVector3fMap ();
    pt.x = p (0);
    pt.y = p (1);
    //ROS_INFO("\n ptXY:x = %f, y = %f ", pt.x, pt.y);
    //std::cout << "\n transformed point : \n" << p << std::endl;
    return pt;
  }

  /**
   * @brief Publish interactive markers for a shape message using interactive marker server
   *
   * @param shape_msg Shape message for which the interactive marker is to be created
   *
   */
  void
  createInteractiveMarker () //,std::vector::size_type sa_size)
  {
    ROS_INFO("\tcreating interactive marker for shape < %d >", shape_.id);
    /* increment shape idx on arrival of every new shape msg */
    //shape_ctr_++;
    //header_ = header;
    //shape_pub_.publish (*shape_msg);

    /* get normal and centroid */
    cob_3d_mapping::Polygon p;
    cob_3d_mapping::fromROSMsg (shape_, p);
    //normal_.push_back (p.normal.normalized ());
    //centroid_.push_back (p.centroid);

    /* transform shape points to 2d and store 2d point in triangle list */
    TPPLPartition pp;
    list<TPPLPoly> polys, tri_list;

    Eigen::Vector3f v, normal, origin;
    if (shape_.params.size () == 4)
    {

      normal (0) = shape_.params[0];
      normal (1) = shape_.params[1];
      normal (2) = shape_.params[2];
      origin (0) = shape_.centroid.x;
      origin (1) = shape_.centroid.y;
      origin (2) = shape_.centroid.z;
      v = normal.unitOrthogonal ();
      //std::cout << "normal: " << normal << std::endl;
      //std::cout << "normal.unitOrthogonal : " << v << std::endl;

      pcl::getTransformationFromTwoUnitVectorsAndOrigin (v, normal, origin, transformation_);
      transformation_inv_ = transformation_.inverse ();
      //std::cout << " transformation trans: " << transformation.translation () << std::endl;
      //std::cout << " transformation rotat: " << transformation.rotation() << std::endl;
      //transformation=transformation.inverse();
    }
    //std::cout << " shape size : " << shape_msg->points.size () << std::endl;
    for (size_t i = 0; i < shape_.points.size (); i++)
    {
      pcl::PointCloud<pcl::PointXYZ> pc;
      TPPLPoly poly;
      pcl::fromROSMsg (shape_.points[i], pc);
      poly.Init (pc.points.size ());
      poly.SetHole (shape_.holes[i]);

      for (size_t j = 0; j < pc.points.size (); j++)
      {
        //          std::cout << " point [ " << j << " ] : " << pc[j] << std::endl;
        poly[j] = MsgToPoint2D (pc[j]);

      }
      //std::cout<< " Hole : "<< shape_msg->holes[i]<<std::endl;
      if (shape_.holes[i])
        poly.SetOrientation (TPPL_CW);
      else
        poly.SetOrientation (TPPL_CCW);

      polys.push_back (poly);
    }
    pp.Triangulate_EC (&polys, &tri_list);

    /* create interactive marker for *this shape */
    stringstream ss;
    //      ss << "shape_visualizer_" << shape_ctr_;
    ss << shape_.id ;
    marker_.name = ss.str ();

    //std::cout << "Interactive Marker Name \n" << marker_.name << "\n" ;
    //int_marker.description = "shape details";
    marker_.header = shape_.header;

    ss.str ("");
    visualization_msgs::InteractiveMarkerControl im_ctrl;
    im_ctrl.always_visible = true;
    ss << "shape_" << shape_.id << "_control";
    im_ctrl.name = ss.str ();
    im_ctrl.description = "shape_markers";
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    /* create marker for *this shape and add it to interactive marker */
    createMarker (tri_list, im_ctrl);
    marker_.controls.push_back (im_ctrl);

    im_server_->insert (marker_);
    /* create menu for *this shape */
    menu_handler_.apply (*im_server_, marker_.name);

    //viz_msg_im_pub_.publish (im);
  }


  /**
   * @brief Feedback callback for normal menu entry
   *
   * @param feedback feedback from rviz when the normal menu entry of a shape is changed
   * @param shape_idx index of shape from which the feedback is received
   * @param menu_h_ptr pointer to menu entries of this shape
   * @param transformation matrix for this shape
   */
  void
  displayNormalCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    //ROS_INFO(" displayNormalCB from shape[ %d ]...", shape_.id);

    stringstream ss;
    interactive_markers::MenuHandler::CheckState check_state;

    menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED)
    {
      //ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
      visualization_msgs::InteractiveMarker imarker;
      ss << "normal_" << shape_.id;
      imarker.name = ss.str();
      imarker.header = shape_.header;
      ss.str("");
      ss.clear();
      //marker_id_ = 0;
      visualization_msgs::Marker marker;
      marker.header = shape_.header;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration ();

      //set color
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1;

      //set scale
      marker.scale.x = 0.05;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      //set pose
      marker.points.resize (2);
      marker.points[0].x = shape_.centroid.x;
      marker.points[0].y = shape_.centroid.y;
      marker.points[0].z = shape_.centroid.z;

      marker.points[1].x = shape_.centroid.x + shape_.params[0];
      marker.points[1].y = shape_.centroid.y + shape_.params[1];
      marker.points[1].z = shape_.centroid.z + shape_.params[2];


      //marker.id = shape_.id;

      visualization_msgs::InteractiveMarkerControl im_ctrl_n;

      im_ctrl_n.always_visible = true;
      ss << "normal_ctrl_" << shape_.id;
      im_ctrl_n.name = ss.str ();
      im_ctrl_n.description = "display_normal";

      //im_ctrl_n.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      im_ctrl_n.markers.push_back (marker);
      imarker.controls.push_back (im_ctrl_n);
      im_server_->insert (imarker);
    }
    else if (check_state == interactive_markers::MenuHandler::CHECKED)
    {
      //ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
      ss << "normal_" << shape_.id;
      im_server_->erase(ss.str());
    }
    menu_handler_.reApply (*im_server_);
    im_server_->applyChanges ();
  }
  /**
   * @brief Feedback callback for centroid menu entry
   *
   * @param feedback feedback from rviz when the centroid menu entry of a shape is changed
   * @param shape_idx index of shape from which the feedback is received
   * @param menu_h_ptr pointer to menu entries of this shape
   */
  void
  displayCentroidCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    //ROS_INFO(" displayCentroidCB from shape[ %d ]...", shape_.id);
    stringstream ss;
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED)
    {
      //ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
      visualization_msgs::InteractiveMarker imarker;
      ss << "centroid_" << shape_.id;
      imarker.name = ss.str();
      imarker.header = shape_.header;
      ss.str("");
      ss.clear();

      visualization_msgs::Marker marker;
      //marker.id = shape_.id;
      marker.header = shape_.header;

      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration ();

      //set color
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 1;

      //set scale
      marker.scale.x = 0.04;
      marker.scale.y = 0.04;
      marker.scale.z = 0.04;
      //std::cout << " shape index : " << shape_idx << std::endl;

      //set pose
      marker.pose.position.x = shape_.centroid.x;
      marker.pose.position.y = shape_.centroid.y;
      marker.pose.position.z = shape_.centroid.z;


      visualization_msgs::InteractiveMarkerControl im_ctrl;
      im_ctrl.always_visible = true;
      ss << "centroid_ctrl_" << shape_.id;
      im_ctrl.name = ss.str ();
      im_ctrl.markers.push_back (marker);
      imarker.controls.push_back (im_ctrl);
      im_server_->insert (imarker);
    }
    if (check_state == interactive_markers::MenuHandler::CHECKED)
    {
      //ROS_INFO(" entry state changed ");
      menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
      ss << "centroid_" << shape_.id;
      im_server_->erase(ss.str());
    }
    menu_handler_.reApply (*im_server_);
    im_server_->applyChanges ();
  }

  /**
   * @brief Display or remove the centroid of a shape
   *
   * @param display flag for displaying or removing centroid
   * @param shape_idx index of shape for which the centroid is to be displayed or removed
   *
   */
  void
  displayCentroid (bool display)
  {
    stringstream ss;
    if (display == true)
    {

    }

    else
    {
    }
    //im_server_->applyChanges();
  }

  visualization_msgs::InteractiveMarker marker_;

  //protected:
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
  cob_3d_mapping_msgs::Shape shape_;
  interactive_markers::MenuHandler menu_handler_;
  Eigen::Affine3f transformation_;
  Eigen::Affine3f transformation_inv_;
};

//####################
//#### node class ####
class ShapeVisualization
{
public:
  // Constructor
  ShapeVisualization ()
  {
    shape_array_sub_ = nh_.subscribe ("shape_array", 1, &ShapeVisualization::shapeArrayCallback, this);
    //viz_msg_pub_ = nh_.advertise<visualization_msgs::Marker> ("marker",10);
    //marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("marker_array",10);
    //    viz_msg_im_pub_ = nh_.advertise<visualization_msgs::InteractiveMarker> ("interactive_marker", 1);
    //    shape_pub_ = nh_.advertise<cob_3d_mapping_msgs::Shape> ("shape", 1);
    im_server_.reset (new interactive_markers::InteractiveMarkerServer ("geometry_map/map", "", false));
    //std::cout << "Ptr in Vis: " << im_server_.get() << std::endl;

  }

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
  void
  shapeArrayCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& sa)
  {
    //im_server_->clear();
    v_sm_.clear();
    im_server_->applyChanges();
    ROS_INFO("shape array with %d shapes received", sa->shapes.size());
    //shape_ctr_ = -1 ;
    //ROS_INFO_ONCE("Shape Array Size: %d ", (int)sa->shapes.size());
    //    if (display_marker_ == true)
    //    {
    //      display_marker_ = false;

    //sa_ = *sa;
    //header_ = sa->header;
    //im.name = "shape_visulization_marker";
    //im.header = sa->header;

    // im.description = "shape normal and centroid";

    for (unsigned int i = 0; i < sa->shapes.size (); i++)
    {
      boost::shared_ptr<ShapeMarker> sm(new ShapeMarker(im_server_, sa->shapes[i]));
      v_sm_.push_back(sm);
      //publishInteractiveMarker(sa->shapes[i]);
    }
    ROS_INFO("Size of vector: %d", v_sm_.size());
    im_server_->applyChanges(); //update changes
    //      std::cout << "Number of Interactive Markers Published : \n" << shape_ctr_2_ <<"\n" ;
  }
  //  }

  /**
   * @brief Display menu entry callback
   */
  /*void
  menuCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    ROS_INFO(" menu callback .........");
  }*/



protected:

  ros::NodeHandle nh_;

  ros::Subscriber shape_array_sub_; // sub for shape array msgs
  //ros::Publisher viz_msg_pub_; // pub for marker msgs
  //  ros::Publisher viz_msg_im_pub_;
  //ros::Publisher marker_array_pub_ ;

  //  ros::Publisher shape_pub_;

  std::vector<boost::shared_ptr<ShapeMarker> > v_sm_;
  //unsigned int shape_ctr_2_;
  //unsigned int marker_id_; // create markers with unique ids
  //unsigned int shape_ctr_; // keep track of shape indices
  //bool display_marker_; // check for displaying markers
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_; // server for interactive markers

  //std_msgs::Header header_; //header of the shape array msg

  //cob_3d_mapping_msgs::ShapeArray sa_;
  //visualization_msgs::InteractiveMarker im; // interactive marker for displaying normals and centroids
  //std::vector<Eigen::Vector3f> normal_; //stores normal of each shape
  //std::vector<Eigen::Vector4f> centroid_; //stores centroid of each shape

};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "shape_visualization");
  ROS_INFO("shape_visualization node started....");
  ShapeVisualization sv;

  ros::spin();
}

