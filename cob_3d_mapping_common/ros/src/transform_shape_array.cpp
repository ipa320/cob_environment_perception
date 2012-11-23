/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2012 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_environment_perception
 * \note
 * ROS package name: cob_3d_mapping_common
 *
 * \author
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 10/2012
 *
 * \brief
 * Node transforming a shape array using tf
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
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

// ROS core
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>


using namespace std;
using namespace cob_3d_mapping;

class TransformShapeArray
{
//protected:
//  ros::NodeHandle nh_;

public:
  string target_frame_;

  tf::TransformListener tf_listener_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  TransformShapeArray() :
    target_frame_("/map")
  {
    ros::NodeHandle private_nh("~");
    pub_ = private_nh.advertise<cob_3d_mapping_msgs::ShapeArray>("output",1);
    sub_ = private_nh.subscribe ("input", 1,  &TransformShapeArray::cloud_cb, this);

    private_nh.getParam ("target_frame", target_frame_);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Callback
  void
  cloud_cb (const cob_3d_mapping_msgs::ShapeArray::ConstPtr sa_in)
  {
    if(sa_in->header.frame_id != target_frame_)
    {
      Eigen::Affine3f af_target = Eigen::Affine3f::Identity();
      tf::StampedTransform trf_map;

      try
      {
        tf_listener_.waitForTransform(target_frame_, sa_in->header.frame_id, sa_in->header.stamp, ros::Duration(2));
        tf_listener_.lookupTransform(target_frame_, sa_in->header.frame_id, sa_in->header.stamp, trf_map);
      }
      catch (tf::TransformException ex) { ROS_ERROR("[transform shape array] : %s",ex.what()); return; }

      Eigen::Affine3d ad;
      tf::TransformTFToEigen(trf_map, ad);
      af_target = ad.cast<float>();
      cob_3d_mapping_msgs::ShapeArray sa_out;
      sa_out.header = sa_in->header;
      sa_out.header.frame_id = target_frame_;
      for (unsigned int i = 0; i < sa_in->shapes.size (); i++)
      {
        cob_3d_mapping_msgs::Shape s;
        s.header = sa_in->header;
        s.header.frame_id = target_frame_;
        if( sa_in->shapes[i].type == cob_3d_mapping_msgs::Shape::POLYGON)
        {
          Polygon::Ptr poly_ptr (new Polygon());
          fromROSMsg(sa_in->shapes[i], *poly_ptr);
          poly_ptr->transform2tf(af_target);
          toROSMsg(*poly_ptr,s);
        }
        else if( sa_in->shapes[i].type == cob_3d_mapping_msgs::Shape::CYLINDER)
        {
          Cylinder::Ptr cyl_ptr (new Cylinder());
          fromROSMsg(sa_in->shapes[i], *cyl_ptr);
          cyl_ptr->transform2tf(af_target);
          toROSMsg(*cyl_ptr,s);
        }
        sa_out.shapes.push_back (s);
      }
      pub_.publish(sa_out);
    }
    else
      pub_.publish(*sa_in);
  }

};


int
main (int argc, char** argv)
{
  ros::init (argc, argv, "transform_shape_array");

  TransformShapeArray b;
  ros::spin ();

  return (0);
}

