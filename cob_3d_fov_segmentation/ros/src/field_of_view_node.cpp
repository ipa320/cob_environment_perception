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
 * ROS package name: cob_3d_fov_segmentation
 *
 * \author
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 02/2013
 *
 * \brief
 * Computes field of view of camera sensors.
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


//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
//#include <cob_3d_mapping_msgs/GetFieldOfView.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <cob_3d_fov_segmentation/field_of_viewConfig.h>
#include <cob_3d_fov_segmentation/field_of_view.h>


using namespace tf;
using namespace cob_3d_mapping;

//####################
//#### node class ####
class FieldOfViewNode
{
public:
  // Constructor
  FieldOfViewNode()
  {
    config_server_.setCallback(boost::bind(&FieldOfViewNode::dynReconfCallback, this, _1, _2));
    fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",1);
    //get_fov_srv_ = n_.advertiseService("get_fov", &FieldOfViewNode::srvCallback_GetFieldOfView, this);
  }


  // Destructor
  ~FieldOfViewNode()
  {
    /// void
  }

  void dynReconfCallback(cob_3d_fov_segmentation::field_of_viewConfig &config, uint32_t level)
  {
    fov_.setSensorFoV_hor(config.sensor_fov_hor_angle);
    fov_.setSensorFoV_ver(config.sensor_fov_ver_angle);
    fov_.setSensorMaxRange(config.sensor_max_range);
    camera_frame_ = config.camera_frame;
    target_frame_ = config.target_frame;

    //new settings -> recalculate
    fov_.computeFieldOfView();
  }


  /**
   * @brief uses global transformation (of the robot) to recalculate the field of view
   *
   * uses global transformation (of the robot) to recalculate the field of view (all vectors)
   *
   * @param stamp timestamp indicating used frame
   * @param target_frame targetframe
   *
   * @return nothing
   */
  void transformFOV()
  {
    //std::string target_frame = std::string("/map");
    StampedTransform st_trf;
    try
    {
      tf_listener_.waitForTransform(target_frame_, camera_frame_, ros::Time(0), ros::Duration(0.1));
      tf_listener_.lookupTransform(target_frame_, camera_frame_, ros::Time(0), st_trf);
    }
    catch (TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    Eigen::Affine3d trafo;
    transformTFToEigen(st_trf, trafo);
    fov_.transformFOV(trafo);
    publishMarker();
  }


  /**
   * @brief generates markers to visualize field of view vectors
   *
   * generates markers to visualize field of view vectors
   *
   * @param stamp timestamp indicating used frame
   * @param target_frame targetframe
   *
   * @return markers
   */
  void publishMarker()
  {
    Eigen::Vector3d p_0;
    Eigen::Vector3d p_1;
    Eigen::Vector3d p_2;
    Eigen::Vector3d p_3;
    Eigen::Vector3d p_4;
    fov_.getFOV(p_0, p_1, p_2, p_3, p_4);

    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = ros::Time::now();

    //marker.pose = pose_msg;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.01;
    marker.points.resize(16);


    marker.points[0].x = p_0(0);
    marker.points[0].y = p_0(1);
    marker.points[0].z = p_0(2);

    marker.points[1].x = p_1(0);
    marker.points[1].y = p_1(1);
    marker.points[1].z = p_1(2);

    marker.points[2].x = p_0(0);
    marker.points[2].y = p_0(1);
    marker.points[2].z = p_0(2);

    marker.points[3].x = p_2(0);
    marker.points[3].y = p_2(1);
    marker.points[3].z = p_2(2);

    marker.points[4].x = p_0(0);
    marker.points[4].y = p_0(1);
    marker.points[4].z = p_0(2);

    marker.points[5].x = p_3(0);
    marker.points[5].y = p_3(1);
    marker.points[5].z = p_3(2);

    marker.points[6].x = p_0(0);
    marker.points[6].y = p_0(1);
    marker.points[6].z = p_0(2);

    marker.points[7].x = p_4(0);
    marker.points[7].y = p_4(1);
    marker.points[7].z = p_4(2);

    marker.points[8].x = p_1(0);
    marker.points[8].y = p_1(1);
    marker.points[8].z = p_1(2);

    marker.points[9].x = p_2(0);
    marker.points[9].y = p_2(1);
    marker.points[9].z = p_2(2);

    marker.points[10].x = p_2(0);
    marker.points[10].y = p_2(1);
    marker.points[10].z = p_2(2);

    marker.points[11].x = p_3(0);
    marker.points[11].y = p_3(1);
    marker.points[11].z = p_3(2);

    marker.points[12].x = p_3(0);
    marker.points[12].y = p_3(1);
    marker.points[12].z = p_3(2);

    marker.points[13].x = p_4(0);
    marker.points[13].y = p_4(1);
    marker.points[13].z = p_4(2);

    marker.points[14].x = p_4(0);
    marker.points[14].y = p_4(1);
    marker.points[14].z = p_4(2);

    marker.points[15].x = p_1(0);
    marker.points[15].y = p_1(1);
    marker.points[15].z = p_1(2);

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    fov_marker_pub_.publish(marker);
  }



  /**
   * @brief request to recalculate the field of view
   *
   * request to recalculate the field of view
   *
   * @param req parameters
   * @param res containing field of view vectors
   *
   * @return nothing
   */
  /*bool srvCallback_GetFieldOfView(cob_3d_mapping_msgs::GetFieldOfView::Request &req,
                                  cob_3d_mapping_msgs::GetFieldOfView::Response &res)
  {
    //ROS_INFO("FieldOfView Trigger");
    transformFOV(req.stamp, req.target_frame);
    geometry_msgs::Point n_msg;
    pointTFToMsg(n_up_t_, n_msg);
    res.fov.points.push_back(n_msg);
    pointTFToMsg(n_down_t_, n_msg);
    res.fov.points.push_back(n_msg);
    pointTFToMsg(n_right_t_, n_msg);
    res.fov.points.push_back(n_msg);
    pointTFToMsg(n_left_t_, n_msg);
    res.fov.points.push_back(n_msg);
    pointTFToMsg(n_origin_t_, n_msg);
    res.fov.points.push_back(n_msg);

    return true;
  }*/

protected:
  ros::NodeHandle n_;
  ros::Publisher fov_marker_pub_;
  //ros::ServiceServer get_fov_srv_;

  TransformListener tf_listener_;
  dynamic_reconfigure::Server<cob_3d_fov_segmentation::field_of_viewConfig> config_server_;

  std::string camera_frame_;
  std::string target_frame_;

  FieldOfView fov_;


};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "field_of_view");

  FieldOfViewNode fov;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    fov.transformFOV();
    loop_rate.sleep();
  }
}



