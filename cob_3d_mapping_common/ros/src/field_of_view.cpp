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
* \date Date of creation: 02/2011
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
#include <cob_3d_mapping_msgs/GetFieldOfView.h>
#include <Eigen/Core>
//#include "cob_3d_mapping_common/reconfigureable_node.h"
#include <dynamic_reconfigure/server.h>
#include <cob_3d_mapping_common/field_of_viewConfig.h>

// external includes
#include <boost/timer.hpp>


using namespace tf;

//####################
//#### node class ####
class FieldOfView
{
public:
  // Constructor
  FieldOfView()
  {
    config_server_.setCallback(boost::bind(&FieldOfView::dynReconfCallback, this, _1, _2));
    fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",10);
    get_fov_srv_ = n_.advertiseService("get_fov", &FieldOfView::srvCallback_GetFieldOfView, this);
    /*setSensorFoV_hor(cob_3d_mapping_common::field_of_viewConfig::__getDefault__().sensor_fov_hor_angle);
    setSensorFoV_ver(cob_3d_mapping_common::field_of_viewConfig::__getDefault__().sensor_fov_ver_angle);
    setSensorMaxRange(cob_3d_mapping_common::field_of_viewConfig::__getDefault__().sensor_max_range);*/
    //camera_frame_ = std::string(/*"/base_kinect_rear_link"*/"/head_cam3d_link");
    computeFieldOfView();

    //setReconfigureCallback2(boost::bind(&callback, this, _1, _2), boost::bind(&callback_get, this, _1));
  }


  // Destructor
  ~FieldOfView()
  {
    /// void
  }

  void dynReconfCallback(cob_3d_mapping_common::field_of_viewConfig &config, uint32_t level)
  {
      setSensorFoV_hor(config.sensor_fov_hor_angle);
      setSensorFoV_ver(config.sensor_fov_ver_angle);
      setSensorMaxRange(config.sensor_max_range);
      camera_frame_ = config.camera_frame;

    //new settings -> recalculate
    computeFieldOfView();
  }

  /**
   * @brief callback for dynamic reconfigure
   *
   * everytime the dynamic reconfiguration changes this function will be called
   *
   * @param inst instance of FieldOfView which parameters should be changed
   * @param config data of configuration
   * @param level bit descriptor which notifies which parameter changed
   *
   * @return nothing
   */
  /*static void callback(FieldOfView *fov, cob_3d_mapping_common::field_of_viewConfig &config, uint32_t level)
  {
    if(!fov)
      return;

    boost::mutex::scoped_lock l(fov->m_mutex);

    if(level&1) //hor changed
      fov->setSensorFoV_hor(config.sensor_fov_hor_angle);
    if(level&2) //ver changed
      fov->setSensorFoV_ver(config.sensor_fov_ver_angle);
    if(level&4) //range changed
      fov->setSensorMaxRange(config.sensor_max_range);

    //new settings -> recalculate
    fov->computeFieldOfView();
  }

  // callback for dynamic reconfigure
  static void callback_get(FieldOfView *fov, cob_3d_mapping_common::field_of_viewConfig &config)
  {
    if(!fov)
      return;

    boost::mutex::scoped_lock l(fov->m_mutex);

    config.sensor_fov_hor_angle = fov->sensor_fov_hor_;
    config.sensor_fov_ver_angle = fov->sensor_fov_ver_;
    config.sensor_max_range = fov->sensor_max_range_;
  }*/

  void setSensorFoV_hor(double val) {
    sensor_fov_hor_ = val*M_PI/180;
  }

  void setSensorFoV_ver(double val) {
    sensor_fov_ver_ = val*M_PI/180;
  }

  void setSensorMaxRange(double val) {
    sensor_max_range_ = val;
  }

  /**
   * @brief calculates coordinate system from parameters
   *
   * calculates coordinate system from parameters
   *
   * @return nothing
   */
  void computeFieldOfView()
  {
    //don't lock
    double fovHorFrac = sensor_fov_hor_/2;
    double fovVerFrac = sensor_fov_ver_/2;

    vec_a_(0) = tan(fovHorFrac)*sensor_max_range_;
    vec_a_(1) = -tan(fovVerFrac)*sensor_max_range_;
    vec_a_(2) = sensor_max_range_;

    vec_b_(0) = -vec_a_(0);
    vec_b_(1) = vec_a_(1);
    vec_b_(2) = sensor_max_range_;

    vec_c_(0) = -vec_a_(0);
    vec_c_(1) = -vec_a_(1);
    vec_c_(2) = sensor_max_range_;

    vec_d_(0) = vec_a_(0);
    vec_d_(1) = -vec_a_(1);
    vec_d_(2) = sensor_max_range_;

    /*std::cout << "a: " << vec_a << std::endl;
			std::cout << "b: " << vec_b << std::endl;
			std::cout << "c: " << vec_c << std::endl;
			std::cout << "d: " << vec_d << std::endl;*/

    /// cross product b x a
    n_up_ = tf::Point(0,vec_a_(2)*vec_b_(0) - vec_a_(0)*vec_b_(2),vec_a_(0)*vec_b_(1) - vec_a_(1)*vec_b_(0));

    /// cross product c x d
    n_down_ = tf::Point(0, -n_up_[1], n_up_[2]);

    /// cross product b x c
    n_left_  = tf::Point(vec_b_(1)*vec_c_(2) - vec_b_(2)*vec_c_(1), 0, vec_b_(0)*vec_c_(1) - vec_b_(1)*vec_c_(0));

    /// cross product d x a
    n_right_ = tf::Point(-n_left_[0], 0, n_left_[2]);

    /*std::cout << "n_up: " << n_up << std::endl;
			std::cout << "n_down: " << n_down << std::endl;
			std::cout << "n_right: " << n_right << std::endl;
			std::cout << "n_left: " << n_left << std::endl;*/

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
  void transformFOV(ros::Time stamp, std::string target_frame)
  {
    //std::string target_frame = std::string("/map");
    StampedTransform transform;
    try
    {
      tf_listener_.waitForTransform(target_frame, camera_frame_, stamp, ros::Duration(0.1));
      tf_listener_.lookupTransform(target_frame, camera_frame_, stamp/*ros::Time(0)*/, transform);
      //tf::Point n_up(n_up_(0),n_up_(1),n_up_(2));
      tf::Stamped<tf::Point> stamped_n_up(n_up_,transform.stamp_,camera_frame_);
      tf::Stamped<tf::Point> stamped_n_up_t;
      try{
        tf_listener_.transformPoint(target_frame,stamped_n_up, stamped_n_up_t);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }

      n_up_t_[0] = stamped_n_up_t.x();
      n_up_t_[1] = stamped_n_up_t.y();
      n_up_t_[2] = stamped_n_up_t.z();

      //tf::Point n_down(n_down_(0),n_down_(1),n_down_(2));
      tf::Stamped<tf::Point> stamped_n_down(n_down_,transform.stamp_,camera_frame_);
      tf::Stamped<tf::Point> stamped_n_down_t;
      try{
        tf_listener_.transformPoint(target_frame,stamped_n_down, stamped_n_down_t);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      n_down_t_[0] = stamped_n_down_t.x();
      n_down_t_[1] = stamped_n_down_t.y();
      n_down_t_[2] = stamped_n_down_t.z();

      //tf::Point n_right(n_right_(0),n_right_(1),n_right_(2));
      tf::Stamped<tf::Point> stamped_n_right(n_right_,transform.stamp_,camera_frame_);
      tf::Stamped<tf::Point> stamped_n_right_t;
      try{
        tf_listener_.transformPoint(target_frame,stamped_n_right, stamped_n_right_t);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      n_right_t_[0] = stamped_n_right_t.x();
      n_right_t_[1] = stamped_n_right_t.y();
      n_right_t_[2] = stamped_n_right_t.z();

      //tf::Point n_left(n_left_(0),n_left_(1),n_left_(2));
      tf::Stamped<tf::Point> stamped_n_left(n_left_,transform.stamp_,camera_frame_);
      tf::Stamped<tf::Point> stamped_n_left_t;
      try{
        tf_listener_.transformPoint(target_frame,stamped_n_left, stamped_n_left_t);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      n_left_t_[0] = stamped_n_left_t.x();
      n_left_t_[1] = stamped_n_left_t.y();
      n_left_t_[2] = stamped_n_left_t.z();

      tf::Point n_origin(0,0,0);
      tf::Stamped<tf::Point> stamped_n_origin(n_origin,transform.stamp_,camera_frame_);
      tf::Stamped<tf::Point> stamped_n_origin_t;
      try{
        tf_listener_.transformPoint(target_frame,stamped_n_origin, stamped_n_origin_t);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      n_origin_t_[0] = stamped_n_origin_t.x();
      n_origin_t_[1] = stamped_n_origin_t.y();
      n_origin_t_[2] = stamped_n_origin_t.z();

      visualization_msgs::Marker marker = generateMarker(target_frame, transform.stamp_);
      fov_marker_pub_.publish(marker);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
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
  visualization_msgs::Marker generateMarker(std::string& target_frame, ros::Time& stamp)
  {
    tf::Pose marker_pose(btQuaternion(0,0,0,1),btVector3(0,0.5,0));
    tf::Stamped<tf::Pose> stamped_marker_pose(marker_pose, stamp, camera_frame_);
    tf::Stamped<tf::Pose> stamped_marker_pose_t;
    try{
				tf_listener_.transformPose(target_frame, stamped_marker_pose, stamped_marker_pose_t);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
    visualization_msgs::Marker marker;
    marker.header.frame_id = camera_frame_;//target_frame;
    marker.header.stamp = stamp;
    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg(stamped_marker_pose_t, pose_msg);
    //marker.pose = pose_msg;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.01;
    marker.points.resize(16);


    marker.points[0].x = 0;
    marker.points[0].y = 0;
    marker.points[0].z = 0;

    marker.points[1].x = vec_a_(0);
    marker.points[1].y = vec_a_(1);
    marker.points[1].z = vec_a_(2);

    marker.points[2].x = 0;
    marker.points[2].y = 0;
    marker.points[2].z = 0;

    marker.points[3].x = vec_b_(0);
    marker.points[3].y = vec_b_(1);
    marker.points[3].z = vec_b_(2);

    marker.points[4].x = 0;
    marker.points[4].y = 0;
    marker.points[4].z = 0;

    marker.points[5].x = vec_c_(0);
    marker.points[5].y = vec_c_(1);
    marker.points[5].z = vec_c_(2);

    marker.points[6].x = 0;
    marker.points[6].y = 0;
    marker.points[6].z = 0;

    marker.points[7].x = vec_d_(0);
    marker.points[7].y = vec_d_(1);
    marker.points[7].z = vec_d_(2);

    marker.points[8].x = vec_a_(0);
    marker.points[8].y = vec_a_(1);
    marker.points[8].z = vec_a_(2);

    marker.points[9].x = vec_b_(0);
    marker.points[9].y = vec_b_(1);
    marker.points[9].z = vec_b_(2);

    marker.points[10].x = vec_b_(0);
    marker.points[10].y = vec_b_(1);
    marker.points[10].z = vec_b_(2);

    marker.points[11].x = vec_c_(0);
    marker.points[11].y = vec_c_(1);
    marker.points[11].z = vec_c_(2);

    marker.points[12].x = vec_c_(0);
    marker.points[12].y = vec_c_(1);
    marker.points[12].z = vec_c_(2);

    marker.points[13].x = vec_d_(0);
    marker.points[13].y = vec_d_(1);
    marker.points[13].z = vec_d_(2);

    marker.points[14].x = vec_d_(0);
    marker.points[14].y = vec_d_(1);
    marker.points[14].z = vec_d_(2);

    marker.points[15].x = vec_a_(0);
    marker.points[15].y = vec_a_(1);
    marker.points[15].z = vec_a_(2);

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    return marker;
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
  bool srvCallback_GetFieldOfView(cob_3d_mapping_msgs::GetFieldOfView::Request &req,
                                  cob_3d_mapping_msgs::GetFieldOfView::Response &res)
  {
    boost::mutex::scoped_lock l(m_mutex);

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
  }

protected:
  ros::NodeHandle n_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;
  ros::Publisher fov_marker_pub_;
  ros::ServiceServer get_fov_srv_;
  ros::Time stamp_;

  TransformListener tf_listener_;
  dynamic_reconfigure::Server<cob_3d_mapping_common::field_of_viewConfig> config_server_;

  double sensor_fov_hor_;       /// horizontal angle of sensor
  double sensor_fov_ver_;       /// vertical angle of sensor
  double sensor_max_range_;     /// maximum range of sensor
  std::string camera_frame_;

  Eigen::Vector3d vec_a_;       /// part of view frustum
  Eigen::Vector3d vec_b_;       /// part of view frustum
  Eigen::Vector3d vec_c_;       /// part of view frustum
  Eigen::Vector3d vec_d_;       /// part of view frustum

  tf::Point n_up_;              /// field of view vector
  tf::Point n_down_;            /// field of view vector
  tf::Point n_right_;           /// field of view vector
  tf::Point n_left_;            /// field of view vector

  tf::Point n_up_t_;              /// transformed field of view vector
  tf::Point n_down_t_;            /// transformed field of view vector
  tf::Point n_right_t_;           /// transformed field of view vector
  tf::Point n_left_t_;            /// transformed field of view vector
  tf::Point n_origin_t_;

  boost::mutex m_mutex;         /// mutex to synchronize compution with dynamic reconfigure

};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "field_of_view");

  FieldOfView fov;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    //fov.transformNormals();
    loop_rate.sleep();
  }
}



