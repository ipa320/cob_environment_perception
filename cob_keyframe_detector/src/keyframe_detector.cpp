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
* \date Date of creation: 09/2011
*
* \brief
* Triggers key frames dependending on robot movement
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
#include <tf_conversions/tf_eigen.h>

// ROS message includes
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/server.h>
#include <cob_keyframe_detector/keyframe_detectorConfig.h>

using namespace tf;

//####################
//#### node class ####
class KeyframeDetector
{

public:
  // Constructor
  KeyframeDetector()
  : first_(true),
    trigger_always_(false)
  {
    config_server_.setCallback(boost::bind(&KeyframeDetector::dynReconfCallback, this, _1, _2));
    camera_info_sub_ = n_.subscribe("camera_info", 1, &KeyframeDetector::cameraInfoSubCallback, this);
    transform_sub_ = n_.subscribe("/tf", 1, &KeyframeDetector::transformSubCallback, this);
    keyframe_trigger_client_ = n_.serviceClient<std_srvs::Trigger>("trigger_keyframe");
  }


  // Destructor
  ~KeyframeDetector()
  {
    /// void
  }

  void dynReconfCallback(cob_keyframe_detector::keyframe_detectorConfig &config, uint32_t level)
  {
    r_limit_ = config.r_limit;
    p_limit_ = config.p_limit;
    y_limit_ = config.y_limit;
    distance_limit_ = config.distance_limit;
    trigger_always_ = config.trigger_always;
  }

  void
  cameraInfoSubCallback(sensor_msgs::CameraInfo::ConstPtr pc_in)
  {
    frame_id_ = pc_in->header.frame_id;
  }

  void
  transformSubCallback(const tf::tfMessageConstPtr& msg)
  {
    boost::mutex::scoped_lock l(m_mutex_pointCloudSubCallback);

    if(frame_id_.size() < 1) {
      ROS_WARN("KeyframeDetector: Frame id is missing, no key frame detected");
      return;
    }
    StampedTransform transform;
    try
    {
      tf_listener_.waitForTransform("/map", frame_id_, ros::Time(0), ros::Duration(0.1));
      tf_listener_.lookupTransform("/map", frame_id_, ros::Time(0), transform);
      //KDL::Frame frame_KDL, frame_KDL_old;
      Eigen::Affine3d trf_eigen, trf_eigen_old;
      tf::transformTFToEigen(transform, trf_eigen);
      tf::transformTFToEigen(transform_old_, trf_eigen_old);
      Eigen::Vector3d rpy = trf_eigen.rotation().eulerAngles(2,1,0);
      Eigen::Vector3d rpy_old = trf_eigen_old.rotation().eulerAngles(2,1,0);

      if(trigger_always_ || first_ || fabs(rpy(0) - rpy_old(0)) > r_limit_ || fabs(rpy(1) - rpy_old(1)) > p_limit_ || fabs(rpy(2) - rpy_old(2)) > y_limit_ ||
          transform.getOrigin().distance(transform_old_.getOrigin()) > distance_limit_)
      {
        if(triggerKeyFrame()) {
          transform_old_ = transform;
          first_ = false;
        }
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[keyframe_detector] : %s",ex.what());
    }
  }

  bool triggerKeyFrame() {
    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;
    if(keyframe_trigger_client_.call(req,res))
    {
      ROS_DEBUG("KeyFrame service called [OK].");
    }
    else
    {
      ROS_WARN("KeyFrame service called [FAILED].", res.success);
      return false;
    }
    return res.success;
  }


protected:
  ros::NodeHandle n_;
  ros::Time stamp_;
  ros::Subscriber camera_info_sub_, transform_sub_;
  TransformListener tf_listener_;
  ros::ServiceClient keyframe_trigger_client_;
  dynamic_reconfigure::Server<cob_keyframe_detector::keyframe_detectorConfig> config_server_;

  bool first_;
  bool trigger_always_;

  StampedTransform transform_old_;

  std::string frame_id_;

  double y_limit_;
  double distance_limit_;
  double r_limit_;
  double p_limit_;

  boost::mutex m_mutex_pointCloudSubCallback;

};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "keyframe_detector");

  KeyframeDetector kd;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep();
  }
}
