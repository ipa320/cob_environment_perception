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
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: registration
 *
 * \author
 *  Author: goa-jh
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: Nov 21, 2011
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

template<typename Point>
bool Registration_RGBDSLAM<Point>::compute_transformation() {
  if(pub_img_.getNumSubscribers()<1||pub_img_depth_.getNumSubscribers()<1||pub_camera_info_.getNumSubscribers()<1||pub_cloud_.getNumSubscribers()<1) {
    sleep(5); //give it its time
  }
  if(pub_img_.getNumSubscribers()<1||pub_img_depth_.getNumSubscribers()<1||pub_camera_info_.getNumSubscribers()<1||pub_cloud_.getNumSubscribers()<1) {
    ROS_ERROR("please start RGBDSlam");
    if(pub_img_.getNumSubscribers()<1)
      ROS_WARN("no subscriber for %s", pub_img_.getTopic().c_str());
    if(pub_img_depth_.getNumSubscribers()<1)
      ROS_WARN("no subscriber for %s", pub_img_depth_.getTopic().c_str());
    if(pub_camera_info_.getNumSubscribers()<1)
      ROS_WARN("no subscriber for %s", pub_camera_info_.getTopic().c_str());
    if(pub_cloud_.getNumSubscribers()<1)
      ROS_WARN("no subscriber for %s", pub_cloud_.getTopic().c_str());

    return false;
  }

  got_tranf_=false;

  ros::Time stamp = ros::Time();

  {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*this->input_org_, cloud);
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "/openni_camera";
    pub_cloud_.publish(cloud);
  }

  {
    cv_bridge::CvImage img;
    img.header.stamp = stamp;
    img.header.frame_id = "/openni_camera";
    img.encoding = "rgb8";
    img.image = *this->input_image_;

    pub_img_.publish(img.toImageMsg());
  }

  if(this->input_depth_image_)
  {
    cv_bridge::CvImage img;
    img.header.stamp = stamp;
    img.header.frame_id = "/openni_camera";
    img.encoding = "mono16";
    img.image = *this->input_depth_image_;

    pub_img_depth_.publish(img.toImageMsg());
  }
  else
    ROS_ERROR("missing depth image... skipping");

  {
    sensor_msgs::CameraInfo ci;
    ci.header.stamp = stamp;
    ci.header.frame_id = "/openni_camera";
    pub_camera_info_.publish(ci);
  }

  //10 times a 1 sec
  int tries=0;
  while(tries<20*2) {

    ++tries;
    usleep(1000*500);
    ros::spinOnce();

    if(got_tranf_)
      break;
  }

  if(!got_tranf_) {
    ROS_WARN("RGBDSlam timed out!");
    return false;
  }

  ROS_INFO("RGBDSlam success...");

  pcl::PointCloud<Point> result;
  pcl::transformPointCloud(*this->input_org_,result,this->transformation_);
  this->register_ += result;

  return true;
}
