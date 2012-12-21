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
 * ROS stack name: cob_driver
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: September 2011
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
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
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>

#include <sensor_msgs/PointCloud2.h>


using namespace std;

class As_Node
{
protected:
  ros::NodeHandle n_;
public:
  As_Node(): n_("~") {
  }

  virtual ~As_Node() {}

  virtual void onInit()=0;

  void start() {

  }
};

class As_Nodelet : public  pcl_ros::PCLNodelet
{
protected:
  ros::NodeHandle n_;
public:
  As_Nodelet() {
  }

  virtual ~As_Nodelet() {}

  void start() {
    PCLNodelet::onInit();
    n_ = getNodeHandle();
  }
};

template <typename Parent>
class TransformPointCloud : public Parent
{

  public:
    string cloud_topic_sub_;
    string cloud_topic_pub_;
    string target_frame_;

    tf::TransformListener tf_listener_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    TransformPointCloud()
    {
    }

    void onInit() {
      cloud_topic_sub_="point_cloud2";
      cloud_topic_pub_="point_cloud2_trans";
      target_frame_ = "/base_link";

      ros::NodeHandle *n=&(this->n_);
      pub_ = n->advertise<sensor_msgs::PointCloud2>(cloud_topic_pub_,1);
      sub_ = this->n_.subscribe (cloud_topic_sub_, 1,  &TransformPointCloud::cloud_cb, this);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
      sensor_msgs::PointCloud2 cloud_out;
      pcl_ros::transformPointCloud (target_frame_, *cloud, cloud_out, tf_listener_);
      pub_.publish(cloud_out);
    }

};

#ifdef COMPILE_NODELET

typedef TransformPointCloud<As_Nodelet> TransformPointCloudN;

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_tools, TransformPointCloud, TransformPointCloudN, nodelet::Nodelet)

#else

int main(int argc, char **argv) {
  ros::init (argc, argv, "transform_pointcloud", ros::init_options::AnonymousName);

  TransformPointCloud<As_Node> sn("transform_pointcloud");
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
