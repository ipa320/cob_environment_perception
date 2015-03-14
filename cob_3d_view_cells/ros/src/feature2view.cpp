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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: cob_3d_view_cells
 *
 * \author
 *  Author: Joshua Hampp
 *
 * \date Date of creation: 31.10.2014
 *
 * \brief
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

#include <cob_3d_mapping_common/node_skeleton.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <std_msgs/Int32.h>

template <typename FeaturePoint, typename Parent>
class Feature2View_Node : public Parent
{
  typedef pcl::PointCloud<FeaturePoint> PointCloud;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher  view_pub_;

public:
  // Constructor
  Feature2View_Node()
  {
  }

  virtual ~Feature2View_Node()
  {}

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);
    point_cloud_sub_ = this->n_.subscribe("features_in", 1, &Feature2View_Node<FeaturePoint, Parent>::pointCloudSubCallback, this);
    view_pub_ = n->advertise<std_msgs::Int32>("view_id", 1);

    /*double filter;
    if(this->n_.getParam("filter",filter))
      seg_.setFilter((float)filter);*/
  }

  void
  pointCloudSubCallback(const boost::shared_ptr<const PointCloud>& pc_in)
  {
    ROS_DEBUG("view cells: point cloud callback");
    
    const bool subscribers =
		(view_pub_.getNumSubscribers()>0);
	
	if(!subscribers) {
		ROS_DEBUG("view cells: no subscribers --> do nothing");
		return;
	}
  }
};

#ifdef COMPILE_NODELET

typedef Feature2View_Node<pcl::PointXYZRGB,As_Nodelet> Feature2View_Nodelet_XYZ;

PLUGINLIB_DECLARE_CLASS(cob_3d_view_cells, Feature2View_Nodelet_XYZ, nodelet::Nodelet)

#else

int main(int argc, char **argv) {
  ros::init(argc, argv, "feature2view");

  Feature2View_Node<pcl::PointXYZ,As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
