/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: registration
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-jh
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Nov 11, 2011
 * ToDo:
 *
 *
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



//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <fstream>


// ROS includes
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <pcl/io/pcd_io.h>
#include "registration/RegistrationPCD.h"
#include "registration/preprocessing/kinect_error.h"

#include <registration/registration_icp.h>


using namespace tf;

//####################
//#### node class ####
class FilterNode
{
  typedef pcl::PointXYZRGB Point;

public:
  // Constructor
  FilterNode()
    {
    onInit();
    }


  // Destructor
  ~FilterNode()
  {
  }

  /**
   * @brief initializes parameters
   *
   * initializes parameters
   *
   * @return nothing
   */
  void
  onInit()
  {
    //n_.param("aggregate_point_map/icp_max_iterations" ,icp_max_iterations ,50);

    point_cloud_pub_ = n_.advertise<pcl::PointCloud<Point> >("result_pc",1);
    point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &FilterNode::pointCloudSubCallback, this);

    filter_ser_ = n_.advertiseService("filter_process", &FilterNode::filterService, this);
  }

  /**
   * @brief callback for point cloud subroutine
   *
   * callback for point cloud subroutine which stores the point cloud
   * for further calculation
   *
   * @param pc_in  new point cloud
   *
   * @return nothing
   */
  void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    ROS_INFO("filter");

    pcl::PointCloud<Point> pc;
    preprocessing::KinectErrorGenerator<Point> filter;

    filter.setInputCloud(pc_in);
    filter.filter(pc);

    pc.header.frame_id="/head_cam3d_frame";
    point_cloud_pub_.publish(pc);
  }

  bool filterService(registration::RegistrationPCD::Request  &req,
                       registration::RegistrationPCD::Response &res )
  {
    ROS_INFO("filter...");

    pcl::PointCloud<Point> pc;
    pcl::io::loadPCDFile(req.pcd_fn, pc);

    preprocessing::KinectErrorGenerator<Point> filter;
    filter.setInputCloud(pc.makeShared());
    filter.filter(pc);

    pcl::io::savePCDFileASCII(req.img_fn, pc);
  }


  ros::NodeHandle n_;
  ros::Time stamp_;


protected:

  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;
  ros::ServiceServer filter_ser_;

};


int main(int argc, char **argv) {
  setVerbosityLevel(pcl::console::L_DEBUG);

  ros::init(argc, argv, "filter");

  FilterNode tn;

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}


