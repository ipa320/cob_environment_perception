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
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
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

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <cob_3d_segmentation/general_segmentation.h>
#include <cob_3d_segmentation/quad_regression/quad_regression.h>


class As_Node
{
protected:
  ros::NodeHandle n_;
public:
  As_Node() {
  }

  virtual ~As_Node() {}

  virtual void onInit()=0;

  void start() {

  }
};

class As_Nodelet : public  nodelet::Nodelet
{
protected:
  ros::NodeHandle n_;
public:
  As_Nodelet() {
  }

  virtual ~As_Nodelet() {}

  void start() {
    //PCLNodelet::onInit();
    n_ = getNodeHandle();
  }
};

template <typename Point, typename PointLabel, typename Parent>
class Segmentation_Node : public Parent
{
  typedef pcl::PointCloud<Point> PointCloud;
  typedef Segmentation::Segmentation_QuadRegression<Point, PointLabel, Segmentation::QPPF::QuadRegression<2, Point, Segmentation::QPPF::CameraModel_Kinect<Point> > > TYPE_QPPF;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher  point_cloud_pub_;

  GeneralSegmentation<Point, PointLabel> *seg_;

  std::string algo_;
public:
  // Constructor
  Segmentation_Node()
  {
  }

  virtual ~Segmentation_Node()
  {}

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);
    point_cloud_sub_ = this->n_.subscribe("/camera/rgb/points", 1, &Segmentation_Node<Point, PointLabel, Parent>::pointCloudSubCallback, this);
    point_cloud_pub_ = n->advertise<PointCloud>("point_cloud_labeled", 1);

    //decide which algorithm should be used
    seg_ = NULL;

    if(n->getParam("algorithm",algo_))
    {
      if(algo_=="quad regression")
      {
        TYPE_QPPF *seg = new TYPE_QPPF();
        seg_ = seg;

        double filter;
        if(this->n_.getParam("filter",filter))
          seg->setFilter((float)filter);

        bool only_planes;
        if(this->n_.getParam("only_planes",only_planes))
          seg->setOnlyPlanes(only_planes);
      }
      else
        ROS_ERROR("%s is no valid segmentation algorithm", algo_.c_str());
    }
    else
    {
      ROS_ERROR("no valid segmentation algorithm selected");
    }
  }

  void
  pointCloudSubCallback(const boost::shared_ptr<const PointCloud>& pc_in)
  {
    ROS_DEBUG("segmentation: point cloud callback");

    if(!seg_)
      return;

    seg_->setInputCloud(pc_in);
    seg_->compute();
    if(point_cloud_pub_.getNumSubscribers()>0)
    {
      pcl::PointCloud<PointLabel> pc_out = *seg_->getOutputCloud();
      pc_out.header = pc_in->header;
      point_cloud_pub_.publish(pc_out);
    }
  }
};

#ifdef COMPILE_NODELET

typedef Segmentation_Node<pcl::PointXYZ,pcl::PointXYZRGB,As_Nodelet> Segmentation_Node_XYZ;

PLUGINLIB_EXPORT_CLASS(Segmentation_Node_XYZ, nodelet::Nodelet);

#else

int main(int argc, char **argv) {
  ros::init(argc, argv, "segmentation");

  Segmentation_Node<pcl::PointXYZ,pcl::PointXYZRGB,As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
