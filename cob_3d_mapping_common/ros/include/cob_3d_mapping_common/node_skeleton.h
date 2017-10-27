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
 *  ROS package name: cob_3d_mapping_common
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
#pragma once


// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

///parent for a node
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

///parent for a nodelet
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

/*
 * USAGE:
template <typename Parent>
class MyNode : public Parent
{
	...
  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);
   
    //do your init. stuff here
  }
};

#ifdef COMPILE_NODELET

typedef MyNode<As_Nodelet> MyNode_Nodelet;

PLUGINLIB_EXPORT_CLASS(MyNode_Nodelet, nodelet::Nodelet);

#else

int main(int argc, char **argv) {
  ros::init(argc, argv, "MyNode");

  MyNode<As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
*/
