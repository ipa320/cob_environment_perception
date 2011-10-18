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
 * ROS stack name: cob_vision
 * ROS package name: cob_vision_ipa_utils
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Aug 2010
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
#ifndef CPC_VISUALIZER_H_
#define CPC_VISUALIZER_H_

//#include <pcl_visualization/pcl_visualizer.h>
#include <cob_vision_ipa_utils/cpc_point.h>

#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>


class CPCVisualizer : public pcl_ros::PCLNodelet
{
private:
	ros::NodeHandle n_; ///< ROS node handle

	ros::Subscriber topicSub_coloredPointCloud_;
	ros::Subscriber topicSub_filteredPointCloud_;

	int feature_ctr_;

public:
	CPCVisualizer();

    virtual ~CPCVisualizer();

    virtual void onInit();

    void initNode();

    void runVis();

    void topicCallback_ColoredPointCloud(const pcl::PointCloud<CPCPoint>::ConstPtr& cloud);
    void topicCallback_FilteredPointCloud(const pcl::PointCloud<CPCPoint>::ConstPtr& cloud);

	//pcl_visualization::PCLVisualizer visualizer_;

};


#endif
