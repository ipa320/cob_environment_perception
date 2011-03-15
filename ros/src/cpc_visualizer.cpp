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

#include <cob_vision_ipa_utils/cpc_visualizer.h>


CPCVisualizer::CPCVisualizer()
	//:visualizer_("CPC"),
	// feature_ctr_(0)
{
}

CPCVisualizer::~CPCVisualizer()
{
}

void CPCVisualizer::onInit()
{
	n_ = getNodeHandle();
	initNode();
}

void CPCVisualizer::initNode()
{
	topicSub_coloredPointCloud_ = n_.subscribe("point_cloud", 1, &CPCVisualizer::topicCallback_ColoredPointCloud, this);
	topicSub_filteredPointCloud_ = n_.subscribe("point_cloud_filtered", 1, &CPCVisualizer::topicCallback_FilteredPointCloud, this);
	//visualizer_.setBackgroundColor (1, 1, 1);
	runVis();
}

void CPCVisualizer::runVis()
{
	//TODO: mutex lock
    ros::Rate r(10);
    while(n_.ok())
    {
    	ros::spinOnce();
    	//visualizer_.spinOnce();
    	r.sleep();
    }
}

void CPCVisualizer::topicCallback_ColoredPointCloud(const pcl::PointCloud<CPCPoint>::ConstPtr& cloud)
{
	//TODO: mutex lock
	std::cout << "callback" << std::endl;
	//vis_.interrupt();
	//boost::this_thread::interruption_point();
	//TODO: remove spheres
	//visualizer_.removePointCloud ("cloud_rgb");
	/*for(int i = 0; i<feature_ctr_; i++)
	{
		std::string name;
		std::stringstream out;
		out << i;
		name = out.str();
		visualizer_.removeActorFromRenderer(name);
	}*/
	//pcl_visualization::PointCloudColorHandlerRGBField<CPCPoint> handler (*cloud);
	//visualizer_.addPointCloud (*cloud, handler, "cloud_rgb");

	/*for (unsigned int i = 0; i<(*cloud).points.size(); i++)
	{
		if((*cloud).points[i].isFeature)
		{
			std::string name;
			std::stringstream out;
			out << feature_ctr_;
			name = out.str();
			visualizer_.addSphere((*cloud).points[i], 0.01, 1, 0, 0, name);
			feature_ctr_++;
		}
	}*/
	//vis_ = boost::thread(boost::bind(&CPCVisualizer::runVis, this));
}

void CPCVisualizer::topicCallback_FilteredPointCloud(const pcl::PointCloud<CPCPoint>::ConstPtr& cloud)
{
	//TODO: mutex lock
	std::cout << "callback2" << std::endl;

	//visualizer_.removePointCloud ("cloud_filtered");

	//pcl_visualization::PointCloudColorHandlerCustom<CPCPoint> handler (*cloud, 255, 0, 0);
	//visualizer_.addPointCloud (*cloud, handler, "cloud_filtered");

}

PLUGINLIB_DECLARE_CLASS(cob_env_model, CPCVisualizer, CPCVisualizer, nodelet::Nodelet);


