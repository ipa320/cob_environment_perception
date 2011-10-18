/*
 * cob_colored_point_cloud_publisher.h
 *
 *  Created on: 25.08.2010
 *      Author: goa
 */

#ifndef COB_COLORED_POINT_CLOUD_PUBLISHER_H_
#define COB_COLORED_POINT_CLOUD_PUBLISHER_H_

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
 * ROS package name: cob_point_cloud_publisher
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: August 2010
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

//##################
//#### includes ####

// standard includes

// ROS includes
#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

// ROS service includes
//--

// external includes
//--


//####################
//#### PcPublisher class ####
class PcPublisher : public nodelet::Nodelet
{
	private:
		ros::NodeHandle n_; ///< ROS node handle

		// topics to subscribe, callback is called for new messages arriving
		image_transport::ImageTransport image_transport_;       ///< Image transport instance
		image_transport::SubscriberFilter xyz_image_subscriber_;        ///< Subscribes to xyz image data
		image_transport::SubscriberFilter confidence_mask_subscriber_;       ///< Subscribes to gray image data
		image_transport::SubscriberFilter color_image_subscriber_;       ///< Subscribes to gray image data
		image_transport::SubscriberFilter feature_mask_subscriber_;       ///< Subscribes to feature data

		message_filters::TimeSynchronizer<sensor_msgs::Image,  ///< Assembles images with same timestamp
							sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> pc_sync_;

		sensor_msgs::CvBridge cv_bridge_0_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_1_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_2_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_3_; ///< Converts ROS image messages to openCV IplImages

		IplImage* c_xyz_image_32F3_;	///< Received point cloud from sensor fusion
		IplImage* c_confidence_mask_32F1_;	///< Received gray values from sensor fusion
		IplImage* c_color_image_8U3_;	///< Received color values from sensor fusion
		IplImage* c_feature_mask_8U1_;

		// topics to publish
		ros::Publisher topicPub_pointCloud_;

    public:
        // Constructor
        PcPublisher();

        // Destructor
        ~PcPublisher()
        {
        }

        void initNode();

        virtual void onInit();


        // topic callback functions
        // function will be called when a new message arrives on a topic
        void syncCallback(const sensor_msgs::Image::ConstPtr& pc_xyz_data, const sensor_msgs::Image::ConstPtr& pc_confidence_data,
        		const sensor_msgs::Image::ConstPtr& pc_color_data,const sensor_msgs::Image::ConstPtr& pc_feature_data);

};




#endif /* COB_COLORED_POINT_CLOUD_PUBLISHER_H_ */
