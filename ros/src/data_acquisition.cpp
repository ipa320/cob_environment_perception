
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
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by:
 *
 * Date of creation: 04/2010
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
#include <fstream>

// ROS includes
#include <ros/ros.h>

#include <cv_bridge/CvBridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cob_vision_utils/GlobalDefines.h>

#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>

// ROS message includes
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cob_srvs/GetColoredPointCloud.h>
#include <cob_srvs/GetPlatformPosition.h>
#include <cob_srvs/GetTransformCamera2Base.h>
#include <cob_srvs/Trigger.h>

using namespace message_filters;

typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SharedSyncPolicy;

//####################
//#### node class ####
class CobDataAcquisitionNode
{

    public:

        // Constructor
		CobDataAcquisitionNode()
			: image_transport_(n),
			  shared_sub_sync_(SharedSyncPolicy(3)),
			  right_color_image_8U3_(0),
			  xyz_image_32F3_(0),
			  grey_image_32F1_(0),
			  robot_pose_(3),
				transformation_camera2base_(4,4)
		{
			init();
		    //topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
		    //topicSub_coloredPointCloud = n.subscribe("/sensor_fusion/ColoredPointCloud", 1, &CobEnvModelNode::topicCallback_coloredPointCloud, this);
		    srv_client_platform_position_ = n.serviceClient<cob_srvs::GetPlatformPosition>("/get_platform_position");
		    srv_client_transform_camera2base_ = n.serviceClient<cob_srvs::GetTransformCamera2Base>("/transform_camera2base");
		    //srv_server_trigger_ = n.advertiseService("UpdateEnvModel", &CobDataAcquisitionNode::srvCallback_UpdateEnvModel, this);

		}


        // Destructor
        ~CobDataAcquisitionNode()
        {
    		if (right_color_image_8U3_) cvReleaseImage(&right_color_image_8U3_);
    		if (xyz_image_32F3_) cvReleaseImage(&xyz_image_32F3_);
    		if (grey_image_32F1_) cvReleaseImage(&grey_image_32F1_);
        }

        unsigned long init()
        {
			shared_sub_sync_.connectInput(right_color_camera_image_sub_, tof_camera_xyz_image_sub_, tof_camera_grey_image_sub_);
			shared_sub_sync_.registerCallback(boost::bind(&CobDataAcquisitionNode::sharedModeSrvCallback, this, _1, _2, _3));
			right_color_camera_image_sub_.subscribe(image_transport_, "right/image_color", 1);
			tof_camera_xyz_image_sub_.subscribe(image_transport_, "image_xyz", 1);
			tof_camera_grey_image_sub_.subscribe(image_transport_, "image_grey", 1);
        	return ipa_Utils::RET_OK;
        }

    	unsigned long getRobotPose()
    	{
    		if(srv_client_platform_position_.call(platform_position_srv_))
    		{
    			ROS_INFO("[env_model_node] Platform position service called [OK].");
    		}
    		else
    		{
    			ROS_ERROR("[env_model_node] Platform position service called [FAILED].");
    		}

    		ROS_INFO("[env_model_node] Platform position (x,y,theta): %f, %f, %f",
    				platform_position_srv_.response.platform_pose.x, platform_position_srv_.response.platform_pose.y,
    				platform_position_srv_.response.platform_pose.theta);

    		robot_pose_(1) = platform_position_srv_.response.platform_pose.x;
    		robot_pose_(2) = platform_position_srv_.response.platform_pose.y;
    		robot_pose_(3) = platform_position_srv_.response.platform_pose.theta;
    		return ipa_Utils::RET_OK;
    	}

    	unsigned long getTransformationCam2Base()
    	{
    		if(srv_client_transform_camera2base_.call(transform_camera2base_srv_))
    		{
    			ROS_INFO("[env_model_node] Transformation Camera2Base service called [OK].");
    		}
    		else
    		{
    			ROS_ERROR("[env_model_node] Transformation Camera2Base service called [FAILED].");
    		}
    		ROS_INFO("[env_model_node] Transformation Camera2Base (x,y,z,roll,pitch,yaw): %f, %f, %f, %f, %f, %f",
    					transform_camera2base_srv_.response.transformation.x, transform_camera2base_srv_.response.transformation.y,
    					transform_camera2base_srv_.response.transformation.z, transform_camera2base_srv_.response.transformation.roll,
    					transform_camera2base_srv_.response.transformation.pitch, transform_camera2base_srv_.response.transformation.yaw);
    		double yaw = transform_camera2base_srv_.response.transformation.yaw;
    		double pitch = transform_camera2base_srv_.response.transformation.pitch;
    		double roll = transform_camera2base_srv_.response.transformation.roll;

    		transformation_camera2base_(1,1) = cos(yaw)*cos(pitch);
    		transformation_camera2base_(1,2) = cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*sin(roll);
    		transformation_camera2base_(1,3) = cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
    		transformation_camera2base_(1,4) = transform_camera2base_srv_.response.transformation.x;
    		transformation_camera2base_(2,1) = sin(yaw)*cos(pitch);
    		transformation_camera2base_(2,2) = sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*sin(roll);
    		transformation_camera2base_(2,3) = sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
    		transformation_camera2base_(2,4) = transform_camera2base_srv_.response.transformation.y;
    		transformation_camera2base_(3,1) = -sin(pitch);
    		transformation_camera2base_(3,2) = cos(pitch)*sin(roll);
    		transformation_camera2base_(3,3) = cos(pitch)*cos(roll);
    		transformation_camera2base_(3,4) = transform_camera2base_srv_.response.transformation.z;
    		transformation_camera2base_(4,1) = 0;
    		transformation_camera2base_(4,2) = 0;
    		transformation_camera2base_(4,3) = 0;
    		transformation_camera2base_(4,4) = 1;
    		return ipa_Utils::RET_OK;
    	}

    	void sharedModeSrvCallback(const sensor_msgs::ImageConstPtr& right_camera_data,
    			const sensor_msgs::ImageConstPtr& tof_camera_xyz_data,
    			const sensor_msgs::ImageConstPtr& tof_camera_grey_data)
    	{
<<<<<<< HEAD:cob_vision/cob_env_model/ros/src/data_acquisition.cpp
    		ROS_INFO("[data_acquisition] sharedModeSrvCallback");
=======
    		ROS_INFO("[data_acquisition] receiving image data");
>>>>>>> 386457fbdb66c2a1a510e8693d7b2a42355c3f6f:cob_vision/cob_env_model/ros/src/data_acquisition.cpp
    		// Convert ROS image messages to openCV IplImages
    		if(right_color_image_8U3_) cvReleaseImage(&right_color_image_8U3_);
    		if(xyz_image_32F3_) cvReleaseImage(&xyz_image_32F3_);
    		if(grey_image_32F1_) cvReleaseImage(&grey_image_32F1_);
    		try
    		{
    			right_color_image_8U3_ = cvCloneImage(cv_bridge_0_.imgMsgToCv(right_camera_data, "passthrough"));
    			xyz_image_32F3_ = cvCloneImage(cv_bridge_1_.imgMsgToCv(tof_camera_xyz_data, "passthrough"));
    			grey_image_32F1_ = cvCloneImage(cv_bridge_2_.imgMsgToCv(tof_camera_grey_data, "passthrough"));
    		}
			catch (sensor_msgs::CvBridgeException& e)
			{
			  ROS_ERROR("[data_acquisition] Could not convert images by cv_bridge.");
			}
    		ROS_INFO("OK");

    	}

    	unsigned long save()
    	{
    		char counterBuffer [50];
    		std::string imageFileName;
    		std::string filetype = "bmp";
    		static unsigned int counter=1;
    		sprintf(counterBuffer, "%04d", counter);
    	    if (right_color_image_8U3_)
    	    {
				imageFileName = std::string("ColorCamRGB_8U3_0_") + counterBuffer + "." + filetype;
				std::cout << "\t ... Image of right color camera saved to " + imageFileName << std::endl;
				cvSaveImage(imageFileName.c_str() , right_color_image_8U3_);
    	    }
    	    if (xyz_image_32F3_)
    	    {
				imageFileName = std::string("RangeCamCoordinate_32F3_0_") + counterBuffer + ".xml";
				std::cout << "\t ... TOF coordinate Image saved to " + imageFileName << std::endl;
				cvSave(imageFileName.c_str() , xyz_image_32F3_);
    	    }
    	    if (grey_image_32F1_)
    	    {
				imageFileName = std::string("RangeCamIntensity_32F1_0_") + counterBuffer + ".xml";
				std::cout << "\t ... TOF grey Image saved to " + imageFileName << std::endl;
				cvSave(imageFileName.c_str() , grey_image_32F1_);
    	    }
    	    std::string pathFileName = std::string("path_and_trafo") + counterBuffer + ".txt";
    	    std::ofstream f(pathFileName.c_str(), std::fstream::out);
    	    f << robot_pose_ << "\n";
    	    f << transformation_camera2base_;
    	    f.close();
    	    counter++;
    	}

		// create a handle for this node, initialize node
		ros::NodeHandle n;

		ros::ServiceClient srv_client_platform_position_;
		ros::ServiceClient srv_client_transform_camera2base_;
		ros::ServiceServer srv_server_trigger_;

		cob_srvs::GetPlatformPosition platform_position_srv_;
		cob_srvs::GetTransformCamera2Base transform_camera2base_srv_;

		image_transport::ImageTransport image_transport_;
		image_transport::SubscriberFilter right_color_camera_image_sub_;	///< Right color camera image topic
		image_transport::SubscriberFilter tof_camera_xyz_image_sub_;	///< Tof camera xyz image topic
		image_transport::SubscriberFilter tof_camera_grey_image_sub_;	///< Tof camera intensity image topic

		message_filters::Synchronizer<SharedSyncPolicy> shared_sub_sync_;

		sensor_msgs::Image color_image_msg_;
		sensor_msgs::Image xyz_image_msg_;
		sensor_msgs::Image confidence_mask_msg_;

		IplImage* right_color_image_8U3_;	///< Received color image of the right camera
		IplImage* xyz_image_32F3_;	///< Received point cloud form tof sensor
		IplImage* grey_image_32F1_;	///< Received gray values from tof sensor

		sensor_msgs::CvBridge cv_bridge_0_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_1_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_2_; ///< Converts ROS image messages to openCV IplImages


		MatrixWrapper::ColumnVector robot_pose_;
		MatrixWrapper::Matrix transformation_camera2base_;
};

int main(int argc, char** argv)
{
    // initialize ROS, specify name of node
    ros::init(argc, argv, "cobDataAcquisitionNode");

    CobDataAcquisitionNode cobDataAcquisitionNode;

	bool first = true;
	int i=0;

    ros::Rate r(1);
    while(cobDataAcquisitionNode.n.ok())
    {
        ros::spinOnce();
        cobDataAcquisitionNode.getTransformationCam2Base();
        cobDataAcquisitionNode.save();
        //cobDataAcquisitionNode.getRobotPose();
		/*cobEnvModelNode.detectFeatures();
		if(first)
			cobEnvModelNode.initializeFilter();
		else
			cobEnvModelNode.updateFilter();
		first = false;*/
		//i++;
    	//r.sleep();
    //}

    return 0;
}
