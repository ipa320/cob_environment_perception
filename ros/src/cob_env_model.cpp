
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
 * Date of creation: 03/2010
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
//--

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "cv_bridge/CvBridge.h"


// ROS message includes
#include <std_msgs/String.h>
#include <cob_msgs/ColoredPointCloud.h>

#include <cob_srvs/GetColoredPointCloud.h>
#include <cob_srvs/Trigger.h>


// external includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cob_sensor_fusion/ColoredPointCloud.h"
//#include "cob_env_model/CuiEnvReconstruction.h"


//####################
//#### node class ####
class CobEnvModelNode
{

    public:

        // Constructor
		CobEnvModelNode();

        
        // Destructor
        ~CobEnvModelNode()
        {
        	/// void
        }

        unsigned long Init();

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_coloredPointCloud(const cob_msgs::ColoredPointCloud::ConstPtr& msg);


        void UpdateColoredPointCloud(const cob_msgs::ColoredPointCloud::ConstPtr& msg);

        bool srvCallback_UpdateEnvModel(cob_srvs::Trigger::Request &req,
										cob_srvs::Trigger::Response &res);

        void callColoredPointCloudService();

		// create a handle for this node, initialize node
		ros::NodeHandle n;

		// topics to publish
		ros::Publisher topicPub_demoPublish;

		// topics to subscribe, callback is called for new messages arriving
		ros::Subscriber topicSub_coloredPointCloud;
		tf::TransformListener listener;

		ros::ServiceServer srvServer_Trigger;
		ros::ServiceClient srvClient_ColoredPointCloud;

		//CuiEnvReconstruction m_EnvReconstruction;

    protected:

		//Matrix m_transformCam2Base;
		ipa_SensorFusion::ColoredPointCloud m_ColoredPointCloud;
		IplImage* m_GreyImage;
		sensor_msgs::CvBridge m_Bridge;

		cob_srvs::GetColoredPointCloud m_ColoredPointCloudSrv;

};


CobEnvModelNode::CobEnvModelNode()
{
    //topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
    topicSub_coloredPointCloud = n.subscribe("/sensor_fusion/ColoredPointCloud", 1, &CobEnvModelNode::topicCallback_coloredPointCloud, this);
    srvClient_ColoredPointCloud = n.serviceClient<cob_srvs::GetColoredPointCloud>("/sensor_fusion/ColoredPointCloud");
    srvServer_Trigger = n.advertiseService("UpdateEnvModel", &CobEnvModelNode::srvCallback_UpdateEnvModel, this);
}

unsigned long CobEnvModelNode::Init()
{
	/*if (m_EnvReconstruction.Init() & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - Main:" << std::endl;
		std::cerr << "\t ... Error while initializing EnvReconstruction.\n";
		return -1;
	}*/
	return 1;
}

void CobEnvModelNode::topicCallback_coloredPointCloud(const cob_msgs::ColoredPointCloud::ConstPtr& msg)
{
	/*tf::StampedTransform transform;
	try
	{
		listener.lookupTransform("/cam", "/base", ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}*/
	//transform in Matrix m_TransformCam2Base umwandeln


	//UpdateColoredPointCloud(msg);
	//ColoredPointCloud Objekt füllen aus msg
	//EnvReconstruction::UpdateFilter;
    ROS_INFO("this is topicCallback_demoSubscribe");
}

void CobEnvModelNode::UpdateColoredPointCloud(const cob_msgs::ColoredPointCloud::ConstPtr& msg)
{
	IplImage* colorImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->colorImage)));
	m_ColoredPointCloud.SetColorImage(colorImage);
	IplImage* xyzImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->pointCloud)));
	m_ColoredPointCloud.SetXYZImage(xyzImage);
	m_GreyImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->greyImage)));
}

bool CobEnvModelNode::srvCallback_UpdateEnvModel(cob_srvs::Trigger::Request &req,
								cob_srvs::Trigger::Response &res)
{
	srvClient_ColoredPointCloud.call(m_ColoredPointCloudSrv);

	return true;

	//warten?
	//UpdateColoredPointCloud(&(m_ColoredPointCloudSrv.response.ColoredPointCloud));
}

void CobEnvModelNode::callColoredPointCloudService()
{
	ROS_INFO("[env_model_node] Colored point cloud service call.");
	if(srvClient_ColoredPointCloud.call(m_ColoredPointCloudSrv))
	{
		ROS_INFO("[env_model_node] Colored point cloud service called [OK].");
	}
	else
	{
		ROS_ERROR("[env_model_node] Colored point cloud service called [FAILED].");
	}
	sensor_msgs::CvBridge cv_bridge_0_; ///< Converts ROS image messages to openCV IplImages
	sensor_msgs::CvBridge cv_bridge_1_; ///< Converts ROS image messages to openCV IplImages
	sensor_msgs::CvBridge cv_bridge_2_; ///< Converts ROS image messages to openCV IplImages
	IplImage* color_image_8U3;
	IplImage* xyz_image_32F3;
	IplImage* grey_image_32F1;
	sensor_msgs::ImageConstPtr colorImage(&(m_ColoredPointCloudSrv.response.colorImage));
	sensor_msgs::ImageConstPtr xyzImage(&(m_ColoredPointCloudSrv.response.xyzImage));
	sensor_msgs::ImageConstPtr greyImage(&(m_ColoredPointCloudSrv.response.confidenceMask));
    try
    {
      color_image_8U3 = cvCloneImage(cv_bridge_0_.imgMsgToCv(colorImage, "passthrough"));
      xyz_image_32F3 = cvCloneImage(cv_bridge_1_.imgMsgToCv(xyzImage, "passthrough"));
      grey_image_32F1 = cvCloneImage(cv_bridge_2_.imgMsgToCv(greyImage, "passthrough"));
      //color_image_8U3 = cvCloneImage(cv_bridge_0_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.colorImage)), "passthrough"));
      //xyz_image_32F3 = cvCloneImage(cv_bridge_0_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.xyzImage)), "passthrough"));
      //grey_image_32F1 = cvCloneImage(cv_bridge_2_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.greyImage)), "passthrough"));
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("[tof_camera_viewer] Could not convert images by cv_bridge.");
    }
		ROS_INFO("[env_model_node] Point cloud received.");
    try
    {
    	IplImage* grey_image_8U3 = cvCreateImage(cvGetSize(grey_image_32F1), IPL_DEPTH_8U, 3);
    	IplImage* xyz_image_8U3 = cvCreateImage(cvGetSize(xyz_image_32F3), IPL_DEPTH_8U, 3);

    	ipa_Utils::ConvertToShowImage(grey_image_32F1, grey_image_8U3, 1);
    	ipa_Utils::ConvertToShowImage(xyz_image_32F3, xyz_image_8U3, 1);
    	cvShowImage("grey data", grey_image_8U3);
    	cvShowImage("xyz data", xyz_image_8U3);
    	cvShowImage("color data", color_image_8U3);
    	cvWaitKey();
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("[sensor_fusion] Could not convert");
    }
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cobEnvModelNode");
    
    CobEnvModelNode cobEnvModelNode;
 
    while(cobEnvModelNode.n.ok())
    {
        ros::spinOnce();
				cobEnvModelNode.callColoredPointCloudService();
				
        /*if (cobEnvModelNode.m_EnvReconstruction.UI() & ipa_Utils::RET_FAILED)
        {
    		std::cerr << "ERROR - Main:" << std::endl;
    		std::cerr << "\t ... Error in cobEnvModelNode.m_EnvReconstruction.UI().\n";
        	break;
        }*/
    }
    
//    ros::spin();

    return 0;
}

