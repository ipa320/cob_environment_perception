
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
#include <cob3_msgs/ColoredPointCloud.h>

#include <cob3_srvs/GetColoredPointCloud.h>
#include <cob3_srvs/Trigger.h>


// external includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cob_sensor_fusion/ColoredPointCloud.h"
#include "cob_env_model/CuiEnvReconstruction.h"


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
        void topicCallback_coloredPointCloud(const cob3_msgs::ColoredPointCloud::ConstPtr& msg);


        void UpdateColoredPointCloud(const cob3_msgs::ColoredPointCloud::ConstPtr& msg);

        bool srvCallback_UpdateEnvModel(cob3_srvs::Trigger::Request &req,
										cob3_srvs::Trigger::Response &res);


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

		cob3_srvs::GetColoredPointCloud m_ColoredPointCloudSrv;

};


CobEnvModelNode::CobEnvModelNode()
{
    //topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
    topicSub_coloredPointCloud = n.subscribe("coloredPointCloud", 1, &CobEnvModelNode::topicCallback_coloredPointCloud, this);
    srvClient_ColoredPointCloud = n.serviceClient<cob3_srvs::GetColoredPointCloud>("GetColoredPointCloud");
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
}

void CobEnvModelNode::topicCallback_coloredPointCloud(const cob3_msgs::ColoredPointCloud::ConstPtr& msg)
{
	tf::StampedTransform transform;
	try
	{
		listener.lookupTransform("/cam", "/base", ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	//transform in Matrix m_TransformCam2Base umwandeln

	//UpdateColoredPointCloud(msg);
	//ColoredPointCloud Objekt füllen aus msg
	//EnvReconstruction::UpdateFilter;
    ROS_INFO("this is topicCallback_demoSubscribe");
}

void CobEnvModelNode::UpdateColoredPointCloud(const cob3_msgs::ColoredPointCloud::ConstPtr& msg)
{
	IplImage* colorImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->colorImage)));
	m_ColoredPointCloud.SetColorImage(colorImage);
	IplImage* xyzImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->pointCloud)));
	m_ColoredPointCloud.SetXYZImage(xyzImage);
	m_GreyImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->greyImage)));
}

bool CobEnvModelNode::srvCallback_UpdateEnvModel(cob3_srvs::Trigger::Request &req,
								cob3_srvs::Trigger::Response &res)
{
	srvClient_ColoredPointCloud.call(m_ColoredPointCloudSrv);
	//warten?
	//UpdateColoredPointCloud(&(m_ColoredPointCloudSrv.response.ColoredPointCloud));
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

