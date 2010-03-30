
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
#include <cv_bridge/CvBridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cob_sensor_fusion/ColoredPointCloud.h>
#include <model/analyticmeasurementmodel_gaussianuncertainty.h>
#include <model/systemmodel.h>


// ROS message includes
#include <std_msgs/String.h>
#include <cob_msgs/ColoredPointCloud.h>

#include <cob_srvs/GetColoredPointCloud.h>
#include <cob_srvs/Trigger.h>


// external includes



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

    	///Create and initialize the system model of the robot
    	/// @return Return code
    	unsigned long CreateSystemModel();

    	///Create and initialize the measurement model for the sensors
    	/// @return Return code
    	unsigned long CreateMeasurementModel();

    	unsigned long GetRobotPose();

    	unsigned long GetMeasurement();

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

		//ROS
		sensor_msgs::CvBridge m_Bridge;
		cob_srvs::GetColoredPointCloud m_ColoredPointCloudSrv;


		//Matrix m_transformCam2Base;
		ipa_SensorFusion::ColoredPointCloud m_ColoredPointCloud;
		IplImage* m_GreyImage;


		AbstractDetector* m_FeatureDetector;

};


CobEnvModelNode::CobEnvModelNode()
{
    //topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
    //topicSub_coloredPointCloud = n.subscribe("/sensor_fusion/ColoredPointCloud", 1, &CobEnvModelNode::topicCallback_coloredPointCloud, this);
    srvClient_ColoredPointCloud = n.serviceClient<cob_srvs::GetColoredPointCloud>("GetColoredPointCloud");
    srvServer_Trigger = n.advertiseService("UpdateEnvModel", &CobEnvModelNode::srvCallback_UpdateEnvModel, this);
}

unsigned long CobEnvModelNode::Init()
{
	m_FeatureDetector = new SURFDetector();
	((SURFDetector*)m_FeatureDetector)->Init(500);
	//m_FeatureDetector = new CalonderSTARDetector();

	CreateMeasurementModel();
	CreateSystemModel();

	return ipa_Utils::RET_OK;
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


unsigned long EnvReconstructionControlFlow::CreateMeasurementModel()
{
	BFL::ColumnVector measNoise_Mu(2);
  	measNoise_Mu(1) = 0.0;
  	measNoise_Mu(2) = 0.0;

  	BFL::SymmetricMatrix measNoise_Cov(2);
  	measNoise_Cov(1,1) = pow(0.5,2);
  	measNoise_Cov(1,2) = 0;
  	measNoise_Cov(2,1) = 0;
  	measNoise_Cov(2,2) = pow(0.5,2);
  	BFL::Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

  	/// create the model
  	ipa_BayesianFilter::MeasPDF2DLandmark *meas_pdf = new ipa_BayesianFilter::MeasPDF2DLandmark(measurement_Uncertainty);

  	m_MeasModel = new BFL::AnalyticMeasurementModelGaussianUncertainty(meas_pdf);


	return ipa_Utils::RET_OK;
}

unsigned long EnvReconstructionControlFlow::CreateSystemModel()
{
	BFL::ColumnVector sysNoise_Mu(3);
  	sysNoise_Mu(1) = 0.0;
  	sysNoise_Mu(2) = 0.0;
  	sysNoise_Mu(3) = 0.0;

  	BFL::SymmetricMatrix sysNoise_Cov(3);
  	sysNoise_Cov(1,1) = pow(0.01,2); //0.05
  	sysNoise_Cov(1,2) = 0.0;
  	sysNoise_Cov(1,3) = 0.0;
  	sysNoise_Cov(2,1) = 0.0;
  	sysNoise_Cov(2,2) = pow(0.01,2); //0.05
  	sysNoise_Cov(2,3) = 0.0;
  	sysNoise_Cov(3,1) = 0.0;
  	sysNoise_Cov(3,2) = 0.0;
  	//sysNoise_Cov(3,3) = pow(0.025,2);
  	sysNoise_Cov(3,3) = pow(0.017,2);


  	BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
  	/// create the nonlinear system model
  	ipa_BayesianFilter::SysPDF2DOdometry *sys_pdf = new ipa_BayesianFilter::SysPDF2DOdometry(system_Uncertainty);
  	m_SysModel = new BFL::SystemModel<BFL::ColumnVector>(sys_pdf);
  	return ipa_Utils::RET_OK;
}

unsigned long GetRobotPose()
{
	//TODO: ROS service call to platform
}

unsigned long GetMeasurement()
{
	srvClient_ColoredPointCloud.call(m_ColoredPointCloudSrv);
	sensor_msgs::CvBridge cv_bridge_0_; ///< Converts ROS image messages to openCV IplImages
	sensor_msgs::CvBridge cv_bridge_1_; ///< Converts ROS image messages to openCV IplImages
	sensor_msgs::CvBridge cv_bridge_2_; ///< Converts ROS image messages to openCV IplImages
	IplImage* color_image_8U3;
	IplImage* xyz_image_32F3;
	IplImage* grey_image_32F1;
    try
    {
      color_image_8U3 = cvCloneImage(cv_bridge_0_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.colorImage)), "passthrough"));
      xyz_image_32F3 = cvCloneImage(cv_bridge_0_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.xyzImage)), "passthrough"));
      grey_image_32F1 = cvCloneImage(cv_bridge_2_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.greyImage)), "passthrough"));
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("[tof_camera_viewer] Could not convert images by cv_bridge.");
    }
    try
    {
    	IplImage* grey_image_8U3 = cvCreateImage(cvGetSize(grey_image_32F1), IPL_DEPTH_8U, 3);
    	IplImage* xyz_image_8U3 = cvCreateImage(cvGetSize(xyz_image_32F3), IPL_DEPTH_8U, 3);

    	ipa_Utils::ConvertToShowImage(grey_image_32F1, grey_image_8U3, 1, 0, 10000);
    	ipa_Utils::ConvertToShowImage(xyz_image_32F3, xyz_image_8U3, 1, 0, 10000);
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

