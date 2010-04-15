
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

#include <model/analyticmeasurementmodel_gaussianuncertainty.h>
#include <model/systemmodel.h>

#include <cob_sensor_fusion/ColoredPointCloud.h>
#include <cob_vision_features/SURFDetector.h>
#include <cob_vision_features/AbstractFeatureVector.h>
#include <cob_vision_slam/MeasurementModels/MeasPDF2DLandmark.h>
#include <cob_vision_slam/SystemModels/SysPDF2DOdometry.h>
#include <cob_vision_slam/SystemModels/SysPDF2DLaserNav.h>
#include <cob_vision_slam/Filters/FastSLAM.h>
#include <cob_vision_slam/Filters/FastSLAMParticle.h>
#include <cob_vision_slam/DataAssociation/KdTreeDataAssociation.h>


// ROS message includes
#include <std_msgs/String.h>
#include <cob_msgs/ColoredPointCloud.h>

#include <cob_srvs/GetColoredPointCloud.h>
#include <cob_srvs/GetPlatformPosition.h>
#include <cob_srvs/GetTransformCamera2Base.h>
#include <cob_srvs/Trigger.h>


// GUI includes
#include <cob_vision_ipa_utils/PointCloudRenderer.h>

// external includes

multimap<int,boost::shared_ptr<AbstractEKF> >* feature_map=0;

void deleter(sensor_msgs::Image* const) {}


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

        unsigned long init();

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        //void topicCallback_coloredPointCloud(const cob_msgs::ColoredPointCloud::ConstPtr& msg);


        //void UpdateColoredPointCloud(const cob_msgs::ColoredPointCloud::ConstPtr& msg);

        bool srvCallback_UpdateEnvModel(cob_srvs::Trigger::Request &req,
										cob_srvs::Trigger::Response &res);

        //void callColoredPointCloudService();

    	///Create and initialize the system model of the robot
    	/// @return Return code
    	unsigned long createSystemModel();

    	///Create and initialize the measurement model for the sensors
    	/// @return Return code
    	unsigned long createMeasurementModel();

    	///Call the service for the robot pose
    	unsigned long getRobotPose();

    	unsigned long getMeasurement();

    	unsigned long detectFeatures();

    	unsigned long calculateTransformationMatrix();

    	///Calls the service for the transformation from right color camera to
    	///manipulation robot base (which is rotated by 90° to the platform base)
    	unsigned long getTransformationCam2Base();

    	unsigned long initializeFilter();

    	unsigned long updateFilter();

    	///Transforms a feature point from tof to platform base
    	unsigned long transformCameraToBase(boost::shared_ptr<AbstractFeature> af);

		// create a handle for this node, initialize node
		ros::NodeHandle n;

		// topics to publish
		ros::Publisher topicPub_demoPublish;

		// topics to subscribe, callback is called for new messages arriving
		ros::Subscriber topicSub_coloredPointCloud_;
		tf::TransformListener transform_listener_;

		ros::ServiceServer srv_server_trigger_;
		ros::ServiceClient srv_client_colored_point_cloud_;
		ros::ServiceClient srv_client_platform_position_;
		ros::ServiceClient srv_client_transform_camera2base_;

		boost::shared_ptr<FastSLAMParticle> map_particle_;

    protected:

		//ROS
		cob_srvs::GetColoredPointCloud colored_point_cloud_srv_;
		cob_srvs::GetPlatformPosition platform_position_srv_;
		cob_srvs::GetTransformCamera2Base transform_camera2base_srv_;


		//Matrix m_transformCam2Base;


		sensor_msgs::CvBridge cv_bridge_0_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_1_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_2_; ///< Converts ROS image messages to openCV IplImages

		AbstractDetector* feature_detector_;
		ipa_SensorFusion::ColoredPointCloud colored_point_cloud_;
		BFL::ColumnVector robot_pose_;
		BFL::AnalyticMeasurementModelGaussianUncertainty* meas_model_;
		BFL::SystemModel<BFL::ColumnVector>* system_model_;
		AbstractFeatureVector* feature_vector_;
		KdTreeDataAssociation data_association_;
		FastSLAM fast_SLAM_;
		MatrixWrapper::Matrix transformation_camera2base_;
		MatrixWrapper::Matrix transformation_tof2camera_;
		MatrixWrapper::Matrix transformation_tof2base_pltf_;

};


CobEnvModelNode::CobEnvModelNode()
	: robot_pose_(3),
	  fast_SLAM_(100,1),
	  transformation_camera2base_(4,4),
	  transformation_tof2camera_(4,4),
	  transformation_tof2base_pltf_(4,4)
{
	init();
    //topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
    //topicSub_coloredPointCloud = n.subscribe("/sensor_fusion/ColoredPointCloud", 1, &CobEnvModelNode::topicCallback_coloredPointCloud, this);
    srv_client_colored_point_cloud_ = n.serviceClient<cob_srvs::GetColoredPointCloud>("get_colored_pc");
    srv_client_platform_position_ = n.serviceClient<cob_srvs::GetPlatformPosition>("/get_platform_position");
    srv_client_transform_camera2base_ = n.serviceClient<cob_srvs::GetTransformCamera2Base>("/transform_camera2base");
    srv_server_trigger_ = n.advertiseService("update_env_model", &CobEnvModelNode::srvCallback_UpdateEnvModel, this);

}

unsigned long CobEnvModelNode::init()
{
	feature_detector_ = new SURFDetector();
	((SURFDetector*)feature_detector_)->Init(/*500*/100);
	//m_FeatureDetector = new CalonderSTARDetector();

	createMeasurementModel();
	createSystemModel();

    XmlRpc::XmlRpcValue trafo_tof2cam;

    if (n.hasParam("trafo_tof2cam"))
    {
        n.getParam("trafo_tof2cam", trafo_tof2cam);
    }
    else
    {
        ROS_ERROR("Parameter trafo_tof2cam not set");
    }
    for (int j = 0; j<trafo_tof2cam.size(); j++ ) // j-th point
    {
        for (int k = 0; k<trafo_tof2cam[j].size(); k++ ) // k-th value of pos
        {
            //ROS_INFO("      pos value %d of %d = %f",k,traj_param[i][j][0].size(),(double)traj_param[i][j][0][k]);
            //ROS_INFO("      vel value %d of %d = %f",k,traj_param[i][j][1].size(),(double)traj_param[i][j][1][k]);
        	transformation_tof2camera_(j+1,k+1) = (double)trafo_tof2cam[j][k];
        }
    }

	transformation_camera2base_(1,1) = 1;
	transformation_camera2base_(1,2) = 0;
	transformation_camera2base_(1,3) = 0;
	transformation_camera2base_(1,4) = 0;
	transformation_camera2base_(2,1) = 0;
	transformation_camera2base_(2,2) = 1;
	transformation_camera2base_(2,3) = 0;
	transformation_camera2base_(2,4) = 0;
	transformation_camera2base_(3,1) = 0;
	transformation_camera2base_(3,2) = 0;
	transformation_camera2base_(3,3) = 1;
	transformation_camera2base_(3,4) = 0;
	transformation_camera2base_(4,1) = 0;
	transformation_camera2base_(4,2) = 0;
	transformation_camera2base_(4,3) = 0;
	transformation_camera2base_(4,4) = 1;

	robot_pose_(1) = 0;
	robot_pose_(2) = 0;
	robot_pose_(3) = 0;

	return ipa_Utils::RET_OK;
}

/*void CobEnvModelNode::topicCallback_coloredPointCloud(const cob_msgs::ColoredPointCloud::ConstPtr& msg)
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

void CobEnvModelNode::UpdateColoredPointCloud(const cob_msgs::ColoredPointCloud::ConstPtr& msg)
{
	IplImage* colorImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->colorImage)));
	m_ColoredPointCloud.SetColorImage(colorImage);
	IplImage* xyzImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->pointCloud)));
	m_ColoredPointCloud.SetXYZImage(xyzImage);
	m_GreyImage = m_Bridge.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(msg->greyImage)));
}*/

bool CobEnvModelNode::srvCallback_UpdateEnvModel(cob_srvs::Trigger::Request &req,
								cob_srvs::Trigger::Response &res)
{
	ROS_INFO("srv command: %d",req.command);
	if(req.command == 0)
	{
		if(getMeasurement() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not get measurement.");
			return false;
		}
		if(getRobotPose() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not get robot pose.");
			//return false;
		}
		if(getTransformationCam2Base() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not get transformation camera2base.");
			//return false;
		}
		calculateTransformationMatrix();
		if(detectFeatures() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not detect features.");
			return false;
		}
		if(initializeFilter() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not initialize filter.");
			return false;
		}
	}
	else if(req.command == 1)
	{
		if(getMeasurement() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not get measurement.");
			return false;
		}
		if(getRobotPose() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not get robot pose.");
			//return false;
		}
		if(getTransformationCam2Base() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not get transformation camera2base.");
			//return false;
		}
		calculateTransformationMatrix();
		if(detectFeatures() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not detect features.");
			return false;
		}
		if(updateFilter() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not update filter.");
			return false;
		}
	}
	else if(req.command == 2)
	{
		//TODO: send map to renderer or collision avoidance
		ROS_INFO("Stopping environment modelling.");
		n.shutdown();
	}
	res.success = 1;
	return true;
}


unsigned long CobEnvModelNode::createMeasurementModel()
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

  	meas_model_ = new BFL::AnalyticMeasurementModelGaussianUncertainty(meas_pdf);


	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::createSystemModel()
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
  	ipa_BayesianFilter::SysPDF2DLaserNav *sys_pdf = new ipa_BayesianFilter::SysPDF2DLaserNav(system_Uncertainty);
  	system_model_ = new BFL::SystemModel<BFL::ColumnVector>(sys_pdf);
  	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::getRobotPose()
{
	//TODO: ROS service call to platform
	if(srv_client_platform_position_.call(platform_position_srv_))
	{
		ROS_INFO("[env_model_node] Platform position service called [OK].");
	}
	else
	{
		ROS_ERROR("[env_model_node] Platform position service called [FAILED].");
		return ipa_Utils::RET_FAILED;
	}

	ROS_INFO("[env_model_node] Platform position (x,y,theta): %f, %f, %f",
			platform_position_srv_.response.platform_pose.x, platform_position_srv_.response.platform_pose.y,
			platform_position_srv_.response.platform_pose.theta);

	robot_pose_(1) = platform_position_srv_.response.platform_pose.x;
	robot_pose_(2) = platform_position_srv_.response.platform_pose.y;
	robot_pose_(3) = platform_position_srv_.response.platform_pose.theta;
	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::getMeasurement()
{
	ROS_INFO("[env_model_node] Colored point cloud service call.");

	if(srv_client_colored_point_cloud_.call(colored_point_cloud_srv_))
	{
		ROS_INFO("[env_model_node] Colored point cloud service called [OK].");
	}
	else
	{
		ROS_ERROR("[env_model_node] Colored point cloud service called [FAILED].");
		return ipa_Utils::RET_FAILED;
	}
	IplImage* color_image_8U3 = 0;
	IplImage* xyz_image_32F3 = 0;
	IplImage* grey_image_32F1 = 0;
	sensor_msgs::ImageConstPtr color_image(&(colored_point_cloud_srv_.response.colorImage),deleter);
	sensor_msgs::ImageConstPtr xyz_image(&(colored_point_cloud_srv_.response.xyzImage),deleter);
	sensor_msgs::ImageConstPtr grey_image(&(colored_point_cloud_srv_.response.confidenceMask),deleter);
    try
    {
      color_image_8U3 = cvCloneImage(cv_bridge_0_.imgMsgToCv(color_image, "passthrough"));
      xyz_image_32F3 = cvCloneImage(cv_bridge_1_.imgMsgToCv(xyz_image, "passthrough"));
      grey_image_32F1 = cvCloneImage(cv_bridge_2_.imgMsgToCv(grey_image, "passthrough"));
      //color_image_8U3 = cvCloneImage(cv_bridge_0_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.colorImage)), "passthrough"));
      //xyz_image_32F3 = cvCloneImage(cv_bridge_0_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.xyzImage)), "passthrough"));
      //grey_image_32F1 = cvCloneImage(cv_bridge_2_.imgMsgToCv((sensor_msgs::ImageConstPtr)(&(m_ColoredPointCloudSrv.response.confidenceMask)), "passthrough"));
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("[tof_camera_viewer] Could not convert images by cv_bridge.");
      return ipa_Utils::RET_FAILED;
    }
	ROS_INFO("[env_model_node] Point cloud received.");

	colored_point_cloud_.SetColorImage(color_image_8U3);
	colored_point_cloud_.SetXYZImage(xyz_image_32F3);
	colored_point_cloud_.SetGreyImage(grey_image_32F1);

	ROS_INFO("[env_model_node] Colored point cloud object updated.");
	ipa_Utils::MaskImage2(colored_point_cloud_.GetXYZImage(), colored_point_cloud_.GetXYZImage(), colored_point_cloud_.GetGreyImage(), colored_point_cloud_.GetGreyImage(), 500, 60000, 3);

    try
    {
    	IplImage* grey_image_8U3 = cvCreateImage(cvGetSize(grey_image_32F1), IPL_DEPTH_8U, 3);
    	IplImage* xyz_image_8U3 = cvCreateImage(cvGetSize(xyz_image_32F3), IPL_DEPTH_8U, 3);

    	ipa_Utils::ConvertToShowImage(grey_image_32F1, grey_image_8U3, 1);
    	ipa_Utils::ConvertToShowImage(xyz_image_32F3, xyz_image_8U3, 1);
    	cvShowImage("grey data", grey_image_8U3);
    	cvShowImage("xyz data", xyz_image_8U3);
    	cvShowImage("color data", colored_point_cloud_.GetColorImage());
    	cvWaitKey();
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("[env_model_node] Could not convert");
    }
	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::detectFeatures()
{
	ROS_INFO("[env_model_node] Detecting features");
	if(colored_point_cloud_.GetColorImage()!=0)
	{
		feature_vector_ = new BlobFeatureVector();
		feature_detector_->DetectFeatures(colored_point_cloud_.GetColorImage(), feature_vector_);
		//TODO: use colorImage0 but transform u and v to sharedImageSize
		AbstractFeatureVector::iterator It;
		for (It=feature_vector_->begin(); It!=feature_vector_->end(); It++)
		{
			unsigned char R,G,B;
			colored_point_cloud_.GetData((*It)->m_u,(*It)->m_v,(*It)->m_x,(*It)->m_y,(*It)->m_z,R,G,B);
			//ROS_INFO("[env_model_node] Feature Data: %f, %f, %f", (*It)->m_x, (*It)->m_y, (*It)->m_z);
		}
		((SURFDetector*)feature_detector_)->FilterFeatures(feature_vector_, 5);
	}
	else
	{
		ROS_ERROR("Could not detect features, no color image in point cloud.");
		return ipa_Utils::RET_FAILED;
	}
	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::getTransformationCam2Base()
{
	//TODO: ROS TF call to get neck angels or transformation
	if(srv_client_transform_camera2base_.call(transform_camera2base_srv_))
	{
		ROS_INFO("[env_model_node] Transformation Camera2Base service called [OK].");
	}
	else
	{
		ROS_ERROR("[env_model_node] Transformation Camera2Base service called [FAILED].");
		return ipa_Utils::RET_FAILED;
	}
	ROS_INFO("[env_model_node] Trnasformation Camera2Base (x,y,z,roll,pitch,yaw): %f, %f, %f, %f, %f, %f",
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
	std::cout << "transformation_camera2base_: " << transformation_camera2base_ << std::endl;
	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::initializeFilter()
{
	for (unsigned int i=0; i<fast_SLAM_.m_Particles.size(); i++)
	{
		fast_SLAM_.m_Particles[i].ValueGet()->SetPose(robot_pose_);
	}

	AbstractFeatureVector::iterator It;
	std::vector<int> mask(feature_vector_->size(),0);
	for (It=feature_vector_->begin(); It!=feature_vector_->end(); It++)
	{
		ColumnVector theta(3);
		transformCameraToBase(*It);
		(*It)->m_Id=-1;
		fast_SLAM_.AddUnknownFeature(*meas_model_, *It, 0);
	}
	data_association_.AddNewFeatures(feature_vector_, mask);
	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::updateFilter()
{
	ROS_INFO("[env_model_node] Updating filter");
	static int step = 1;

	fast_SLAM_.UpdatePosition(*system_model_, robot_pose_);
	int associationCtr = 0;
	unsigned int ctrAssociations;
	data_association_.AssociateData(feature_vector_);

	AbstractFeatureVector::iterator It;
	for (It=feature_vector_->begin(); It!=feature_vector_->end(); It++)
	{
		transformCameraToBase(*It);
		if((*It)->m_Id != -1)
		{
			associationCtr++;
			fast_SLAM_.UpdateKnownFeature(*meas_model_, *system_model_, *It, ctrAssociations);
		}
	}

	map_particle_ = fast_SLAM_.Resample(ctrAssociations);


	std::cout << "ctrAssociations: " << ctrAssociations << std::endl;
	std::cout << feature_vector_->size() << " features extracted" << std::endl;
	std::cout << associationCtr << " associated by descriptor" << std::endl;
	std::cout << map_particle_->m_RejectedFeatures.size() << " rejected in particle" << std::endl;
	std::cout << associationCtr - map_particle_->m_RejectedFeatures.size() << " remaining associations" << std::endl;
	std::cout << "map size: " << map_particle_->GetMap()->size() << std::endl;
	std::cout << "Original robot pose: " << robot_pose_ << std::endl;
	std::cout << "Corrected robot pose: " << *(map_particle_->GetPose()) << std::endl;

	/// Remove class associations from rejected particles
	for (unsigned int i=0; i<map_particle_->m_RejectedFeatures.size(); i++)
	{
		/// Remove class label if necessary
		map_particle_->m_RejectedFeatures[i]->m_Id = -1;
	}

	unsigned int i = 0;
	std::vector<int> mask(feature_vector_->size(),0);
	for (It=feature_vector_->begin(), i=0; It!=feature_vector_->end(); It++, i++)
	{
		/// Add new feature only if it is not masked
		/// It is important that the features ID corresponds
		/// to the position in the particle map
		if((*It)->m_Id == -1)
		{
			if ( mask[i] == 0) fast_SLAM_.AddUnknownFeature(*meas_model_, *It, step);
		}
		else mask[i]=1; //don't add known features to global list
	}
	data_association_.AddNewFeatures(feature_vector_, mask);
	step++;

    feature_map = map_particle_->GetMap();

	delete feature_vector_;

	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::calculateTransformationMatrix()
{
	MatrixWrapper::Matrix manbase2pltfbase(4,4);
	manbase2pltfbase = 0;
	manbase2pltfbase(1,2) = 1;
	manbase2pltfbase(2,1) = -1;
	manbase2pltfbase(3,3) = 1;
	manbase2pltfbase(4,4) = 1;
	transformation_tof2base_pltf_ = manbase2pltfbase*transformation_camera2base_*transformation_tof2camera_;

	for(int i=1; i<5; i++)
	{
		for(int j=1; j<5; j++)
		{
			if(fabs(transformation_tof2base_pltf_(i,j)) < 0.0001) transformation_tof2base_pltf_(i,j) = 0;
		}
	}
	std::cout << "transformation matrix: " << transformation_tof2base_pltf_ << std::endl;
}
unsigned long CobEnvModelNode::transformCameraToBase(boost::shared_ptr<AbstractFeature> af)
{
	BFL::ColumnVector xyz_cam(4);
	BFL::ColumnVector xyz_base(4);
	xyz_cam(1) = af->m_x;
	xyz_cam(2) = af->m_y;
	xyz_cam(3) = af->m_z;
	xyz_cam(4) = 1;

	//std::cout << manbase2pltfbase*transformation_camera2base_*transformation_tof2camera_ << std::endl;

	xyz_base = transformation_tof2base_pltf_*xyz_cam;
	//std::cout << xyz_base << std::endl;

	af->m_x = xyz_base(1);
	af->m_y = xyz_base(2);
	af->m_z = xyz_base(3);

	return ipa_Utils::RET_OK;
}


//#######################
//#### main programm ####



void RenderMap()
{
	if(feature_map!=0)
	{
		multimap<int,boost::shared_ptr<AbstractEKF> >::iterator It;
		for ( It=feature_map->begin() ; It != feature_map->end(); It++ )
		{
			int step = ((EKF2DLandmark*)It->second.operator ->())->m_Step;
			glColor3f(0.0f,0.0f,1.0f);
			glPointSize(4.0f);
			glBegin(GL_POINTS);
			glVertex3f(((EKF2DLandmark*)It->second.operator ->())->PostGet()->ExpectedValueGet()(1),
					((EKF2DLandmark*)It->second.operator ->())->PostGet()->ExpectedValueGet()(2),
					((EKF2DLandmark*)It->second.operator ->())->m_Height);
			glEnd( );
		}
	}
	/*else
		std::cout << "no map" << std::endl;*/
}

int main(int argc, char** argv)
{
    // initialize ROS, specify name of node
    ros::init(argc, argv, "cobEnvModelNode");
    
    CobEnvModelNode cobEnvModelNode;

	int i=0;
	unsigned char cmd=0;

	PointCloudRenderer::Init();
	PointCloudRenderer::SetExternRenderFunc(&RenderMap);
	PointCloudRenderer::SetKeyboardPointer(&cmd);
	PointCloudRenderer::Run();
 
    ros::Rate r(0.5);
    while(cobEnvModelNode.n.ok() && i<5)
    {
        ros::spinOnce();
    	/*cobEnvModelNode.getMeasurement();
		//cobEnvModelNode.getRobotPose();
		cobEnvModelNode.getTransformationCam2Base();
		cobEnvModelNode.detectFeatures();
		if(first)
			cobEnvModelNode.initializeFilter();
		else
			cobEnvModelNode.updateFilter();
		first = false;*/
		//i++;
        //feature_map = cobEnvModelNode.map_particle_->GetMap();
    	r.sleep();
    }

	while(cmd!='q')
	{
		usleep(100);
    }
		PointCloudRenderer::Exit();

    return 0;
}

