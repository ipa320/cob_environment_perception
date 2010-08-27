
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
 * Date of creation: 08/2010
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
#include <image_transport/image_transport.h>
#include <pcl_ros/subscriber.h>
#include <pcl_ros/publisher.h>
#include <pcl/ros/register_point_struct.h>
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"

#include <cob_vision_ipa_utils/cpc_point.h>


#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <model/analyticmeasurementmodel_gaussianuncertainty.h>
#include <model/systemmodel.h>

#include <cob_sensor_fusion/ColoredPointCloud.h>
#include <cob_vision_features/SURFDetector.h>
#include <cob_vision_features/AbstractFeatureVector.h>


// ROS message includes
#include <std_msgs/String.h>
#include <cob_msgs/ColoredPointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <cob_srvs/GetColoredPointCloud.h>
#include <cob_srvs/GetPlatformPosition.h>
#include <cob_srvs/GetTransformCamera2Base.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/GetEnvModel.h>


// GUI includes
#include <cob_vision_ipa_utils/PointCloudRenderer.h>

// external includes
#include <boost/timer.hpp>


using namespace ipa_Features;

BFL::ColumnVector* robot_pose=0;

void deleter(sensor_msgs::Image* const) {}


//####################
//#### node class ####
class CobEnvModelNode
{

    public:

        // Constructor
		CobEnvModelNode(const ros::NodeHandle& nh);

        
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


    	///Call the service for the robot pose
    	unsigned long getRobotPose();

    	unsigned long getMeasurement();

    	//void topicCallback_ColoredPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud);
    	void topicCallback_ColoredPointCloud(const pcl::PointCloud<CPCPoint>::ConstPtr& cloud);

    	unsigned long detectFeatures();

    	unsigned long calculateTransformationMatrix();

    	///Calls the service for the transformation from right color camera to
    	///manipulation robot base (which is rotated by 90° to the platform base)
    	unsigned long getTransformationCam2Base();


    	///Transforms a feature point from tof to platform base
    	unsigned long transformCameraToBase(boost::shared_ptr<AbstractFeature> af);

		// create a handle for this node, initialize node
		ros::NodeHandle n;

		image_transport::ImageTransport image_transport_;

		// topics to publish
		image_transport::Publisher color_image_pub_;	///< Publishes the color information of the point cloud
		image_transport::Publisher point_cloud_pub_;			///< Publishes the point cloud
		image_transport::Publisher confidence_mask_pub_;			///< Publishes the confidence mask
		image_transport::Publisher feature_mask_pub_;

		// topics to subscribe, callback is called for new messages arriving
		//pcl_ros::Subscriber<sensor_msgs::PointCloud2> topicSub_coloredPointCloud_;
		pcl_ros::Subscriber<CPCPoint> topicSub_coloredPointCloud_;
		pcl_ros::Publisher<CPCPoint> topicPub_filteredPointCloud_;

		tf::TransformListener transform_listener_;

		ros::ServiceServer srv_server_trigger_;
		ros::ServiceServer srv_get_env_model_;
		ros::ServiceClient srv_client_colored_point_cloud_;
		ros::ServiceClient srv_client_platform_position_;
		ros::ServiceClient srv_client_transform_camera2base_;


    protected:

		//ROS
		cob_srvs::GetColoredPointCloud colored_point_cloud_srv_;
		cob_srvs::GetPlatformPosition platform_position_srv_;
		cob_srvs::GetTransformCamera2Base transform_camera2base_srv_;
		cob_srvs::GetEnvModel env_model_srv_;

		tf::TransformListener tf_listener;

		//Matrix m_transformCam2Base;


		sensor_msgs::CvBridge cv_bridge_0_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_1_; ///< Converts ROS image messages to openCV IplImages
		sensor_msgs::CvBridge cv_bridge_2_; ///< Converts ROS image messages to openCV IplImages

		AbstractLocalFeatureDetector* feature_detector_;
		ipa_SensorFusion::ColoredPointCloud colored_point_cloud_;
		BFL::ColumnVector robot_pose_;
		BFL::AnalyticMeasurementModelGaussianUncertainty* meas_model_;
		BFL::SystemModel<BFL::ColumnVector>* system_model_;
		AbstractFeatureVectorPtr feature_vector_;
		MatrixWrapper::Matrix transformation_camera2base_;
		MatrixWrapper::Matrix transformation_tof2camera_;
		MatrixWrapper::Matrix transformation_camera2neck_;
		MatrixWrapper::Matrix transformation_tof2base_pltf_;


};


CobEnvModelNode::CobEnvModelNode(const ros::NodeHandle& nh)
	: n(nh),
	  image_transport_(nh),
	  robot_pose_(3),
	  transformation_camera2base_(4,4),
	  transformation_tof2camera_(4,4),
	  transformation_camera2neck_(4,4),
	  transformation_tof2base_pltf_(4,4)
{
	init();
    //topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
    //topicSub_coloredPointCloud = n.subscribe("/sensor_fusion/ColoredPointCloud", 1, &CobEnvModelNode::topicCallback_coloredPointCloud, this);
    srv_client_colored_point_cloud_ = n.serviceClient<cob_srvs::GetColoredPointCloud>("get_colored_pc");
    srv_client_platform_position_ = n.serviceClient<cob_srvs::GetPlatformPosition>("/get_platform_position");
    srv_client_transform_camera2base_ = n.serviceClient<cob_srvs::GetTransformCamera2Base>("/transform_camera2base");
    srv_server_trigger_ = n.advertiseService("update_env_model", &CobEnvModelNode::srvCallback_UpdateEnvModel, this);
    //srv_get_env_model_ = n.advertiseService("get_env_model", &CobEnvModelNode::srvCallback_GetEnvModel, this);
	color_image_pub_ = image_transport_.advertise("fused_image_color", 1);
	point_cloud_pub_ = image_transport_.advertise("fused_image_xyz", 1);
	confidence_mask_pub_ = image_transport_.advertise("fused_image_confidence", 1);
	feature_mask_pub_ = image_transport_.advertise("fused_image_features", 1);

	//topicSub_coloredPointCloud_.subscribe(n,"point_cloud", 1, boost::bind(&CobEnvModelNode::topicCallback_ColoredPointCloud, this, _1));
	topicSub_coloredPointCloud_.subscribe(n,"point_cloud", 1, boost::bind(&CobEnvModelNode::topicCallback_ColoredPointCloud, this, _1));
	topicPub_filteredPointCloud_.advertise(n,"point_cloud_filtered", 1);
}

unsigned long CobEnvModelNode::init()
{
	feature_detector_ = new SURFDetector();
	((SURFDetector*)feature_detector_)->Init(500);
	//m_FeatureDetector = new CalonderSTARDetector();


    /*XmlRpc::XmlRpcValue trafo_tof2cam;

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

    XmlRpc::XmlRpcValue trafo_cam2neck;

    if (n.hasParam("trafo_cam2neck"))
    {
        n.getParam("trafo_cam2neck", trafo_cam2neck);
    }
    else
    {
        ROS_ERROR("Parameter trafo_cam2neck not set");
    }
    for (int j = 0; j<trafo_cam2neck.size(); j++ ) // j-th point
    {
        for (int k = 0; k<trafo_cam2neck[j].size(); k++ ) // k-th value of pos
        {
            //ROS_INFO("      pos value %d of %d = %f",k,traj_param[i][j][0].size(),(double)traj_param[i][j][0][k]);
            //ROS_INFO("      vel value %d of %d = %f",k,traj_param[i][j][1].size(),(double)traj_param[i][j][1][k]);
        	transformation_camera2neck_(j+1,k+1) = (double)trafo_cam2neck[j][k];
        }
    }*/


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
		/*if(getRobotPose() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not get robot pose.");
			//return false;
		}
		if(getTransformationCam2Base() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not get transformation camera2base.");
			//return false;
		}
		/*calculateTransformationMatrix();
		if(detectFeatures() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not detect features.");
			return false;
		}
		if(initializeFilter() == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Could not initialize filter.");
			return false;
		}*/
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
	}
	else if(req.command == 3)
	{
		//TODO: send map to renderer or collision avoidance
		ROS_INFO("Stopping environment modelling.");
		n.shutdown();
	}
	res.success = 1;
	return true;
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
	robot_pose = &robot_pose_;
	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::getMeasurement()
{
	ROS_INFO("[env_model_node] Colored point cloud service call.");
	boost::timer t;

	if(srv_client_colored_point_cloud_.call(colored_point_cloud_srv_))
	{
		ROS_INFO("[env_model_node] Colored point cloud service called [OK].");
	}
	else
	{
		ROS_ERROR("[env_model_node] Colored point cloud service called [FAILED].");
		return ipa_Utils::RET_FAILED;
	}
	cv::Mat color_image_8U3;
	cv::Mat xyz_image_32F3;
	cv::Mat grey_image_32F1;
	sensor_msgs::ImageConstPtr color_image(&(colored_point_cloud_srv_.response.colorImage),deleter);
	sensor_msgs::ImageConstPtr xyz_image(&(colored_point_cloud_srv_.response.xyzImage),deleter);
	sensor_msgs::ImageConstPtr grey_image(&(colored_point_cloud_srv_.response.confidenceMask),deleter);
    try
    {
      /*color_image_8U3 = cvCloneImage(cv_bridge_0_.imgMsgToCv(color_image, "passthrough"));
      xyz_image_32F3 = cvCloneImage(cv_bridge_1_.imgMsgToCv(xyz_image, "passthrough"));
      grey_image_32F1 = cvCloneImage(cv_bridge_2_.imgMsgToCv(grey_image, "passthrough"));*/
    	color_image_8U3 = cv_bridge_0_.imgMsgToCv(color_image, "passthrough");
    	xyz_image_32F3 = cv_bridge_1_.imgMsgToCv(xyz_image, "passthrough");
    	grey_image_32F1 = cv_bridge_2_.imgMsgToCv(grey_image, "passthrough");
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("[tof_camera_viewer] Could not convert images by cv_bridge.");
      return ipa_Utils::RET_FAILED;
    }
	ROS_INFO("[env_model_node] Point cloud received.");
	ROS_INFO("\tTime: %f", t.elapsed());
	t.restart();

	colored_point_cloud_.SetColorImage(color_image_8U3);
	colored_point_cloud_.SetXYZImage(xyz_image_32F3);
	colored_point_cloud_.SetGreyImage(grey_image_32F1);

	ROS_INFO("[env_model_node] Colored point cloud object updated.");
	ROS_INFO("\tTime: %f", t.elapsed());
	t.restart();

	//TODO: write to mask, take a look at ROS outlier removal, use parameters
	ipa_Utils::FilterByAmplitude(colored_point_cloud_.GetXYZImage(), colored_point_cloud_.GetGreyImage(), 0, 0, 500, 60000);
	cv::Mat buf;
	ipa_Utils::FilterSpeckles(colored_point_cloud_.GetXYZImage(),20,0.01, buf);

	ROS_INFO("[env_model_node] Colored point cloud filtered.");
	ROS_INFO("\tTime: %f", t.elapsed());
	t.restart();

	feature_vector_ = AbstractFeatureVectorPtr(new AbstractFeatureVector());
	feature_detector_->DetectFeatures(colored_point_cloud_.GetColorImage(), *feature_vector_);

	ROS_INFO("[env_model_node] SURF features detected.");
	ROS_INFO("\tTime: %f", t.elapsed());
	t.restart();

	cv::Mat feature_mask(color_image_8U3.size(),CV_8UC1, cv::Scalar(0));
	AbstractFeatureVector::iterator It;
	for (It=feature_vector_->begin(); It!=feature_vector_->end(); It++)
	{
		//if((*It)->Get<double>(M_R) > 5)
			feature_mask.at<unsigned char>((int)((*It)->Get<double>(M_V)),(int)((*It)->Get<double>(M_U))) = 255;
	}

	ROS_INFO("[env_model_node] feature mask created.");
	ROS_INFO("\tTime: %f", t.elapsed());
	t.restart();
	/*feature_vector_->DrawListInIplImage(colored_point_cloud_.GetColorImage(), cvScalar(255,255,100));
	cv::imshow("feature mask", feature_mask);
	cv::imshow("color",colored_point_cloud_.GetColorImage());
	cv::waitKey();*/

	sensor_msgs::Image color_image_msg;
	sensor_msgs::Image xyz_image_msg;
	sensor_msgs::Image confidence_mask_msg;
	sensor_msgs::Image feature_mask_msg;

	// Convert openCV IplImages to ROS messages
	try
	{
		IplImage color_img = colored_point_cloud_.GetColorImage();
		IplImage xyz_img = colored_point_cloud_.GetXYZImage();
		IplImage grey_img = colored_point_cloud_.GetGreyImage();
		IplImage feature_img = feature_mask;

		color_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(&color_img, "passthrough"));
		xyz_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(&xyz_img, "passthrough"));
		confidence_mask_msg = *(sensor_msgs::CvBridge::cvToImgMsg(&grey_img, "passthrough"));
		feature_mask_msg = *(sensor_msgs::CvBridge::cvToImgMsg(&feature_img, "passthrough"));
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("[color_camera] Could not convert IplImage to ROS message");
	}

	// Set time stamp
	ros::Time now = ros::Time::now();
	color_image_msg.header.stamp = now;
	xyz_image_msg.header.stamp = now;
	confidence_mask_msg.header.stamp = now;
	feature_mask_msg.header.stamp = now;

	// Publish message
	color_image_pub_.publish(color_image_msg);
	point_cloud_pub_.publish(xyz_image_msg);
	confidence_mask_pub_.publish(confidence_mask_msg);
	feature_mask_pub_.publish(feature_mask_msg);

	ROS_INFO("[env_model_node] Point cloud published.");
	ROS_INFO("\tTime: %f", t.elapsed());

	return ipa_Utils::RET_OK;
}

void CobEnvModelNode::topicCallback_ColoredPointCloud(const pcl::PointCloud<CPCPoint>::ConstPtr& cloud/*const sensor_msgs::PointCloud2ConstPtr& cloud*/)
{
	pcl::io::savePCDFileASCII ("cloud_unfiltered.pcd", *cloud);
	/*pcl::PointCloud<pcl::PointXYZ> input;
	  pcl::PCDReader reader;
	  reader.read ("cloud_unfiltered.pcd", input);*/

	  ROS_INFO ("PointCloud before filtering: %d data points (%s).", cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str ());

	pcl::PointCloud<CPCPoint> cloud_filtered;
	/*pcl::VoxelGrid<pcl::PointXYZ> vox_filter;
	vox_filter.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(input));
	vox_filter.setLeafSize(0.01, 0.01, 0.01);
	vox_filter.filter(cloud_filtered);*/
	pcl::VoxelGrid<CPCPoint> vox_filter;
	vox_filter.setInputCloud(cloud);
	vox_filter.setLeafSize(0.02, 0.02, 0.02);
	//vox_filter.setDownsampleAllData(false);
	boost::timer t;
	vox_filter.filter(cloud_filtered);
	ROS_INFO("[env_model_node] Pointcloud downsampled.");
	ROS_INFO("\tTime: %f", t.elapsed());
	ROS_INFO ("PointCloud after filtering: %d data points (%s).", cloud_filtered.width * cloud_filtered.height, pcl::getFieldsList (cloud_filtered).c_str ());
	pcl::io::savePCDFileASCII ("cloud_filtered.pcd", cloud_filtered);
	topicPub_filteredPointCloud_.publish(cloud_filtered);
}

unsigned long CobEnvModelNode::detectFeatures()
{
	ROS_INFO("[env_model_node] Detecting features");
/*	if(!colored_point_cloud_.GetColorImage().empty())
	{
		feature_vector_ = new BlobFeatureVector();
		feature_detector_->DetectFeatures(colored_point_cloud_.GetColorImage(), *feature_vector_);
		//TODO: use colorImage0 but transform u and v to sharedImageSize
		AbstractFeatureVector::iterator It;
		for (It=feature_vector_->begin(); It!=feature_vector_->end(); It++)
		{
			unsigned char R,G,B;
			colored_point_cloud_.GetData((*It)->Get<double>(M_U),(*It)->Get<double>(M_V),(*It)->Get<double>(M_X),(*It)->Get<double>(M_Y),(*It)->Get<double>(M_Z),R,G,B);
			(*It)->Get<double>(M_RED) = (double) R;
			(*It)->Get<double>(M_GREEN) = (double) G;
			(*It)->Get<double>(M_BLUE) = (double) B;
			//ROS_INFO("[env_model_node] Feature Data: %f, %f, %f", (*It)->m_x, (*It)->m_y, (*It)->m_z);
		}
		((SURFDetector*)feature_detector_)->FilterFeatures(feature_vector_, 5);
	}
	else
	{
		ROS_ERROR("Could not detect features, no color image in point cloud.");
		return ipa_Utils::RET_FAILED;
	}
	ROS_INFO("OK");*/
	return ipa_Utils::RET_OK;
}

/*unsigned long CobEnvModelNode::getTransformationCam2Base()
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
	transformation_camera2base_(1,2) = cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll);
	transformation_camera2base_(1,3) = cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
	transformation_camera2base_(1,4) = transform_camera2base_srv_.response.transformation.x;
	transformation_camera2base_(2,1) = sin(yaw)*cos(pitch);
	transformation_camera2base_(2,2) = sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll);
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
}*/

unsigned long CobEnvModelNode::getTransformationCam2Base()
{
    tf::StampedTransform transform;
    try{
      tf_listener.lookupTransform("/base_link", "/torso_upper_neck_tilt_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    /*ROS_INFO("rotation: %f,%f,%f\n%f,%f,%f\n%f,%f,%f",transform.getBasis()[0][0],transform.getBasis()[0][1],transform.getBasis()[1][2],
    		transform.getBasis()[1][0],transform.getBasis()[1][1],transform.getBasis()[1][2],
    		transform.getBasis()[2][0],transform.getBasis()[2][1],transform.getBasis()[2][2]);*/

    MatrixWrapper::Matrix transformation_neck2base(4,4);
    for(int i=0; i<3; i++)
    {
    	for(int j=0; j<3; j++)
    	{
    		transformation_neck2base(i+1,j+1) = transform.getBasis()[i][j];
    	}
    }
    transformation_neck2base(1,4) = transform.getOrigin().x();
    transformation_neck2base(2,4) = transform.getOrigin().y();
    transformation_neck2base(3,4) = transform.getOrigin().z();
    transformation_neck2base(4,1) = 0;
    transformation_neck2base(4,2) = 0;
    transformation_neck2base(4,3) = 0;
    transformation_neck2base(4,4) = 1;

    transformation_camera2base_ = transformation_camera2neck_*transformation_neck2base;

    std::cout << "transformation_camera2neck_: " << transformation_camera2neck_ << std::endl;
    std::cout << "transformation_neck2base: " << transformation_neck2base << std::endl;
    std::cout << "transformation_camera2base_: " << transformation_camera2base_ << std::endl;
    ROS_INFO("translation: %f,%f,%f",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());

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
	return ipa_Utils::RET_OK;
}

unsigned long CobEnvModelNode::transformCameraToBase(boost::shared_ptr<AbstractFeature> af)
{
	BFL::ColumnVector xyz_cam(4);
	BFL::ColumnVector xyz_base(4);
	xyz_cam(1) = af->Get<double>(M_X);
	xyz_cam(2) = af->Get<double>(M_Y);
	xyz_cam(3) = af->Get<double>(M_Z);
	xyz_cam(4) = 1;

	//std::cout << transformation_tof2base_pltf_ << std::endl;

	xyz_base = transformation_tof2base_pltf_*xyz_cam;
	//std::cout << xyz_base << std::endl;

	af->Get<double>(M_X) = xyz_base(1);
	af->Get<double>(M_Y) = xyz_base(2);
	af->Get<double>(M_Z) = xyz_base(3);

	return ipa_Utils::RET_OK;
}


//#######################
//#### main programm ####




int main(int argc, char** argv)
{
    // initialize ROS, specify name of node
    ros::init(argc, argv, "cobEnvModelNode");
    
	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;

    bool use_opengl = false;
    nh.getParam("/cob_env_model/env_model/use_opengl", use_opengl);

    CobEnvModelNode cobEnvModelNode(nh);

	int i=0;
	unsigned char cmd=0;

 
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


    return 0;
}

