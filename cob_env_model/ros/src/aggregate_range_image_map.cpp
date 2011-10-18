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
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 01/2011
 * ToDo:
 * switch all console outputs to ROS_DEBUG erledigt
 * set flag to say whether pointclouds should be saved to files or not erledigt
 * rename variables according to coding guidelines: erledigt
 * 	see http://pointclouds.org/documentation/advanced/pcl_style_guide.php#variables
 * add comments to explain functionality
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
#include <sstream>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <cob_env_model/point_types.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/field_of_view_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/passthrough.h>
#include <pcl/range_image/range_image.h>

#include "reconfigureable_node.h"
#include <cob_env_model/aggregate_range_image_mapConfig.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model/GetFieldOfView.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### node class ####
class AggregateRangeImageMap : public pcl_ros::PCLNodelet, protected Reconfigurable_Node<cob_env_model::aggregate_range_image_mapConfig>
{
	typedef pcl::PointXYZRGB Point;

public:
    // Constructor
	AggregateRangeImageMap()
	   : Reconfigurable_Node<cob_env_model::aggregate_range_image_mapConfig>("AggregateRangeImageMap"),
	     first_(true),
	     ctr_(0)//,
	     /*set_maximumiterations_(50),
	     set_maxcorrespondencedistance_(0.1),
	     set_transformationepsilon_(1e-6),
	     file_path_("/home/goa/pcl_daten/table/icp/map_"),
	     ros_debug(true),
	     save_pc_(true),
	     save_icp_fov_map_(true),
	     save_pc_aligned_(true),
	     save_icp_fov_pc_(true),
	     save_map_fov_(true),
	     save_icp_map_(true),
	     vox_filter_setleafsize1(0.02),
		 vox_filter_setleafsize2(0.02),
		 vox_filter_setleafsize3(0.02),
		 r_limit_(0.1),
		 y_limit_(0.1),
		 p_limit_(0.1),
		 distance_limit_(0.3)*/
	{
	    setReconfigureCallback(boost::bind(&callback, this, _1, _2));
	}


    // Destructor
    ~AggregateRangeImageMap()
    {
    	/// void
    }

    // callback for dynamic reconfigure
    static void callback(AggregatePointMap *inst, cob_env_model::aggregate_range_image_mapConfig &config, uint32_t level)
    {
      if(!inst)
        return;

      boost::mutex::scoped_lock l(inst->m_mutex_point_reconf_);

      inst->set_maximumiterations_ = config.set_maximumiterations;
      inst->set_maxcorrespondencedistance_ = config.set_maxcorrespondencedistance;
      inst->set_transformationepsilon_ = config.set_transformationepsilon;
      inst->file_path_ = config.file_path;
      inst->save_pc_ = config.save_pc;
      inst->ros_debug = config.ros_debug;
      inst->save_icp_fov_map_ = config.save_icp_fov_map;
      inst->save_pc_aligned_ = config.save_pc_aligned;
      inst->save_icp_fov_pc_ = config.save_icp_fov_pc;
      inst->save_map_fov_ = config.save_map_fov;
      inst->save_icp_map_ = config.save_icp_map;
      inst->vox_filter_setleafsize1 = config.vox_filter_setleafsize1;
      inst->vox_filter_setleafsize2 = config.vox_filter_setleafsize2;
      inst->vox_filter_setleafsize3 = config.vox_filter_setleafsize3;
      inst->r_limit_ = config.r_limit;
      inst->p_limit_ = config.p_limit;
      inst->y_limit_ = config.y_limit;
      inst->distance_limit_ = config.distance_limit;

    }

    void onInit()
    {
    	PCLNodelet::onInit();
    	n_ = getNodeHandle();

		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &AggregateRangeImageMap::pointCloudSubCallback, this);
		point_cloud_pub_ = n_.advertise<pcl::PointCloud<pcl::PointWithRange> >("point_cloud2_map",1);
		point_cloud_pub_aligned_ = n_.advertise<pcl::PointCloud<pcl::PointWithRange> >("point_cloud2_aligned",1);
		//point_cloud_pub_aligned2_ = n_.advertise<pcl::PointCloud<PointWithRange> >("pc_aligned_and_boundary",1);
		fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",10);
		get_fov_srv_client_ = n_.serviceClient<cob_env_model::GetFieldOfView>("get_fov");
		//TODO: Read parameters from launch file

	/*	n_.param("aggregate_point_map/set_maxiterations_FOV_", set_maximumiterations_FOV_, 70);
		n_.param("aggregate_point_map/set_maxcorrespondencedistance_FOV_", set_maxcorrespondencedistance_FOV_ ,0.1);
		n_.param("aggregate_point_map/set_transformationepsilon_FOV_",set_transformationepsilon_FOV_ ,1e-6); */
		n_.param("aggregate_point_map/set_maximumiterations_" ,set_maximumiterations_ ,50);
		n_.param("aggregate_point_map/set_maxcorrespondencedistance_" ,set_maxcorrespondencedistance_,0.1);
		n_.param("aggregate_point_map/set_transformationepsilon_" ,set_transformationepsilon_,1e-6);
		n_.param("aggregate_point_map/file_path" ,file_path_ ,std::string("/home/goa/pcl_daten/table/icp/map_"));
		n_.param("aggregate_point_map/ros_debug" ,ros_debug ,true);
		n_.param("aggregate_point_map/save_pc_",save_pc_ , false);
		n_.param("aggregate_point_map/save_icp_fov_map_",save_icp_fov_map_ ,false);
		n_.param("aggregate_point_map/save_pc_aligned_",save_pc_aligned_,false);
		n_.param("aggregate_point_map/save_icp_fov_pc_" ,save_icp_fov_pc_,false);
		n_.param("aggregate_point_map/save_map_fov_" ,save_map_fov_,false);
		n_.param("aggregate_point_map/save_icp_map_" ,save_icp_map_,false);
		n_.param("aggregate_point_map/vox_filter_setleafsize1" ,vox_filter_setleafsize1, 0.001);
		n_.param("aggregate_point_map/vox_filter_setleafsize2" ,vox_filter_setleafsize2, 0.001);
		n_.param("aggregate_point_map/vox_filter_setleafsize3" ,vox_filter_setleafsize3, 0.001);
		n_.param("aggregate_point_map/r_limit_",r_limit_,0.1);
		n_.param("aggregate_point_map/y_limit_",y_limit_,0.1);
	    n_.param("aggregate_point_map/p_limit_",p_limit_,0.1);
	    n_.param("aggregate_point_map/distance_limit_",distance_limit_,0.3);
    }

    void pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc)
    {
        boost::mutex::scoped_lock l(m_mutex_point_reconf_);

    	pcl::RangeImage::Ptr ri = pcl::RangeImage::Ptr(new pcl::RangeImage);
    	ri->height = pc->height;
    	ri->width = pc->width;
    	ri->points.resize(ri->height*ri->width);
		ri->is_dense = false;
		ri->header = pc->header;
		for(int i = 0; i<pc->size(); i++)
		{
			pcl::PointWithRange p;
			{
				ri->points[i].x = pc->points[i].x;
				ri->points[i].y = pc->points[i].y;
				ri->points[i].z = pc->points[i].z;
				ri->points[i].range = sqrt(pc->points[i].x*pc->points[i].x+pc->points[i].y*pc->points[i].y+pc->points[i].z*pc->points[i].z);
			}
		}
		ri->setAngularResolution(0.001559); //for kinect
    	//ROS_INFO("PointCloudSubCallback");
    	StampedTransform transform;
    	try
    	{
       		//tf_listener_.waitForTransform("/map", pc->header.frame_id, pc->header.stamp, ros::Duration(2));
    		tf_listener_.lookupTransform("/map", ri->header.frame_id, ri->header.stamp/*ros::Time(0)*/, transform);
    		KDL::Frame frame_KDL, frame_KDL_old;
    		tf::TransformTFToKDL(transform, frame_KDL);
    		tf::TransformTFToKDL(transform_old_, frame_KDL_old);
    		double r,p,y;
    		frame_KDL.M.GetRPY(r,p,y);
    		double r_old,p_old,y_old;
    		frame_KDL_old.M.GetRPY(r_old,p_old,y_old);

			if(first_)
			{
				pcl_ros::transformPointCloud(*ri, *ri, transform);
				map_ = *ri;
				frustum_ = *ri;
				map_.header.frame_id="/map";
				//point_cloud_pub_aligned_.publish(map_);
				//downsampleMap();
				//point_cloud_pub_.publish(map_);
				ctr_++;
				first_ = false;
			}
			else
			{
				if(fabs(r-r_old) > r_limit_ || fabs(p-p_old) > p_limit_ || fabs(y-y_old) > y_limit_ ||
						transform.getOrigin().distance(transform_old_.getOrigin()) > distance_limit_)
				{
			    	boost::timer t;
					ROS_DEBUG_STREAM_COND(ros_debug ,  "Registering new point cloud" << std::endl);
					transform_old_ = transform;
					//transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
					//pcl_ros::transformPointCloud ("/map", *(pc.get()), *(pc.get()), tf_listener_);
					//pcl_ros::transformPointCloud(*ri, *ri, transform);
					Eigen::eigen2_Transform3d trafo_e2;
					tf::TransformTFToEigen (transform, trafo_e2);
					Eigen::Matrix4f trafo_matrix;
					for(int i=0; i<4; i++)
					{
						for(int j=0; j<4; j++)
						{
							trafo_matrix(i,j) = trafo_e2(i,j);
						}
					}
					Eigen::Affine3f trafo_eigen;
					trafo_eigen = Eigen::Affine3f(trafo_matrix);
					ri->setTransformationToRangeImageSystem (trafo_eigen.inverse());
					//Eigen::Vector3f transformed_point = trafo_eigen*ri->points[100000].getVector3fMap();
					//ROS_DEBUG_STREAM_COND(ros_debug ,  "frame_id " << pc->header.frame_id << std::endl);
					//ri->header.frame_id = "/map";

					if(save_pc_==true)
					{
						std::stringstream ss2;
						ss2 << file_path_ << "/pc_" << ctr_ << ".pcd";
						pcl::io::savePCDFileASCII (ss2.str(), *ri);
					}
			        /*int x,y;
			        ri->getImagePoint (transformed_point, x, y);
			        std::cout << "p_trans: " << transformed_point << std::endl;
			        ROS_INFO("x,y: %d,%d", x,y);
			        return;*/
					doFOVICP(*ri);
					//doICP(pc);
					//addToMap(pc);
					//pcl::PointCloud<pcl::PointWithRange> *ri_ptr = &ri;
					pcl::PointCloud<pcl::PointWithRange>::Ptr ri_ptr = pcl::PointCloud<pcl::PointWithRange>::Ptr(ri);
					//point_cloud_pub_aligned_.publish(ri_ptr);
					//downsampleMap();
					//point_cloud_pub_.publish(map_);
					if(save_pc_aligned_==true)
					{
						ROS_INFO("Saving pc_aligned.");
						std::stringstream ss;
						ss << file_path_ << "/pc_aligned_" << ctr_ << ".pcd";
						pcl::io::savePCDFileASCII (ss.str(), *ri);
					}
					if(save_icp_fov_map_ ==true)
						{
							std::stringstream ss1;
							ss1 << file_path_ << "/map_" << ctr_ << ".pcd";
							pcl::io::savePCDFileASCII (ss1.str(), map_);
						}
					ctr_++;
					ROS_INFO("Frustum size: %d", frustum_.size());
					if (save_map_fov_==true)
					{
						std::stringstream ss3;
						ss3 << file_path_ << "/map_fov_" << ctr_ << ".pcd";
						pcl::io::savePCDFileASCII (ss3.str(), frustum_);
					}
			    	ROS_INFO("ICP took %f s", t.elapsed());
				}
			}
    	}
    	catch (tf::TransformException ex)
    	{
    		ROS_ERROR("%s",ex.what());
    	}
    }


    void doFOVICP(pcl::RangeImage& ri)
    {
		boost::timer t;
    	//setup IO stuff
		std::fstream filestr;
		//filestr.open("/home/goa/pcl_daten/table/icp_fov/meas.csv", std::fstream::in | std::fstream::out | std::fstream::app);

		//generate marker for FOV visualization in RViz
		//visualization_msgs::Marker marker = generateMarker(sensor_fov_hor_,sensor_fov_ver_,sensor_max_range_, map_.header.frame_id, pc->header.stamp);
		//fov_marker_pub_.publish(marker);
		cob_env_model::GetFieldOfView get_fov_srv;
		get_fov_srv.request.target_frame = std::string("/map");
		get_fov_srv.request.stamp = ri.header.stamp;
		if(get_fov_srv_client_.call(get_fov_srv))
		{
			ROS_DEBUG_STREAM_COND(ros_debug ,"[aggregate_point_map] FOV service called [OK].");
		}
		else
		{
			ROS_ERROR("[aggregate_point_map] FOV service called [FAILED].");
			return;
		}
		n_up_t_(0) = get_fov_srv.response.fov.points[0].x;
		n_up_t_(1) = get_fov_srv.response.fov.points[0].y;
		n_up_t_(2) = get_fov_srv.response.fov.points[0].z;
		n_down_t_(0) = get_fov_srv.response.fov.points[1].x;
		n_down_t_(1) = get_fov_srv.response.fov.points[1].y;
		n_down_t_(2) = get_fov_srv.response.fov.points[1].z;
		n_right_t_(0) = get_fov_srv.response.fov.points[2].x;
		n_right_t_(1) = get_fov_srv.response.fov.points[2].y;
		n_right_t_(2) = get_fov_srv.response.fov.points[2].z;
		n_left_t_(0) = get_fov_srv.response.fov.points[3].x;
		n_left_t_(1) = get_fov_srv.response.fov.points[3].y;
		n_left_t_(2) = get_fov_srv.response.fov.points[3].z;
		n_origin_t_(0) = get_fov_srv.response.fov.points[4].x;
		n_origin_t_(1) = get_fov_srv.response.fov.points[4].y;
		n_origin_t_(2) = get_fov_srv.response.fov.points[4].z;
		n_max_range_t_(0) = get_fov_srv.response.fov.points[5].x;
		n_max_range_t_(1) = get_fov_srv.response.fov.points[5].y;
		n_max_range_t_(2) = get_fov_srv.response.fov.points[5].z;


		//segment FOV
		//seg_.setInputCloud(map_.makeShared());
		//transformNormals(map_.header.frame_id, pc->header.stamp);
		//pcl::PointIndices indices;
		//seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, n_origin_t_, n_max_range_t_);
		//ROS_INFO("FOV segment: %d", indices.indices.size());
		/*pcl::RangeImage frustum;
		pcl::ExtractIndices<PointWithRange> extractIndices;
		extractIndices.setInputCloud(map_.makeShared());
		extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
		extractIndices.filter(frustum);*/

		//do ICP
		/*pcl::IterativeClosestPoint<Point,Point> icp;
		//TODO: Test
		icp.setInputCloud(pc->makeShared());
		//icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
		icp.setInputTarget(frustum.makeShared());
		icp.setMaximumIterations(set_maximumiterations_);
		icp.setMaxCorrespondenceDistance(set_maxcorrespondencedistance_);
		icp.setTransformationEpsilon (set_transformationepsilon_);*/
		//pcl::RangeImage ri_aligned;
		Eigen::Affine3f id;//(Eigen::Affine3f::Identity);
		id.setIdentity();
		Eigen::Affine3f trafo = ri.doIcp(frustum_, id, 10, 0.1, 0.05, 100,1,1);
		std::cout << "ICP trafo: " << trafo << std::endl;
		pcl::transformPointCloud(ri, ri, trafo.inverse());
		ri.header.frame_id = "/map";
		//icp.align(pc_aligned);
		map_ += ri;
		frustum_ = ri;

		//do logging
		double time = t.elapsed();

		//point_cloud_pub_aligned_.publish(ri_aligned);
		//point_cloud_pub_aligned2_.publish(frustum);

    }




    void downsampleMap()
    {
		pcl::VoxelGrid<pcl::PointWithRange> vox_filter;
		vox_filter.setInputCloud(map_.makeShared());
		//TODO: launchfile parameter
		vox_filter.setLeafSize(vox_filter_setleafsize1,vox_filter_setleafsize2,vox_filter_setleafsize3);
		vox_filter.filter(map_);
    }


    ros::NodeHandle n_;
    ros::Time stamp_;


protected:
    ros::Subscriber point_cloud_sub_;		//subscriber for input pc
    ros::Publisher point_cloud_pub_;		//publisher for map
    ros::Publisher point_cloud_pub_aligned_;//publisher for aligned pc
    //ros::Publisher point_cloud_pub_aligned2_;//publisher for aligned pc
    ros::Publisher fov_marker_pub_;			//publisher for FOV marker
    ros::ServiceClient get_fov_srv_client_;

    TransformListener tf_listener_;
    StampedTransform transform_old_;

    pcl::RangeImage map_;	//FOV ICP map
	pcl::RangeImage frustum_;

    bool first_;
    int set_maximumiterations_;
    double set_maxcorrespondencedistance_;
    double set_transformationepsilon_;
/*
    int set_maximumiterations_FOV_;
    double set_maxcorrespondencedistance_FOV_;
    double set_transformationepsilon_FOV_;
*/
    double vox_filter_setleafsize1;
    double vox_filter_setleafsize2;
    double vox_filter_setleafsize3;

    bool ros_debug;
    std::string file_path_;

    //Speichervariablen
    bool save_pc_;
    bool save_icp_fov_map_;
    bool save_pc_aligned_;
    bool save_icp_fov_pc_;
    bool save_map_fov_;
    bool save_icp_map_;


    double y_limit_;
    double distance_limit_;
    double r_limit_;
    double p_limit_;

	Eigen::Vector3d n_up_t_;
	Eigen::Vector3d n_down_t_;
	Eigen::Vector3d n_right_t_;
	Eigen::Vector3d n_left_t_;
	Eigen::Vector3d n_origin_t_;
	Eigen::Vector3d n_max_range_t_;

	ipa_env_model::FieldOfViewSegmentation<PointWithRange> seg_;

	int ctr_;

	boost::mutex m_mutex_point_reconf_;

};


PLUGINLIB_DECLARE_CLASS(cob_env_model, AggregateRangeImageMap, AggregateRangeImageMap, nodelet::Nodelet)

