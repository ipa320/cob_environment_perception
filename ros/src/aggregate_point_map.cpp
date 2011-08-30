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
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cob_env_model/point_types.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/field_of_view_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model_msgs/GetFieldOfView.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### node class ####
class AggregatePointMap : public pcl_ros::PCLNodelet
{
  typedef pcl::PointXYZ Point;

public:
  // Constructor
  AggregatePointMap()
  : first_(true),
    ctr_(0)//,
  /*icp_max_iterations_(50),
	     icp_max_corr_dist_(0.1),
	     icp_trf_epsilon_(1e-6),
	     file_path_("/home/goa/pcl_daten/table/icp/map_"),
	     ros_debug(true),
	     save_pc_(true),
	     save_icp_fov_map_(true),
	     save_pc_aligned_(true),
	     save_icp_fov_pc_(true),
	     save_map_fov_(true),
	     save_icp_map_(true),
	     voxel_leafsize_x_(0.02),
		 voxel_leafsize_y_(0.02),
		 voxel_leafsize_z_(0.02),
		 r_limit_(0.1),
		 y_limit_(0.1),
		 p_limit_(0.1),
		 distance_limit_(0.3)*/
  {
    pcl::io::loadPCDFile("/home/goa/pcl_daten/kitchen_ground_truth/whole_kitchen.pcd", ref_map_);
  }


  // Destructor
  ~AggregatePointMap()
  {
    /// void
  }

  void onInit()
  {
    PCLNodelet::onInit();
    n_ = getNodeHandle();

    point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &AggregatePointMap::pointCloudSubCallback, this);
    point_cloud_pub_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_map",1);
    point_cloud_pub_aligned_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_aligned",1);
    //point_cloud_pub_aligned2_ = n_.advertise<pcl::PointCloud<Point> >("pc_aligned_and_boundary",1);
    fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",10);
    get_fov_srv_client_ = n_.serviceClient<cob_env_model_msgs::GetFieldOfView>("get_fov");
    //TODO: Read parameters from launch file

    /*	n_.param("aggregate_point_map/set_maxiterations_FOV_", icp_max_iterations_FOV_, 70);
		n_.param("aggregate_point_map/icp_max_corr_dist_FOV_", icp_max_corr_dist_FOV_ ,0.1);
		n_.param("aggregate_point_map/icp_trf_epsilon_FOV_",icp_trf_epsilon_FOV_ ,1e-6); */
    n_.param("aggregate_point_map/icp_max_iterations" ,icp_max_iterations_ ,50);
    n_.param("aggregate_point_map/icp_max_corr_dist" ,icp_max_corr_dist_,0.1);
    n_.param("aggregate_point_map/icp_trf_epsilon" ,icp_trf_epsilon_,1e-6);
    n_.param("aggregate_point_map/file_path" ,file_path_ ,std::string("/home/goa/pcl_daten/table/icp/map_"));
    n_.param("aggregate_point_map/save_pc",save_pc_ , false);
    n_.param("aggregate_point_map/save_map",save_map_ ,false);
    n_.param("aggregate_point_map/save_pc_aligned",save_pc_aligned_,false);
    n_.param("aggregate_point_map/save_map_fov" ,save_map_fov_,false);
    n_.param("aggregate_point_map/save_pc_trans" ,save_pc_trans_,false);
    n_.param("aggregate_point_map/voxel_leafsize_x" ,voxel_leafsize_x_, 0.01);
    n_.param("aggregate_point_map/voxel_leafsize_y" ,voxel_leafsize_y_, 0.01);
    n_.param("aggregate_point_map/voxel_leafsize_z" ,voxel_leafsize_z_, 0.01);
    n_.param("aggregate_point_map/r_limit",r_limit_,0.1);
    n_.param("aggregate_point_map/y_limit",y_limit_,0.1);
    n_.param("aggregate_point_map/p_limit",p_limit_,0.1);
    n_.param("aggregate_point_map/distance_limit",distance_limit_,0.3);
    std::stringstream ss;
    ss << file_path_ << "/gt.pcd";
    pcl::io::savePCDFileASCII (ss.str(), ref_map_);
  }

  void pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    boost::timer t;
    pcl::PointCloud<Point>::Ptr pc = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    //ROS_INFO("PointCloudSubCallback");
    StampedTransform transform;
    try
    {
      std::stringstream ss2;
      tf_listener_.waitForTransform("/map", pc_in->header.frame_id, pc_in->header.stamp, ros::Duration(0.1));
      tf_listener_.lookupTransform("/map", pc_in->header.frame_id, pc_in->header.stamp/*ros::Time(0)*/, transform);
      KDL::Frame frame_KDL, frame_KDL_old;
      tf::TransformTFToKDL(transform, frame_KDL);
      tf::TransformTFToKDL(transform_old_, frame_KDL_old);
      double r,p,y;
      frame_KDL.M.GetRPY(r,p,y);
      double r_old,p_old,y_old;
      frame_KDL_old.M.GetRPY(r_old,p_old,y_old);

      /*if(first_)
      {
        if(save_pc_==true)
        {
          ss2 << file_path_ << "/pc_" << ctr_ << ".pcd";
          pcl::io::savePCDFileASCII (ss2.str(), *pc_in);
        }
        //transform.setOrigin(btVector3(-1.3, 1.09, 3.14));
        pcl::VoxelGrid<Point> voxel;
        voxel.setInputCloud(pc_in);
        voxel.setLeafSize(voxel_leafsize_x_,voxel_leafsize_y_,voxel_leafsize_z_);
        voxel.filter(*pc);
        pcl_ros::transformPointCloud(*pc, *pc, transform);
        pc->header.frame_id = "/map";
        pcl::copyPointCloud(*pc, map_);
        map_.header.frame_id="/map";
        point_cloud_pub_aligned_.publish(pc);
        //downsampleMap();
        point_cloud_pub_.publish(map_);
        first_ = false;
        if(save_pc_trans_==true)
        {
          ss2.str("");
          ss2.clear();
          ss2 << file_path_ << "/pc_trans_" << ctr_ << ".pcd";
          pcl::io::savePCDFileASCII (ss2.str(), *pc);
        }
        if(save_map_ ==true)
        {
          std::stringstream ss1;
          ss1 << file_path_ << "/map_" << ctr_ << ".pcd";
          pcl::io::savePCDFileASCII (ss1.str(), map_);
        }
        if(save_pc_aligned_==true)
        {
          ROS_INFO("Saving pc_aligned.");
          std::stringstream ss;
          ss << file_path_ << "/pc_aligned_" << ctr_ << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), *pc);
        }
        ctr_++;
      }
      else*/
      {
        if(fabs(r-r_old) > r_limit_ || fabs(p-p_old) > p_limit_ || fabs(y-y_old) > y_limit_ ||
            transform.getOrigin().distance(transform_old_.getOrigin()) > distance_limit_)
        {
          if(save_pc_==true)
          {
            std::stringstream ss2;
            ss2 << file_path_ << "/pc_" << ctr_ << ".pcd";
            pcl::io::savePCDFileASCII (ss2.str(), *pc_in);
          }
          pcl::VoxelGrid<Point> voxel;
          voxel.setInputCloud(pc_in);
          voxel.setLeafSize(voxel_leafsize_x_,voxel_leafsize_y_,voxel_leafsize_z_);
          voxel.filter(*pc);
          ROS_DEBUG("Registering new point cloud");
          //transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
          //pcl_ros::transformPointCloud ("/map", *(pc.get()), *(pc.get()), tf_listener_);
          pcl_ros::transformPointCloud(*pc, *pc, transform);
          pcl_ros::transformPointCloud(*pc_in, *pc_in, transform);
          pc->header.frame_id = "/map";
          pc_in->header.frame_id = "/map";
          //shiftCloud(pc);
          //shiftCloud(pc_in);
          if(save_pc_trans_==true)
          {
            ss2.str("");
            ss2.clear();
            ss2 << file_path_ << "/pc_trans_" << ctr_ << ".pcd";
            pcl::io::savePCDFileASCII (ss2.str(), *pc);
            ss2.str("");
            ss2.clear();
            ss2 << file_path_ << "/pc_in_trans_" << ctr_ << ".pcd";
            pcl::io::savePCDFileASCII (ss2.str(), *pc_in);
          }

          pcl::PointCloud<Point> pc_aligned;
          Eigen::Matrix4f icp_transform;
          if(doFOVICPUsingReference(pc, pc_aligned, icp_transform))
          {
            std::cout << "icp_transform: " << icp_transform << std::endl;
            transform_old_ = transform;
            pcl::transformPointCloud(*pc_in,*pc_in,icp_transform);
            pc_in->header.frame_id = "/map";
            if(first_)
            {
              pcl::copyPointCloud(pc_aligned, map_);
              map_.header.frame_id="/map";
              first_ = false;
            }
            else
              map_ += pc_aligned;
              //map_ += *pc_in;
            //doICP(pc);
            downsampleMap();
            point_cloud_pub_.publish(map_);
            //pc->header.frame_id = "/map";
            point_cloud_pub_aligned_.publish(pc_in);
          }
          else
            ROS_INFO("FOV not successful");
    
	  ROS_INFO("[aggregate_point_map] ICP took %f s", t.elapsed());

          if(save_map_ ==true)
          {
            std::stringstream ss1;
            ss1 << file_path_ << "/map_" << ctr_ << ".pcd";
            pcl::io::savePCDFileASCII (ss1.str(), map_);
          }
          if(save_pc_aligned_==true)
          {
            ROS_INFO("Saving pc_aligned.");
            std::stringstream ss;
            ss << file_path_ << "/pc_aligned_" << ctr_ << ".pcd";
            pcl::io::savePCDFileASCII (ss.str(), *pc_in);
          }
          ctr_++;
        }
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[aggregate_point_map] : %s",ex.what());
    }
  }


  bool
  doFOVICP(const pcl::PointCloud<Point>::Ptr& pc,
           pcl::PointCloud<Point>& pc_aligned,
           Eigen::Matrix4f& final_transformation)
  {

    cob_env_model_msgs::GetFieldOfView get_fov_srv;
    get_fov_srv.request.target_frame = std::string("/map");
    get_fov_srv.request.stamp = pc->header.stamp;
    if(get_fov_srv_client_.call(get_fov_srv))
    {
      ROS_DEBUG("FOV service called [OK].");
    }
    else
    {
      ROS_WARN("FOV service called [FAILED].");
      return false;
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
    seg_.setInputCloud(map_.makeShared());
    //transformNormals(map_.header.frame_id, pc->header.stamp);
    pcl::PointIndices indices;
    seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, n_origin_t_, n_max_range_t_);
    pcl::PointCloud<Point> frustum;
    pcl::ExtractIndices<Point> extractIndices;
    extractIndices.setInputCloud(map_.makeShared());
    extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractIndices.filter(frustum);
    ROS_DEBUG("[aggregate_point_map] Frustum size: %d", (int)frustum.size());
    if (save_map_fov_==true)
    {
      std::stringstream ss;
      ss << file_path_ << "/map_fov_" << ctr_ << ".pcd";
      pcl::io::savePCDFileASCII (ss.str(), frustum);
    }

    //do ICP
    pcl::IterativeClosestPoint<Point,Point> icp;
    icp.setInputCloud(pc->makeShared());
    //icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    icp.setInputTarget(frustum.makeShared());
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
    icp.setTransformationEpsilon (icp_trf_epsilon_);
    //pcl::PointCloud<Point> pc_aligned;
    icp.align(pc_aligned);
    final_transformation = icp.getFinalTransformation();

    //TODO: check if converged, check fitness score; if not => don't use pc
    ROS_INFO("[aggregate_point_map] ICP has converged: %d\n", icp.hasConverged());
    ROS_INFO("[aggregate_point_map] Fitness score: %f", icp.getFitnessScore());
    return true;
  }


  bool
  doFOVICPUsingReference(const pcl::PointCloud<Point>::Ptr& pc,
           pcl::PointCloud<Point>& pc_aligned,
           Eigen::Matrix4f& final_transformation)
  {

    /*cob_env_model::GetFieldOfView get_fov_srv;
    get_fov_srv.request.target_frame = std::string("/map");
    get_fov_srv.request.stamp = pc->header.stamp;
    if(get_fov_srv_client_.call(get_fov_srv))
    {
      ROS_DEBUG("FOV service called [OK].");
    }
    else
    {
      ROS_WARN("FOV service called [FAILED].");
      return false;
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
    seg_.setInputCloud(ref_map_.makeShared());
    //transformNormals(map_.header.frame_id, pc->header.stamp);
    pcl::PointIndices indices;
    seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, n_origin_t_, n_max_range_t_);
    pcl::PointCloud<Point> frustum;
    pcl::ExtractIndices<Point> extractIndices;
    extractIndices.setInputCloud(ref_map_.makeShared());
    extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractIndices.filter(frustum);
    ROS_DEBUG("[aggregate_point_map] Frustum size: %d", (int)frustum.size());
    if (save_map_fov_==true)
    {
      std::stringstream ss;
      ss << file_path_ << "/map_fov_" << ctr_ << ".pcd";
      pcl::io::savePCDFileASCII (ss.str(), frustum);
    }*/

    //do ICP
    pcl::IterativeClosestPoint<Point,Point> icp;
    icp.setInputCloud(pc->makeShared());
    //icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    icp.setInputTarget(ref_map_.makeShared());
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
    icp.setTransformationEpsilon (icp_trf_epsilon_);
    //pcl::PointCloud<Point> pc_aligned;
    icp.align(pc_aligned);
    final_transformation = icp.getFinalTransformation();

    //TODO: check if converged, check fitness score; if not => don't use pc
    ROS_INFO("[aggregate_point_map] ICP params (max_it, corr_dist, eps): %d,%f,%f\n", icp_max_iterations_, icp_max_corr_dist_, icp_trf_epsilon_);
    ROS_INFO("[aggregate_point_map] ICP has converged: %d\n", icp.hasConverged());
    ROS_INFO("[aggregate_point_map] Fitness score: %f", icp.getFitnessScore());
    return true;
  }


  void doICP(const pcl::PointCloud<Point>::Ptr& pc)
  {
    //TODO: change map2_ to map_, flag for IO operations, publish to callback
    //Open file for timer log
    std::fstream filestr;
    filestr.open("/home/goa/pcl_daten/table/icp/meas.csv", std::fstream::in | std::fstream::out | std::fstream::app);
    boost::timer t;

    //Perform ICP
    pcl::IterativeClosestPoint<Point,Point> icp;
    icp.setInputCloud(pc);
    icp.setInputTarget(map_.makeShared());
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
    icp.setTransformationEpsilon (icp_trf_epsilon_);
    pcl::PointCloud<Point> pc_aligned;
    icp.align(pc_aligned);
    map_ += pc_aligned;

    //TODO: output as ROS_DEBUG
    double time = t.elapsed();
    ROS_DEBUG("ICP has converged: %d\n", icp.hasConverged());
    ROS_DEBUG("Fitness score: %f", icp.getFitnessScore());
    ROS_DEBUG("\tTime: %f", time);

    //TODO: parameter for file path
    if(save_icp_map_==true)
    {
      std::stringstream ss1;
      ss1 << file_path_ << ctr_ << ".pcd";
      pcl::io::savePCDFileASCII (ss1.str(), map_);
    }
    /*pcl::VoxelGrid<Point> vox_filter;
		vox_filter.setInputCloud(map_.makeShared());
		vox_filter.setLeafSize(0.03, 0.03, 0.03);
		vox_filter.filter(map2_);*/
    //point_cloud_pub_.publish(map2_);

    filestr.close();
  }


  void downsampleMap()
  {
    pcl::VoxelGrid<Point> vox_filter;
    vox_filter.setInputCloud(map_.makeShared());
    vox_filter.setLeafSize(voxel_leafsize_x_,voxel_leafsize_y_,voxel_leafsize_z_);
    vox_filter.filter(map_);
  }

  void shiftCloud(const pcl::PointCloud<Point>::Ptr& pc)
  {
    for(int i=0; i<pc->size(); i++)
      pc->points[i].y+=0.15;
      //pc->points[i].z+=0.2;
  }


  ros::NodeHandle n_;
  ros::Time stamp_;


protected:
  ros::Subscriber point_cloud_sub_;		//subscriber for input pc
  ros::Publisher point_cloud_pub_;		//publisher for map
  ros::Publisher point_cloud_pub_aligned_;      //publisher for aligned pc
  ros::Publisher fov_marker_pub_;		//publisher for FOV marker
  ros::ServiceClient get_fov_srv_client_;

  TransformListener tf_listener_;
  StampedTransform transform_old_;

  pcl::PointCloud<Point> map_;	//FOV ICP map
  pcl::PointCloud<Point> ref_map_;  //reference map

  bool first_;

  // Parameters for ICP
  int icp_max_iterations_;
  double icp_max_corr_dist_;
  double icp_trf_epsilon_;

  double voxel_leafsize_x_;
  double voxel_leafsize_y_;
  double voxel_leafsize_z_;

  // Parameters for file saving
  std::string file_path_;
  bool save_pc_;
  bool save_map_;
  bool save_pc_aligned_;
  bool save_icp_fov_pc_;
  bool save_map_fov_;
  bool save_icp_map_;
  bool save_pc_trans_;


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

  ipa_env_model::FieldOfViewSegmentation<Point> seg_;

  int ctr_;

};


PLUGINLIB_DECLARE_CLASS(cob_env_model, AggregatePointMap, AggregatePointMap, nodelet::Nodelet)

