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
 * ToDo: add documentation
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
#include <actionlib/server/simple_action_server.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cob_3d_mapping_common/point_types.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_3d_mapping_point_map/impl/field_of_view_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
//#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetFieldOfView.h>
#include "cob_3d_mapping_msgs/TriggerMappingAction.h"
#include <cob_3d_mapping_msgs/SetReferenceMap.h>
#include <cob_3d_mapping_msgs/GetPointMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "cob_3d_mapping_point_map/point_map.h"

#include "cob_3d_mapping_common/reconfigureable_node.h"
#include <cob_3d_mapping_point_map/aggregate_point_mapConfig.h>




using namespace tf;

//####################
//#### node class ####
class AggregatePointMap : public pcl_ros::PCLNodelet, protected Reconfigurable_Node<cob_3d_mapping_point_map::aggregate_point_mapConfig>
{
  typedef pcl::PointXYZRGB Point;

public:
  // Constructor
  AggregatePointMap()
  : Reconfigurable_Node<cob_3d_mapping_point_map::aggregate_point_mapConfig>("AggregatePointMap"),
    point_map_(&ctr_),
    ctr_(0),
    is_running_(false)
    {
    setReconfigureCallback2(boost::bind(&callback, this, _1, _2), boost::bind(&callback_get, this, _1));
    //setReconfigureCallback(boost::bind(&callback, this, _1, _2));
    }


  // Destructor
  ~AggregatePointMap()
  {
    /// void
  }

  /**
   * @brief callback for dynamic reconfigure
   *
   * everytime the dynamic reconfiguration changes this function will be called
   *
   * @param inst instance of AggregatePointMap which parameters should be changed
   * @param config data of configuration
   * @param level bit descriptor which notifies which parameter changed
   *
   * @return nothing
   */
  static void callback(AggregatePointMap *inst, cob_3d_mapping_point_map::aggregate_point_mapConfig &config, uint32_t level)
  {
    if(!inst)
      return;

    boost::mutex::scoped_lock l1(inst->m_mutex_actionCallback);
    boost::mutex::scoped_lock l2(inst->m_mutex_pointCloudSubCallback);

    inst->point_map_.setICP_maxIterations(config.icp_max_iterations);
    inst->point_map_.setICP_maxCorrDist(config.icp_max_first_corr_dist);
    inst->point_map_.setICP_maxCorrDist(config.icp_max_corr_dist);
    inst->point_map_.setReuse(config.reuse);
    inst->point_map_.setICP_trfEpsilon(config.icp_trf_epsilon);
    inst->file_path_ = config.file_path;
    inst->save_pc_ = inst->save_map_fov_ = inst->save_map_ = config.save;
    inst->save_pc_aligned_ = config.save_pc_aligned;
    inst->save_pc_trans_ = config.save_pc_trans;
    inst->voxel_leafsize_x_ = config.voxel_leafsize_x;
    inst->voxel_leafsize_y_ = config.voxel_leafsize_y;
    inst->voxel_leafsize_z_ = config.voxel_leafsize_z;
    inst->point_map_.setUseReferenceMap(config.use_reference_map);
    inst->use_fov_ = config.use_fov;

    ROS_INFO("callback");
  }

  // callback for dynamic reconfigure
  static void callback_get(AggregatePointMap *inst, cob_3d_mapping_point_map::aggregate_point_mapConfig &config)
  {
    if(!inst)
      return;

    config.icp_max_iterations=inst->point_map_.getICP_maxIterations();
    config.icp_max_first_corr_dist=inst->point_map_.getICP_maxFirstCorrDist();
    config.icp_max_corr_dist=inst->point_map_.getICP_maxCorrDist();
    config.reuse=inst->point_map_.getReuse();
    config.icp_trf_epsilon=inst->point_map_.getICP_trfEpsilon();
    config.file_path=inst->file_path_;
    config.save = inst->save_pc_;
    config.save_pc_aligned = inst->save_pc_aligned_;
    config.save_pc_trans = inst->save_pc_trans_;
    config.voxel_leafsize_x = inst->voxel_leafsize_x_;
    config.voxel_leafsize_y = inst->voxel_leafsize_y_;
    config.voxel_leafsize_z = inst->voxel_leafsize_z_;
    config.use_reference_map=inst->point_map_.getUseReferenceMap();
    config.use_fov = inst->use_fov_;

  }


  /**
   * @brief initializes parameters
   *
   * initializes parameters
   *
   * @return nothing
   */
  void
  onInit()
  {
    PCLNodelet::onInit();
    n_ = getNodeHandle();

    point_cloud_pub_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_map",1);
    point_cloud_pub_aligned_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_aligned",1);
    //fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",10);
    get_fov_srv_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetFieldOfView>("get_fov");
    clear_map_server_ = n_.advertiseService("clear_point_map", &AggregatePointMap::clearMap, this);
    keyframe_trigger_server_ = n_.advertiseService("trigger_keyframe", &AggregatePointMap::onKeyframeCallback, this);
    set_reference_map_server_ = n_.advertiseService("set_reference_map", &AggregatePointMap::setReferenceMap, this);
    get_map_server_ = n_.advertiseService("get_point_map", &AggregatePointMap::getMap, this);
    as_= new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerMappingAction>(n_, "trigger_mapping", boost::bind(&AggregatePointMap::actionCallback, this, _1), false);
    as_->start();

    int icp_max_iterations;
    double icp_max_corr_dist;
    double icp_max_first_corr_dist;
    double icp_trf_epsilon;
    bool use_reference_map, reuse;

    //TODO: values in launch file are not shown in reconfigure_gui; define bounds in gui for realistic values
    n_.param("aggregate_point_map/icp_max_iterations" ,icp_max_iterations ,50);
    n_.param("aggregate_point_map/icp_max_corr_dist" ,icp_max_corr_dist,0.05);;
    n_.param("aggregate_point_map/icp_max_first_corr_dist" ,icp_max_first_corr_dist,0.3);
    n_.param("aggregate_point_map/icp_trf_epsilon" ,icp_trf_epsilon,0.0005);
    n_.param("aggregate_point_map/use_reference_map",use_reference_map,false);
    n_.param("aggregate_point_map/use_fov",use_fov_,false);
    //use_fov_=true;
    n_.param("aggregate_point_map/reuse",reuse,true);
    point_map_.setICP_maxIterations(icp_max_iterations);
    point_map_.setICP_maxCorrDist(icp_max_corr_dist);
    point_map_.setICP_maxFirstCorrDist(icp_max_first_corr_dist);
    point_map_.setICP_trfEpsilon(icp_trf_epsilon);
    point_map_.setUseReferenceMap(use_reference_map);
    point_map_.setReuse(reuse);


    n_.param("aggregate_point_map/file_path" ,file_path_ ,std::string("~/pcl_daten/table/icp/map_"));
    n_.param("aggregate_point_map/save_pc",save_pc_ , false);
    n_.param("aggregate_point_map/save_map",save_map_ ,false);
    n_.param("aggregate_point_map/save_pc_aligned",save_pc_aligned_,false);
    n_.param("aggregate_point_map/save_map_fov" ,save_map_fov_,false);
    n_.param("aggregate_point_map/save_pc_trans" ,save_pc_trans_,false);
    n_.param("aggregate_point_map/voxel_leafsize_x" ,voxel_leafsize_x_, 0.05);
    n_.param("aggregate_point_map/voxel_leafsize_y" ,voxel_leafsize_y_, 0.05);
    n_.param("aggregate_point_map/voxel_leafsize_z" ,voxel_leafsize_z_, 0.05);
    /*std::stringstream ss;
    ss << file_path_ << "/gt.pcd";
    /pcl::io::savePCDFileASCII (ss.str(), ref_map_);*/
    if(use_reference_map)
    {
      std::string ref_map_path;
      n_.param("aggregate_point_map/file_path_ref_map" ,ref_map_path ,std::string("/home/goa/pcl_daten/kitchen_ground_truth/whole_kitchen.pcd"));
      pcl::io::loadPCDFile(ref_map_path, *point_map_.getRefMap());
    }
  }

  /**
   * @brief callback for point cloud subroutine
   *
   * callback for point cloud subroutine which stores the point cloud
   * for further calculation
   *
   * @param pc_in  new point cloud
   *
   * @return nothing
   */
  void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    pc_in_ = *pc_in;
  }

  /**
   * @brief callback for point cloud subroutine
   *
   * callback for keyframe subroutine which loads in the first step
   * the unexact transformation from the laser sensors and calibrates the
   * input cloud from the 3d camera. This already transformed data will be
   * used to build a 3d map either aligned to the first frame or to an
   * existing map. Additionally debug output to *.pcd files are possible.
   *
   * @param req  not used
   * @param res  not used
   *
   * @return nothing
   */
  bool onKeyframeCallback(cob_srvs::Trigger::Request &req,
                          cob_srvs::Trigger::Response &res)
  {
    res.success.data = false;

    if(!is_running_)
      return true;

    boost::mutex::scoped_lock l(m_mutex_pointCloudSubCallback);

    boost::timer t;
    pcl::PointCloud<Point>::Ptr pc = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    ROS_INFO("PointCloudSubCallback");
    //TODO: make separate node for key frame selection, trigger registration
    StampedTransform transform;
    try
    {
      std::stringstream ss2;
      tf_listener_.waitForTransform("/map", pc_in_.header.frame_id, pc_in_.header.stamp, ros::Duration(0.1));
      tf_listener_.lookupTransform("/map", pc_in_.header.frame_id, pc_in_.header.stamp/*ros::Time(0)*/, transform);

      if(save_pc_==true)
      {
        std::stringstream ss2;
        ss2 << file_path_ << "/pc_" << ctr_ << ".pcd";
        pcl::io::savePCDFileASCII (ss2.str(), pc_in_);
      }
      pcl_ros::transformPointCloud(pc_in_, pc_in_, transform);
      pc_in_.header.frame_id = "/map";
      pcl::VoxelGrid<Point> voxel;
      voxel.setInputCloud(pc_in_.makeShared());
      voxel.setLeafSize(voxel_leafsize_x_,voxel_leafsize_y_,voxel_leafsize_z_);
      voxel.filter(*pc);
      ROS_DEBUG("Registering new point cloud");
      //shiftCloud(pc);
      //shiftCloud(pc_in_);
      if(save_pc_trans_==true)
      {
        ss2.str("");
        ss2.clear();
        ss2 << file_path_ << "/pc_trans_" << ctr_ << ".pcd";
        pcl::io::savePCDFileASCII (ss2.str(), *pc);
        ss2.str("");
        ss2.clear();
        ss2 << file_path_ << "/pc_in_trans_" << ctr_ << ".pcd";
        pcl::io::savePCDFileASCII (ss2.str(), pc_in_);
      }

      cob_3d_mapping_msgs::GetFieldOfView get_fov_srv;
      if(use_fov_) {

        Eigen::Vector3d n_up;
        Eigen::Vector3d n_down;
        Eigen::Vector3d n_right;
        Eigen::Vector3d n_left;
        Eigen::Vector3d n_origin;
        Eigen::Vector3d n_max_range;

        get_fov_srv.request.target_frame = std::string("/map");
        get_fov_srv.request.stamp = pc->header.stamp;
        if(get_fov_srv_client_.call(get_fov_srv))
        {
          ROS_DEBUG("FOV service called [OK].");

          n_up(0) = get_fov_srv.response.fov.points[0].x;
          n_up(1) = get_fov_srv.response.fov.points[0].y;
          n_up(2) = get_fov_srv.response.fov.points[0].z;
          n_down(0) = get_fov_srv.response.fov.points[1].x;
          n_down(1) = get_fov_srv.response.fov.points[1].y;
          n_down(2) = get_fov_srv.response.fov.points[1].z;
          n_right(0) = get_fov_srv.response.fov.points[2].x;
          n_right(1) = get_fov_srv.response.fov.points[2].y;
          n_right(2) = get_fov_srv.response.fov.points[2].z;
          n_left(0) = get_fov_srv.response.fov.points[3].x;
          n_left(1) = get_fov_srv.response.fov.points[3].y;
          n_left(2) = get_fov_srv.response.fov.points[3].z;
          n_origin(0) = get_fov_srv.response.fov.points[4].x;
          n_origin(1) = get_fov_srv.response.fov.points[4].y;
          n_origin(2) = get_fov_srv.response.fov.points[4].z;
          n_max_range(0) = get_fov_srv.response.fov.points[5].x;
          n_max_range(1) = get_fov_srv.response.fov.points[5].y;
          n_max_range(2) = get_fov_srv.response.fov.points[5].z;

          //segment FOV
          ipa_env_model::FieldOfViewSegmentation<Point> seg_;

          seg_.setInputCloud(point_map_.getUsedMap());
          //transformNormals(map_.header.frame_id, pc->header.stamp);
          pcl::PointIndices indices;
          seg_.segment(indices, n_up, n_down, n_right, n_left, n_origin, n_max_range);
          pcl::PointCloud<Point>::Ptr frustum = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
          pcl::ExtractIndices<Point> extractIndices;
          extractIndices.setInputCloud(point_map_.getUsedMap());
          extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
          extractIndices.filter(*frustum);
          ROS_DEBUG("[aggregate_point_map] Frustum size: %d", (int)frustum->size());

          point_map_.setUsedMapToRegistrate(frustum);
        }
        else
        {
          ROS_WARN("FOV service called [FAILED].");
          return false;
        }
      }
      else
        point_map_.setUsedMapToRegistrate(point_map_.getUsedMap());

      if(point_map_.compute(pc_in_.makeShared(), pc)) {
        downsampleMap();
        point_cloud_pub_.publish(*point_map_.getMap());
        //pc->header.frame_id = "/map";
        point_cloud_pub_aligned_.publish(pc_in_);
      }
      else
        ROS_WARN("ICP not successful");

      ROS_DEBUG("[aggregate_point_map] ICP took %f s", t.elapsed());

      if(save_map_ ==true)
      {
        std::stringstream ss1;
        ss1 << file_path_ << "/map_" << ctr_ << ".pcd";
        pcl::io::savePCDFileASCII (ss1.str(), *point_map_.getMap());
      }
      if(save_pc_aligned_==true)
      {
        std::stringstream ss;
        ss << file_path_ << "/pc_aligned_" << ctr_ << ".pcd";
        pcl::io::savePCDFileASCII (ss.str(), pc_in_);
      }
      ctr_++;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[aggregate_point_map] : %s",ex.what());
      return false;
    }

    res.success.data = true;
    return true;
  }


  /**
   * @brief action callback
   *
   * default action callback to start or stop node
   *
   * @param goal settings
   *
   * @return nothing
   */
  void
  actionCallback(const cob_3d_mapping_msgs::TriggerMappingGoalConstPtr &goal)
  {
    boost::mutex::scoped_lock l(m_mutex_actionCallback);

    cob_3d_mapping_msgs::TriggerMappingResult result;
    if(goal->start && !is_running_)
    {
      point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &AggregatePointMap::pointCloudSubCallback, this);
      is_running_ = true;
    }
    else if(!goal->start && is_running_)
    {
      point_cloud_sub_.shutdown();
      //first_ = true;
      is_running_ = false;
    }
    as_->setSucceeded(result);
  }

  /**
   * @brief clears map
   *
   * deletes 3d map of the environment
   *
   * @param req not needed
   * @param res not needed
   *
   * @return nothing
   */
  bool
  clearMap(cob_srvs::Trigger::Request &req,
           cob_srvs::Trigger::Response &res)
  {
    //TODO: add mutex
    ROS_INFO("Clearing point map...");
    point_map_.clearMap();
    return true;
  }

  /**
   * @brief service callback for GetPointMap service
   *
   * Fills the service response of the GetPointMap service with the current point map
   *
   * @param req request to send map
   * @param res the current point map
   *
   * @return nothing
   */
  bool
  getMap(cob_3d_mapping_msgs::GetPointMap::Request &req,
         cob_3d_mapping_msgs::GetPointMap::Response &res)
  {
    pcl::toROSMsg(*(point_map_.getMap()), res.map);
    return true;
  }

  /**
   * @brief sets reference map
   *
   * sets the 3d map representing the environment which is used to align new frames
   *
   * @param req containing reference map
   * @param res not needed
   *
   * @return nothing
   */
  bool
  setReferenceMap(cob_3d_mapping_msgs::SetReferenceMap::Request &req,
                  cob_3d_mapping_msgs::SetReferenceMap::Response &res)
  {
    pcl::fromROSMsg(req.map, *point_map_.getRefMap());
    return true;
  }


  /**
   * @brief downsamples the map
   *
   * downsamples the map using the voxel_lefsize parameters to voxelize
   *
   * @return nothing
   */
  void
  downsampleMap()
  {
    pcl::VoxelGrid<Point> vox_filter;
    vox_filter.setInputCloud(point_map_.getMap()->makeShared());
    vox_filter.setLeafSize(voxel_leafsize_x_,voxel_leafsize_y_,voxel_leafsize_z_);
    vox_filter.filter(*point_map_.getMap());
  }

  /*void
  shiftCloud(const pcl::PointCloud<Point>::Ptr& pc)
  {
    for(unsigned int i=0; i<pc->size(); i++)
    {
      //pc->points[i].y+=0.15;
      pc->points[i].z+=0.2;
      pc->points[i].x-=0.25;
      //if(ctr_==0 || ctr_==1) pc->points[i].x-=0.25;
    }
  }*/


  ros::NodeHandle n_;
  ros::Time stamp_;


protected:
  ros::Subscriber point_cloud_sub_;		//subscriber for input pc
  ros::Publisher point_cloud_pub_;		//publisher for map
  ros::Publisher point_cloud_pub_aligned_;      //publisher for aligned pc
  //ros::Publisher fov_marker_pub_;		//publisher for FOV marker
  ros::ServiceClient get_fov_srv_client_;
  ros::ServiceServer clear_map_server_;
  ros::ServiceServer keyframe_trigger_server_;
  ros::ServiceServer set_reference_map_server_;
  ros::ServiceServer get_map_server_;
  actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerMappingAction>* as_;

  TransformListener tf_listener_;

  PointMap point_map_;

  int ctr_;
  bool is_running_;
  bool use_fov_;               /// if map should be cut by frustum (reduce input information)

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
  bool save_pc_trans_;


  pcl::PointCloud<Point> pc_in_;

  boost::mutex m_mutex_pointCloudSubCallback, m_mutex_actionCallback;

};

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_point_map, AggregatePointMap, AggregatePointMap, nodelet::Nodelet)

