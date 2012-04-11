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
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pluginlib/class_list_macros.h>
//#include <pcl/registration/icp.h>
//#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_common/reconfigureable_node.h>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/pcl_nodelet.h>
//#include <pcl/filters/extract_indices.h>
//#include <visualization_msgs/Marker.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetFieldOfView.h>
#include <cob_3d_mapping_msgs/TriggerMappingAction.h>
#include <cob_3d_mapping_msgs/SetReferenceMap.h>
#include <cob_3d_mapping_msgs/GetPointMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
//#include <boost/numeric/ublas/matrix.hpp>

#include "cob_3d_mapping_point_map/impl/field_of_view_segmentation.hpp"
#include "cob_3d_mapping_point_map/point_map.h"
#include "cob_3d_mapping_point_map/point_map_nodeletConfig.h"

//#include <sensor_msgs/CameraInfo.h>

//#include "../../../cob_3d_registration/common/include/registration/general_registration.h"
//#include "../../../cob_3d_registration/common/include/registration/registration_info.h"


//using namespace tf;

typedef message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> TimeSync;
typedef boost::shared_ptr<TimeSync> TimeSyncPtr;

//####################
//#### node class ####
class PointMapNodelet : public pcl_ros::PCLNodelet, protected Reconfigurable_Node<cob_3d_mapping_point_map::point_map_nodeletConfig>
{
  typedef pcl::PointXYZRGB Point;

public:
  // Constructor
  PointMapNodelet()
  : Reconfigurable_Node<cob_3d_mapping_point_map::point_map_nodeletConfig>("PointMapNodelet"),
    ctr_(0),
    is_running_(false),
    map_frame_id_("/map")
    {

    map_.header.frame_id = map_frame_id_;

    /*reg_ = new Registration_Infobased<Point>();
    ((Registration_Infobased<Point>*)reg_)->setThresholdDiff(0.06);
    ((Registration_Infobased<Point>*)reg_)->setThresholdStep(0.06);
    ((Registration_Infobased<Point>*)reg_)->setMinInfo(1);
    ((Registration_Infobased<Point>*)reg_)->setMaxInfo(17);
    ((Registration_Infobased<Point>*)reg_)->SetAlwaysRelevantChanges(true);*/

    setReconfigureCallback2(boost::bind(&callback, this, _1, _2), boost::bind(&callback_get, this, _1));
    setReconfigureCallback(boost::bind(&callback, this, _1, _2));
    }


  // Destructor
  ~PointMapNodelet()
  {
    //delete reg_;
  }

  /**
   * @brief callback for dynamic reconfigure
   *
   * everytime the dynamic reconfiguration changes this function will be called
   *
   * @param inst instance of PointMapNodelet which parameters should be changed
   * @param config data of configuration
   * @param level bit descriptor which notifies which parameter changed
   *
   * @return nothing
   */
  static void callback(PointMapNodelet *inst, cob_3d_mapping_point_map::point_map_nodeletConfig &config, uint32_t level)
  {
    if(!inst)
      return;

    //boost::mutex::scoped_lock l1(inst->m_mutex_actionCallback);
    //boost::mutex::scoped_lock l2(inst->m_mutex_pointCloudSubCallback);

    inst->file_path_ = config.file_path;
    inst->save_ = config.save;
    inst->voxel_leafsize_ = config.voxel_leafsize;
  }

  // callback for dynamic reconfigure
  static void callback_get(PointMapNodelet *inst, cob_3d_mapping_point_map::point_map_nodeletConfig &config)
  {
    if(!inst)
      return;

    config.file_path=inst->file_path_;
    config.save = inst->save_;
    config.voxel_leafsize = inst->voxel_leafsize_;

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

    //sync_ = TimeSyncPtr(new TimeSync(point_cloud_sub_, transform_sub_, 10));
    //sync_->registerCallback(boost::bind(&PointMapNodelet::registerCallback, this, _1, _2));
    map_pub_ = n_.advertise<pcl::PointCloud<Point> >("map",1);
    clear_map_server_ = n_.advertiseService("clear_point_map", &PointMapNodelet::clearMap, this);
    get_map_server_ = n_.advertiseService("get_point_map", &PointMapNodelet::getMap, this);
    as_= new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerMappingAction>(n_, "trigger_mapping", boost::bind(&PointMapNodelet::actionCallback, this, _1), false);
    as_->start();


    n_.param("aggregate_point_map/file_path" ,file_path_ ,std::string("~/pcl_daten/table/icp/map_"));
    n_.param("aggregate_point_map/save_",save_ , false);
    //n_.param("aggregate_point_map/save_map",save_ ,false);
    n_.param("aggregate_point_map/voxel_leafsize" ,voxel_leafsize_, 0.03);
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
  /*void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    boost::mutex::scoped_lock l(m_mutex_pointCloudSubCallback);
    pc_in_ = *pc_in;
  }*/


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
  void
  registerCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
  {
    ROS_INFO("PointCloudSubCallback");
    pcl::PointCloud<Point> pc;
    pcl::fromROSMsg(*pc_msg, pc);
    pcl::PointCloud<Point>::Ptr pc_ptr = pc.makeShared();
    if (pc.size() < 1)
      return;
    //tf::TransformTFToEigen(trf_reg, af_reg);
    //pcl::transformPointCloud(pc, pc, af.affine());

    //boost::mutex::scoped_lock l(m_mutex_pointCloudSubCallback);

    //boost::timer t;
    //pcl::PointCloud<Point>::Ptr pc = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    //TODO: apply registration trafo
    tf::StampedTransform trf_map;
    try
    {
      std::stringstream ss2;
      tf_listener_.waitForTransform(map_frame_id_, pc.header.frame_id, pc.header.stamp, ros::Duration(0.1));
      tf_listener_.lookupTransform(map_frame_id_, pc.header.frame_id, pc.header.stamp/*ros::Time(0)*/, trf_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[aggregate_point_map] : %s",ex.what());
      return;
    }
    Eigen::Affine3d af;
    tf::TransformTFToEigen(trf_map,af);
    Eigen::Matrix4f trf = af.matrix().cast<float>();
    pcl::transformPointCloud(pc, pc, trf);
    pc.header.frame_id = map_frame_id_;
    //map_.header.frame_id="/map";

    updateMap(pc);
    map_pub_.publish(map_);

    //ROS_DEBUG("[aggregate_point_map] ICP took %f s", t.elapsed());

    if(save_)
    {
      std::stringstream ss1;
      ss1 << file_path_ << "/map_" << ctr_ << ".pcd";
      pcl::io::savePCDFileASCII (ss1.str(), map_);
    }
    ctr_++;
    return;
  }

  void
  updateMap(pcl::PointCloud<Point>& pc)
  {
    map_ += pc;
    downsampleMap();
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
    //boost::mutex::scoped_lock l(m_mutex_actionCallback);

    cob_3d_mapping_msgs::TriggerMappingResult result;
    if(goal->start && !is_running_)
    {
      ROS_INFO("Starting mapping...");
      point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &PointMapNodelet::registerCallback, this);
      //point_cloud_sub_.subscribe(n_, "point_cloud2", 1);
      //transform_sub_.subscribe(n_, "transform_reg", 1);
      is_running_ = true;
    }
    else if(!goal->start && is_running_)
    {
      ROS_INFO("Stopping mapping...");
      point_cloud_sub_.shutdown();
      //transform_sub_.unsubscribe();
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
    map_.clear();
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
    pcl::toROSMsg(map_, res.map);
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
    ROS_WARN("not needed");
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
    vox_filter.setInputCloud(map_.makeShared());
    vox_filter.setLeafSize(voxel_leafsize_,voxel_leafsize_,voxel_leafsize_);
    vox_filter.filter(map_);
  }


  ros::NodeHandle n_;


protected:
  //message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
  message_filters::Subscriber<geometry_msgs::TransformStamped> transform_sub_;
  TimeSyncPtr sync_;
  ros::Subscriber point_cloud_sub_;		//subscriber for input pc
  //ros::Subscriber camera_info_sub_;             //subscriber for input pc
  ros::Publisher map_pub_;		//publisher for map
  //ros::Publisher point_cloud_pub_aligned_;      //publisher for aligned pc
  //ros::Publisher fov_marker_pub_;		//publisher for FOV marker
  ros::ServiceClient get_fov_srv_client_;
  ros::ServiceServer clear_map_server_;
  //ros::ServiceServer keyframe_trigger_server_;
  //ros::ServiceServer set_reference_map_server_;
  ros::ServiceServer get_map_server_;
  actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerMappingAction>* as_;

  tf::TransformListener tf_listener_;

  int ctr_;
  bool is_running_;

  // Parameters from launch file
  std::string file_path_;
  bool save_;
  std::string map_frame_id_;
  double voxel_leafsize_;

  pcl::PointCloud<Point> map_;

};

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_point_map, PointMapNodelet, PointMapNodelet, nodelet::Nodelet)

