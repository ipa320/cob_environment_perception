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
#include "cob_env_model_msgs/TriggerMappingAction.h"
#include <cob_env_model_msgs/SetReferenceMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "cob_env_model/map/point_map.h"

#include "reconfigureable_node.h"
#include <cob_env_model/aggregate_point_mapConfig.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model_msgs/GetFieldOfView.h>
#include "cob_env_model_msgs/TriggerMappingAction.h"
#include <cob_env_model_msgs/SetReferenceMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### node class ####
class AggregatePointMap : public pcl_ros::PCLNodelet, protected Reconfigurable_Node<cob_env_model::aggregate_point_mapConfig>
{
  typedef pcl::PointXYZRGB Point;

public:
  // Constructor
  AggregatePointMap()
  : Reconfigurable_Node<cob_env_model::aggregate_point_mapConfig>("AggregatePointMap"),
    point_map_(&ctr_),
    ctr_(0),
    is_running_(false)
    {
    //TODO: remove, get ref map by service, write client, check if map loaded
    pcl::io::loadPCDFile("~/pcl_daten/kitchen_ground_truth/whole_kitchen.pcd", *point_map_.getRefMap());

    setReconfigureCallback(boost::bind(&callback, this, _1, _2));
    }


  // Destructor
  ~AggregatePointMap()
  {
    /// void
  }

  // callback for dynamic reconfigure
  static void callback(AggregatePointMap *inst, cob_env_model::aggregate_point_mapConfig &config, uint32_t level)
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
    inst->r_limit_ = config.r_limit;
    inst->p_limit_ = config.p_limit;
    inst->y_limit_ = config.y_limit;
    inst->distance_limit_ = config.distance_limit;
    inst->point_map_.setUseReferenceMap(config.use_reference_map);

  }

  void
  onInit()
  {
    PCLNodelet::onInit();
    n_ = getNodeHandle();

    point_cloud_pub_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_map",1);
    point_cloud_pub_aligned_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_aligned",1);
    //point_cloud_pub_aligned2_ = n_.advertise<pcl::PointCloud<Point> >("pc_aligned_and_boundary",1);
    fov_marker_pub_ = n_.advertise<visualization_msgs::Marker>("fov_marker",10);
    get_fov_srv_client_ = n_.serviceClient<cob_env_model_msgs::GetFieldOfView>("get_fov");
    clear_map_server_ = n_.advertiseService("clear_point_map", &AggregatePointMap::clearMap, this);
    set_reference_map_server_ = n_.advertiseService("set_reference_map", &AggregatePointMap::setReferenceMap, this);
    as_= new actionlib::SimpleActionServer<cob_env_model_msgs::TriggerMappingAction>(n_, "trigger_mapping", boost::bind(&AggregatePointMap::actionCallback, this, _1), false);
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
    n_.param("aggregate_point_map/r_limit",r_limit_,0.1);
    n_.param("aggregate_point_map/y_limit",y_limit_,0.1);
    n_.param("aggregate_point_map/p_limit",p_limit_,0.1);
    n_.param("aggregate_point_map/distance_limit",distance_limit_,0.3);
    std::stringstream ss;
    ss << file_path_ << "/gt.pcd";
    //pcl::io::savePCDFileASCII (ss.str(), ref_map_);
  }

  void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    boost::mutex::scoped_lock l(m_mutex_pointCloudSubCallback);

    boost::timer t;
    pcl::PointCloud<Point>::Ptr pc = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    //ROS_INFO("PointCloudSubCallback");
    //TODO: make separate node for key frame selection, trigger registration
    StampedTransform transform;
    try
    {
      std::stringstream ss2;
      tf_listener_.waitForTransform("/map", pc_in->header.frame_id, pc_in->header.stamp, ros::Duration(0.1));
      tf_listener_.lookupTransform("/map", pc_in->header.frame_id, pc_in->header.stamp/*ros::Time(0)*/, transform);
      KDL::Frame frame_KDL, frame_KDL_old;
      tf::TransformTFToKDL(transform, frame_KDL);
      tf::TransformTFToKDL(point_map_.getOldTransform(), frame_KDL_old);
      double r,p,y;
      frame_KDL.M.GetRPY(r,p,y);
      double r_old,p_old,y_old;
      frame_KDL_old.M.GetRPY(r_old,p_old,y_old);

      if(point_map_.isFirst() || fabs(r-r_old) > r_limit_ || fabs(p-p_old) > p_limit_ || fabs(y-y_old) > y_limit_ ||
          transform.getOrigin().distance(point_map_.getOldTransform().getOrigin()) > distance_limit_)
      {
        if(save_pc_==true)
        {
          std::stringstream ss2;
          ss2 << file_path_ << "/pc_" << ctr_ << ".pcd";
          pcl::io::savePCDFileASCII (ss2.str(), *pc_in);
        }
        pcl_ros::transformPointCloud(*pc_in, *pc_in, transform);
        pc_in->header.frame_id = "/map";
        pcl::VoxelGrid<Point> voxel;
        voxel.setInputCloud(pc_in);
        voxel.setLeafSize(voxel_leafsize_x_,voxel_leafsize_y_,voxel_leafsize_z_);
        voxel.filter(*pc);
        ROS_DEBUG("Registering new point cloud");
        //transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
        //pcl_ros::transformPointCloud ("/map", *(pc.get()), *(pc.get()), tf_listener_);
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

        cob_env_model_msgs::GetFieldOfView get_fov_srv;
        if(!point_map_.getUseReferenceMap()) {
          get_fov_srv.request.target_frame = std::string("/map");
          get_fov_srv.request.stamp = pc->header.stamp;
          if(get_fov_srv_client_.call(get_fov_srv))
          {
            ROS_DEBUG("FOV service called [OK].");
          }
          else
          {
            ROS_WARN("FOV service called [FAILED].");
            return;
          }

          if(point_map_.compute(pc_in, pc, transform, &get_fov_srv)) {
            downsampleMap();
            point_cloud_pub_.publish(*point_map_.getMap());
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
            pcl::io::savePCDFileASCII (ss1.str(), *point_map_.getMap());
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

  void
  actionCallback(const cob_env_model_msgs::TriggerMappingGoalConstPtr &goal)
  {
    boost::mutex::scoped_lock l(m_mutex_actionCallback);

    cob_env_model_msgs::TriggerMappingResult result;
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

  bool
  clearMap(cob_srvs::Trigger::Request &req,
           cob_srvs::Trigger::Response &res)
  {
    //TODO: add mutex
    ROS_INFO("Clearing point map...");
    point_map_.clearMap();
    return true;
  }

  bool
  setReferenceMap(cob_env_model_msgs::SetReferenceMap::Request &req,
                  cob_env_model_msgs::SetReferenceMap::Response &res)
  {
    pcl::fromROSMsg(req.map, *point_map_.getRefMap());
    return true;
  }

  void downsampleMap()
  {
    pcl::VoxelGrid<Point> vox_filter;
    vox_filter.setInputCloud(point_map_.getMap()->makeShared());
    vox_filter.setLeafSize(voxel_leafsize_x_,voxel_leafsize_y_,voxel_leafsize_z_);
    vox_filter.filter(*point_map_.getMap());
  }

  void
  shiftCloud(const pcl::PointCloud<Point>::Ptr& pc)
  {
    for(unsigned int i=0; i<pc->size(); i++)
    {
      //pc->points[i].y+=0.15;
      pc->points[i].z+=0.2;
      pc->points[i].x-=0.25;
      //if(ctr_==0 || ctr_==1) pc->points[i].x-=0.25;
    }
  }


  ros::NodeHandle n_;
  ros::Time stamp_;


protected:
  ros::Subscriber point_cloud_sub_;		//subscriber for input pc
  ros::Publisher point_cloud_pub_;		//publisher for map
  ros::Publisher point_cloud_pub_aligned_;      //publisher for aligned pc
  ros::Publisher fov_marker_pub_;		//publisher for FOV marker
  ros::ServiceClient get_fov_srv_client_;
  ros::ServiceServer clear_map_server_;
  ros::ServiceServer set_reference_map_server_;
  actionlib::SimpleActionServer<cob_env_model_msgs::TriggerMappingAction>* as_;

  TransformListener tf_listener_;

  PointMap point_map_;

  bool is_running_;

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

  double y_limit_;
  double distance_limit_;
  double r_limit_;
  double p_limit_;

  int ctr_;

  boost::mutex m_mutex_pointCloudSubCallback, m_mutex_actionCallback;

};

//TODO: move to common/src
double testPointMap(pcl::PointCloud<pcl::PointXYZRGB> pc, pcl::PointCloud<pcl::PointXYZRGB> _pc_in, const pcl::PointCloud<pcl::PointXYZRGB> * const ref_map, int max_it, double max_dist, double trf, bool reset, const std::string &fn_out, double &time) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in = _pc_in.makeShared();

  setVerbosityLevel(pcl::console::L_DEBUG);

  static PointMap point_map_(0);

  if(reset)
    point_map_ = PointMap(0);

  if(!point_map_.getUseReferenceMap()) {
    *point_map_.getRefMap() = *ref_map;
    point_map_.setUseReferenceMap(true);

    point_map_.setICP_maxIterations(max_it);
    point_map_.setICP_maxCorrDist(max_dist);
    point_map_.setICP_maxFirstCorrDist(0.3);
    point_map_.setICP_trfEpsilon(trf);
    point_map_.setReuse(true);
  }

  /*{
    pcl::VoxelGrid<pcl::PointXYZ> vox_filter;
    vox_filter.setInputCloud(pc_in);
    vox_filter.setLeafSize(0.05,0.05,0.05);
    vox_filter.filter(_pc_in);
  }*/
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> vox_filter;
    vox_filter.setInputCloud(pc.makeShared());
    vox_filter.setLeafSize(0.05,0.05,0.05);
    vox_filter.filter(pc);
  }

  StampedTransform transform;
  if(point_map_.compute(pc_in, pc.makeShared(), transform, NULL)) {

    if(fn_out.size()>0) {
      pcl::VoxelGrid<pcl::PointXYZRGB> vox_filter;
      vox_filter.setInputCloud(point_map_.getMap()->makeShared());
      vox_filter.setLeafSize(0.05,0.05,0.05);
      vox_filter.filter(*point_map_.getMap());

      pcl::io::savePCDFileASCII (fn_out.c_str(), *point_map_.getMap());
    }

    //ROS_INFO("Success");
  }

  time=point_map_.getComputionTime();

  return point_map_.getFitness();
}


/*int main (int argc, char** argv)
{
  ros::init (argc, argv, "aggregate_point_map");

  AggregatePointMap apm;

  //preload files
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> files,files_in;
  pcl::PointCloud<pcl::PointXYZRGB> ref_map;

  pcl::io::loadPCDFile("/home/goa-jh/bagfiles/kitchen_real_empty/icp/pc_in_trans_0.pcd", ref_map);
  for(int loopCount=0; loopCount<9; loopCount++) {
    std::stringstream ss1;
    ss1 << "/home/goa-jh/bagfiles/kitchen_real_empty/icp/pc_trans_" << loopCount << ".pcd";
    std::stringstream ss2;
    ss2 << "/home/goa-jh/bagfiles/kitchen_real_empty/icp/pc_in_trans_" << loopCount << ".pcd";

    pcl::PointCloud<pcl::PointXYZRGB> _pc_in, _pc;
    pcl::io::loadPCDFile(ss2.str().c_str(), _pc_in);
    files_in.push_back(_pc_in.makeShared());

    pcl::io::loadPCDFile(ss1.str().c_str(), _pc);
    files.push_back(_pc.makeShared());
  }
/*
  {
    pcl::VoxelGrid<pcl::PointXYZ> vox_filter;
    vox_filter.setInputCloud(ref_map.makeShared());
    vox_filter.setLeafSize(0.5,0.5,0.5);
    vox_filter.filter(ref_map);
  }


  int max_it=50;
  double max_dist=0.05, trf=0.0005;//1e-6;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep();

    FILE *f=fopen("/home/goa-jh/log_apm_real_reuse.txt","w");
    //for(max_it=50; max_it<1000; max_it+=10)
    {
      //for(max_dist=0.05; max_dist<1.6; max_dist+=0.1) {
      //for(max_dist=0.1; max_dist<1.3; max_dist+=0.2)
      {
        //for(trf=1e-8; trf<1e-4; trf*=10)
        {

          double res=0;
          double time=0.;
          for(int loopCount=0; loopCount<files.size(); loopCount++) {
            double t=0.;

            std::stringstream ss3;
            ss3 << "/home/goa-jh/bagfiles/myown/mapo_"<<max_it<<"_"<<max_dist<<"_" << loopCount << ".pcd";

            res+=testPointMap(*files[loopCount],*files_in[loopCount], &ref_map, max_it, max_dist, trf, loopCount==0,ss3.str(),t);
            time+=t;

            if(!ros::ok())
               return 0;
          }

          std::stringstream ss;
          ss<<max_it<<"\t"<<max_dist<<"\t"<<trf<<"\t"<<time<<"\t"<<res<<"\r\n";
          fputs(ss.str().c_str(), f);

        }

        fflush(f);
      }

    }
    fclose(f);

    return 0;

  }
}*/

PLUGINLIB_DECLARE_CLASS(cob_env_model, AggregatePointMap, AggregatePointMap, nodelet::Nodelet)

