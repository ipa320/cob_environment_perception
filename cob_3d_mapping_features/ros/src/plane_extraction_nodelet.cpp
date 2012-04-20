/****************************************************************
 *
 * Copyright (c) 2011
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
 * Date of creation: 02/2011
 * ToDo:
 *
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
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
//#include <tf_conversions/tf_kdl.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/server/simple_action_server.h>

#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/concave_hull.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include "pcl/filters/voxel_grid.h"
#include <Eigen/StdVector>

//#include <cob_3d_mapping_common/reconfigureable_node.h>
#include <dynamic_reconfigure/server.h>
#include <cob_3d_mapping_features/plane_extraction_nodeletConfig.h>


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_3d_mapping_msgs/GetPlane.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "cob_3d_mapping_features/plane_extraction.h"
#include "cob_3d_mapping_msgs/PlaneExtractionAction.h"


using namespace tf;
using namespace cob_3d_mapping_features;

//####################
//#### nodelet class ####
class PlaneExtractionNodelet : public pcl_ros::PCLNodelet//, protected Reconfigurable_Node<plane_extraction_nodeletConfig>
{
public:
  typedef pcl::PointXYZRGB Point;
  // Constructor
  PlaneExtractionNodelet()
  : as_(0),
    mode_action_(false)
  {
    ctr_ = 0;
    min_cluster_size_ = 300;

    passthrough_min_z_ = -0.1;
    passthrough_max_z_ = 2.0;
    vox_leaf_size_ = 0.03;
    cluster_tolerance_ = 0.05;
    max_cluster_size_ = 60000;
    radius_ = 0.02;
    optimize_coefficients_ = true;
    normal_distance_weight_ = 0.04;
    max_iterations_ = 99;
    distance_threshold_ = 0.02;

    //setReconfigureCallback(boost::bind(&callback, this, _1, _2));
  }

  // Destructor
  ~PlaneExtractionNodelet()
  {
    /// void
    if(as_) delete as_;
  }

  void dynReconfCallback(cob_3d_mapping_features::plane_extraction_nodeletConfig &config, uint32_t level)
  {
    file_path_ = config.file_path;
    save_to_file_ = config.save_to_file;
    plane_constraint_ = config.plane_constraint;
    mode_action_ = config.mode_action;
    target_frame_ = config.target_frame;

    pe.setFilePath(file_path_);
    pe.setSaveToFile(save_to_file_);
    pe.setPlaneConstraint((PlaneConstraint)plane_constraint_);
    pe.setClusteringParamClusterTolerance (cluster_tolerance_);
    pe.setClusteringParamMinClusterSize (min_cluster_size_);
    pe.setClusteringParamMaxClusterSize (max_cluster_size_);
    pe.setNormalEstimationParamRadius (radius_);
    pe.setSegmentationParamOptimizeCoefficients (optimize_coefficients_);
    pe.setSegmentationParamNormalDistanceWeight (normal_distance_weight_);
    pe.setSegmentationParamMaxIterations (max_iterations_);
    pe.setSegmentationParamDistanceThreshold (distance_threshold_);
  }

  /**
   * @brief callback for dynamic reconfigure
   *
   * everytime the dynamic reconfiguration changes this function will be called
   *
   * @param inst instance of PlaneExtractionNodelet which parameters should be changed
   * @param config data of configuration
   * @param level bit descriptor which notifies which parameter changed
   *
   * @return nothing
   */
  /*static void callback(PlaneExtractionNodelet *inst, cob_3d_mapping_features::plane_extraction_nodeletConfig &config, uint32_t level)
  {
    if(!inst)
      return;

    //boost::mutex::scoped_lock l1(inst->m_mutex_actionCallback);
    //boost::mutex::scoped_lock l2(inst->m_mutex_pointCloudSubCallback);
    boost::mutex::scoped_lock lock(inst->mutex_);

    inst->file_path_ = config.file_path;
    inst->save_to_file_ = config.save_to_file;
    inst->plane_constraint_ = config.plane_constraint;
    inst->mode_action_ = config.mode_action;
    inst->target_frame_ = config.target_frame;

    inst->pe.setFilePath(inst->file_path_);
    inst->pe.setSaveToFile(inst->save_to_file_);
    inst->pe.setPlaneConstraint((PlaneConstraint)inst->plane_constraint_);
  }*/


  /**
   * @brief initializes parameters
   *
   * initializes parameters
   *
   * @return nothing
   */
  void onInit()
  {
    PCLNodelet::onInit();
    n_ = getNodeHandle();

    config_server_.setCallback(boost::bind(&PlaneExtractionNodelet::dynReconfCallback, this, _1, _2));
    point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &PlaneExtractionNodelet::pointCloudSubCallback, this);
    viz_marker_pub_ = n_.advertise<visualization_msgs::Marker>("plane_marker",10);
    shape_array_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray>("shape_array",1);

    as_= new actionlib::SimpleActionServer<cob_3d_mapping_msgs::PlaneExtractionAction>(n_, "plane_extraction", boost::bind(&PlaneExtractionNodelet::actionCallback, this, _1), false);
    as_->start();

    get_plane_ = n_.advertiseService("get_plane", &PlaneExtractionNodelet::srvCallback, this);

    n_.param("plane_extraction/file_path" ,file_path_ ,std::string("/home/goa-hh/"));
    n_.param("plane_extraction/save_to_file" ,save_to_file_ ,false);
    n_.param("plane_extraction/plane_constraint", plane_constraint_ ,0);
    n_.param("plane_extraction/mode_action", mode_action_ ,false);
    n_.param("plane_extraction/target_frame" ,target_frame_ ,std::string("/map"));
    n_.param("plane_extraction/passthrough_min_z" ,passthrough_min_z_,-0.1);
    n_.param("plane_extraction/passthrough_max_z" ,passthrough_max_z_,2.0);
    n_.param ("plane_extraction/vox_leaf_size", vox_leaf_size_, 0.03);
    n_.param ("plane_extraction/cluster_tolerance", cluster_tolerance_, 0.05);
    n_.param ("plane_extraction/min_cluster_size", min_cluster_size_, 999);
    n_.param ("plane_extraction/max_cluster_size", max_cluster_size_, 60000);
    n_.param ("plane_extraction/radius", radius_, 0.02);
    n_.param ("plane_extraction/optimize_coefficients", optimize_coefficients_, true);
    n_.param ("plane_extraction/normal_distance_weight", normal_distance_weight_, 0.04);
    n_.param ("plane_extraction/max_iterations", max_iterations_, 99);
    n_.param ("plane_extraction/distance_threshold", distance_threshold_, 0.02);

    pe.setFilePath(file_path_);
    pe.setSaveToFile(save_to_file_);
    pe.setPlaneConstraint((PlaneConstraint)plane_constraint_);

    pe.setClusteringParamClusterTolerance (cluster_tolerance_);
    pe.setClusteringParamMinClusterSize (min_cluster_size_);
    pe.setClusteringParamMaxClusterSize (max_cluster_size_);
    pe.setNormalEstimationParamRadius (radius_);
    pe.setSegmentationParamOptimizeCoefficients (optimize_coefficients_);
    pe.setSegmentationParamNormalDistanceWeight (normal_distance_weight_);
    pe.setSegmentationParamMaxIterations (max_iterations_);
    pe.setSegmentationParamDistanceThreshold (distance_threshold_);
  }


  /**
   * @brief extracts planes from a point cloud
   *
   * extracts planes from a point cloud
   *
   * @param pc_in input point cloud
   * @param v_cloud_hull output point cloud with points lying inside the plane
   * @param v_hull_polygons output polygons which describes the plane
   * @param v_coefficients_plane coefficients of the plane calculated by the segmentation of the inliers
   *
   * @return nothing
   */
  void extractPlane(const pcl::PointCloud<Point>::Ptr& pc_in,
                    std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > >& v_cloud_hull,
                    std::vector<std::vector<pcl::Vertices> >& v_hull_polygons,
                    std::vector<pcl::ModelCoefficients>& v_coefficients_plane)
  {
    //std::cout << "pc frame:" << pc_in->header.frame_id << std::endl;
    //pcl::io::savePCDFileASCII ("/home/goa/tmp/before_trans.pcd", *pc_in);
    //pcl::io::savePCDFileASCII ("/home/goa/tmp/after_trans.pcd", *pc_in);
    //std::cout << "pc frame:" << pc_in->header.frame_id << std::endl;
    // Downsample input
    pcl::VoxelGrid<Point> voxel;
    voxel.setInputCloud(pc_in);
    voxel.setLeafSize (vox_leaf_size_, vox_leaf_size_, vox_leaf_size_);
    voxel.setFilterFieldName("z");
    voxel.setFilterLimits(passthrough_min_z_, passthrough_max_z_);
    pcl::PointCloud<Point>::Ptr pc_vox = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    voxel.filter(*pc_vox);
    //pcl::io::savePCDFileASCII ("/home/goa/tmp/after_voxel.pcd", *pc_vox);
    //ROS_INFO("pc size after voxel: %d", pc_vox->size());
    //TODO: transform to /base_link or /map
    pe.extractPlanes(pc_vox, v_cloud_hull, v_hull_polygons, v_coefficients_plane);


  }

  /**
   * @brief extracts planes from a point cloud
   *
   * point cloud will be transformed and then extracted if mode_action is false
   *
   * @param pc_in input point cloud, should be in a coordinate system with z pointing upwards
   *
   * @return nothing
   */
  void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    //boost::mutex::scoped_lock l2(m_mutex_pointCloudSubCallback);

    //ROS_INFO("Extract plane callback");
    boost::mutex::scoped_try_lock lock(mutex_);
    if(!lock)
    //if(!lock.owns_lock())
    {
      //ROS_INFO(" pointCloudSubCallback not owning lock");
      return;
    }
    pcl::PointCloud<Point> pc_trans;
    //if(pc_in->header.frame_id!="/map")
    {
      //ROS_INFO("transforming pc");
      //pcl_ros::transformPointCloud ("/map", pc_in->header.stamp, *pc_in, "/map", pc_trans, tf_listener_);
      pcl_ros::transformPointCloud (target_frame_, *pc_in, pc_trans, tf_listener_);
    }
    /*else
      ROS_INFO(" pointCloudSubCallback owns lock");*/
    //TODO: mode action or topic
    //extractPlane(pc_in);
    if(mode_action_)
      pcl::copyPointCloud(pc_trans, pc_cur_);
    else
    {
      std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > > v_cloud_hull;
      std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
      std::vector<pcl::ModelCoefficients> v_coefficients_plane;
      extractPlane(pc_trans.makeShared(), v_cloud_hull, v_hull_polygons, v_coefficients_plane);
      publishShapeArray(v_cloud_hull, v_hull_polygons, v_coefficients_plane, pc_trans.header);
      for(unsigned int i = 0; i < v_cloud_hull.size(); i++)
      {
        publishMarker(v_cloud_hull[i], pc_trans.header, 0, 0, 1);
        ctr_++;
        //ROS_INFO("%d planes published so far", ctr_);
      }

      publishShapeArray(v_cloud_hull, v_hull_polygons, v_coefficients_plane, pc_trans.header);
    }

  }

  /**
   * @brief extracts planes from a point cloud and saves coefficients of nearest table
   *
   * extracts planes from a point cloud and saves coefficients of nearest table
   *
   * @param goal unused
   *
   * @return nothing
   */
  void
  actionCallback(const cob_3d_mapping_msgs::PlaneExtractionGoalConstPtr &goal)
  {
    //boost::mutex::scoped_lock l1(m_mutex_actionCallback);

    ROS_INFO("action callback");
    //TODO: use scoped_lock
    boost::mutex::scoped_lock lock(mutex_);
    /*if(!lock)
    //if(!lock.owns_lock())
    {
      ROS_INFO(" actionCallback not owning lock");
      return;
    }
    else
      ROS_INFO(" actionCallback owns lock");*/
    feedback_.currentStep.data = std::string("plane extraction");
    std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > > v_cloud_hull;
    std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
    std::vector<pcl::ModelCoefficients> v_coefficients_plane;
    extractPlane(pc_cur_.makeShared(), v_cloud_hull, v_hull_polygons, v_coefficients_plane);
    if(v_cloud_hull.size() == 0)
    {
      as_->setAborted();
      return;
    }
    pcl::copyPointCloud(pc_cur_, pc_plane_);
    // only save dominant plane
    StampedTransform transform;
    try
    {
      tf_listener_.waitForTransform("/map", "/head_cam3d_link", pc_cur_.header.stamp, ros::Duration(2));
      tf_listener_.lookupTransform("/map", "/head_cam3d_link", pc_cur_.header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[plane_extraction] : %s",ex.what());
    }
    btVector3 bt_rob_pose = transform.getOrigin();
    Eigen::Vector3f rob_pose(bt_rob_pose.x(),bt_rob_pose.y(),bt_rob_pose.z());
    ROS_INFO("Rob pose: (%f,%f,%f)", bt_rob_pose.x(),bt_rob_pose.y(),bt_rob_pose.z());
    unsigned int idx = 0;
    pe.findClosestTable(v_cloud_hull, v_coefficients_plane, rob_pose, idx);
    ROS_INFO("Hull %d size: %d", idx, v_cloud_hull[idx].size());
    pcl::copyPointCloud(v_cloud_hull[idx], hull_);
    plane_coeffs_ = v_coefficients_plane[idx];
    as_->setSucceeded(result_);
  }

  /**
   * @brief publishes nearest table
   *
   * publishes nearest table
   *
   * @param req unused
   * @param res result with coefficients
   *
   * @return nothing
   */
  bool
  srvCallback(cob_3d_mapping_msgs::GetPlane::Request &req, cob_3d_mapping_msgs::GetPlane::Response &res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    sensor_msgs::PointCloud2 pc_out, hull_out;
    pcl::toROSMsg(pc_plane_, pc_out);
    pcl::toROSMsg(hull_, hull_out);
    res.pc = pc_out;
    res.hull = hull_out;
    res.plane_coeffs.resize(4);
    res.plane_coeffs[0].data = plane_coeffs_.values[0];
    res.plane_coeffs[1].data = plane_coeffs_.values[1];
    res.plane_coeffs[2].data = plane_coeffs_.values[2];
    res.plane_coeffs[3].data = plane_coeffs_.values[3];
    ROS_INFO("Hull size: %d", res.hull.width*res.hull.height);
    return true;
  }

  /**
   * @brief creates polygon from parameters and publish it
   *
   * creates polygon from parameters and publish it
   *
   * @param cloud_hull point cloud
   * @param hull_polygons polygons
   * @param coefficients_plane coefficients
   * @param header header of published polygons
   *
   * @return nothing
   */
  void
  publishShapeArray(std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > >& v_cloud_hull,
                    std::vector<std::vector<pcl::Vertices> >& v_hull_polygons,
                    std::vector<pcl::ModelCoefficients>& v_coefficients_plane,
                    std_msgs::Header header)
  {
    cob_3d_mapping_msgs::ShapeArray sa;
    //p.polygons.resize(hull_polygons.size());
    sa.header = header;
    /*p.normal.x = coefficients_plane.values[0];
    p.normal.y = coefficients_plane.values[1];
    p.normal.z = coefficients_plane.values[2];
    p.d.data = coefficients_plane.values[3];*/
    for(unsigned int i=0; i<v_cloud_hull.size(); i++)
    {
      //std::cout << "normal: " << v_coefficients_plane[i].values[0] << ","  << v_coefficients_plane[i].values[1] << "," << v_coefficients_plane[i].values[2] << std::endl;
      //std::cout << "d: " << v_coefficients_plane[i].values[3] << std::endl << std::endl;
      //s.points.resize(v_hull_polygons[i].size());
      for(unsigned int j=0; j<v_hull_polygons[i].size(); j++)
      {
        cob_3d_mapping_msgs::Shape s;
        s.type = cob_3d_mapping_msgs::Shape::PLANE;
        s.params.resize(4);
        for(unsigned int c=0; c<4; c++)
          s.params[c] = v_coefficients_plane[i].values[c];
        if (v_hull_polygons[i][j].vertices.size()==0) continue;
        pcl::PointCloud<pcl::PointXYZ> pc;
        for(unsigned int k=0; k<v_hull_polygons[i][j].vertices.size(); k++)
        {
          int idx = v_hull_polygons[i][j].vertices[k];
          pcl::PointXYZ p;
          p.x = v_cloud_hull[i].points[idx].x;
          p.y = v_cloud_hull[i].points[idx].y;
          p.z = v_cloud_hull[i].points[idx].z;
          pc.points.push_back(p);
        }
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(pc, pc_msg);
        s.points.push_back(pc_msg);
        sa.shapes.push_back(s);
      }
    }
    shape_array_pub_.publish(sa);
  }

  /**
   * @brief publish markers
   *
   * publish markers
   *
   * @param cloud_hull point cloud
   * @param header header of published polygons
   * @param r red
   * @param g green
   * @param b blue
   *
   * @return nothing
   */
  void
  publishMarker(pcl::PointCloud<Point>& cloud_hull,
                std_msgs::Header header,
                float r, float g, float b)
  {
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.lifetime = ros::Duration();
    marker.header = header;
    marker.id = ctr_;


    //create the marker in the table reference frame
    //the caller is responsible for setting the pose of the marker to match

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 1;

    geometry_msgs::Point pt;

    for(unsigned int i = 0; i < cloud_hull.points.size(); ++i)
    {
      pt.x = cloud_hull.points[i].x;
      pt.y = cloud_hull.points[i].y;
      pt.z = cloud_hull.points[i].z;

      marker.points.push_back(pt);
    }
    pt.x = cloud_hull.points[0].x;
    pt.y = cloud_hull.points[0].y;
    pt.z = cloud_hull.points[0].z;

    //marker.points.push_back(pt);

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    viz_marker_pub_.publish(marker);
  }

  ros::NodeHandle n_;


protected:
  ros::Subscriber point_cloud_sub_;
  ros::Publisher viz_marker_pub_;
  ros::Publisher shape_array_pub_;
  dynamic_reconfigure::Server<cob_3d_mapping_features::plane_extraction_nodeletConfig> config_server_;

  ros::ServiceServer get_plane_;

  actionlib::SimpleActionServer<cob_3d_mapping_msgs::PlaneExtractionAction>* as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  cob_3d_mapping_msgs::PlaneExtractionFeedback feedback_;
  cob_3d_mapping_msgs::PlaneExtractionResult result_;
  boost::mutex mutex_;

  PlaneExtraction pe;                   /// class for actual calculation
  pcl::PointCloud<Point> pc_cur_;       /// point cloud
  pcl::PointCloud<Point> pc_plane_;     /// point cloud for plane
  pcl::PointCloud<Point> hull_;         /// hull
  pcl::ModelCoefficients plane_coeffs_; /// coefficients

  TransformListener tf_listener_;
  int ctr_;                             /// counter for published planes, also used as id
  //unsigned int min_cluster_size_;       /// parameter for cluster size
  std::string file_path_;
  bool save_to_file_;
  bool mode_action_;
  int plane_constraint_;                /// constraint parameter for PlaneExtraction (pe)
  std::string target_frame_;

  double vox_leaf_size_;				///  voxel filter leaf size
  double passthrough_min_z_;
  double passthrough_max_z_;

  //boost::mutex m_mutex_pointCloudSubCallback, m_mutex_actionCallback;
  //clustering parameters
  double cluster_tolerance_;int min_cluster_size_;int max_cluster_size_;

  //Normal Estimation parameters
  double radius_;

  //segmentation parameters
  bool optimize_coefficients_;
  double normal_distance_weight_;int max_iterations_;
  double distance_threshold_;

};

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_features, PlaneExtractionNodelet, PlaneExtractionNodelet, nodelet::Nodelet)
