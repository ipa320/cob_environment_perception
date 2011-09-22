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
#include <tf_conversions/tf_kdl.h>
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

#include <reconfigureable_node.h>
#include <cob_env_model/plane_extraction_nodeletConfig.h>


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model_msgs/GetPlane.h>
#include <cob_env_model_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "cob_env_model/features/plane_extraction.h"
#include "cob_env_model_msgs/PlaneExtractionAction.h"


using namespace tf;

//####################
//#### nodelet class ####
class PlaneExtractionNodelet : public pcl_ros::PCLNodelet, protected Reconfigurable_Node<cob_env_model::plane_extraction_nodeletConfig>
{
public:
  typedef pcl::PointXYZ Point;
  // Constructor
  PlaneExtractionNodelet()
  : as_(0),
    mode_action_(false),
    Reconfigurable_Node<cob_env_model::plane_extraction_nodeletConfig>("PlaneExtractionNodelet")
  {
    ctr_ = 0;
    min_cluster_size_ = 300;

    setReconfigureCallback(boost::bind(&callback, this, _1, _2));
  }

  // Destructor
  ~PlaneExtractionNodelet()
  {
    /// void
    if(as_) delete as_;
  }

  // callback for dynamic reconfigure
  static void callback(PlaneExtractionNodelet *inst, cob_env_model::plane_extraction_nodeletConfig &config, uint32_t level)
  {
    if(!inst)
      return;

    boost::mutex::scoped_lock l1(inst->m_mutex_actionCallback);
    boost::mutex::scoped_lock l2(inst->m_mutex_pointCloudSubCallback);

    inst->file_path_ = config.file_path;
    inst->save_to_file_ = config.save_to_file;
    inst->plane_constraint_ = config.plane_constraint;
    inst->mode_action_ = config.mode_action;
    inst->target_frame_ = config.target_frame;

    inst->pe.setFilePath(inst->file_path_);
    inst->pe.setSaveToFile(inst->save_to_file_);
    inst->pe.setPlaneConstraint((PlaneConstraint)inst->plane_constraint_);
  }


  void onInit()
  {
    PCLNodelet::onInit();
    n_ = getNodeHandle();

    point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &PlaneExtractionNodelet::pointCloudSubCallback, this);
    viz_marker_pub_ = n_.advertise<visualization_msgs::Marker>("plane_marker",100);
    chull_pub_ = n_.advertise<pcl::PointCloud<Point> >("chull",1);
    object_cluster_pub_ = n_.advertise<pcl::PointCloud<Point> >("object_cluster",1);
    polygon_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("polygons",1);
    polygon_array_pub_ = n_.advertise<cob_env_model_msgs::PolygonArray>("polygon_array",1);

    as_= new actionlib::SimpleActionServer<cob_env_model_msgs::PlaneExtractionAction>(n_, "plane_extraction", boost::bind(&PlaneExtractionNodelet::actionCallback, this, _1), false);
    as_->start();

    get_plane_ = n_.advertiseService("get_plane", &PlaneExtractionNodelet::srvCallback, this);

    n_.param("plane_extraction/file_path" ,file_path_ ,std::string("/home/goa/tmp/"));
    n_.param("plane_extraction/save_to_file" ,save_to_file_ ,false);
    n_.param("plane_extraction/plane_constraint", plane_constraint_ ,0);
    n_.param("plane_extraction/mode_action", mode_action_ ,false);
    n_.param("plane_extraction/target_frame" ,target_frame_ ,std::string("/map"));
    n_.param("plane_extraction/passthrough_min_z" ,passthrough_min_z_,-0.1);
    n_.param("plane_extraction/passthrough_max_z" ,passthrough_max_z_,2.0);
    pe.setFilePath(file_path_);
    pe.setSaveToFile(save_to_file_);
    pe.setPlaneConstraint((PlaneConstraint)plane_constraint_);
  }

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
    voxel.setLeafSize(0.03,0.03,0.03);
    voxel.setFilterFieldName("z");
    voxel.setFilterLimits(passthrough_min_z_, passthrough_max_z_);
    pcl::PointCloud<Point>::Ptr pc_vox = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    voxel.filter(*pc_vox);
    //pcl::io::savePCDFileASCII ("/home/goa/tmp/after_voxel.pcd", *pc_vox);
    //ROS_INFO("pc size after voxel: %d", pc_vox->size());
    //TODO: transform to /base_link or /map
    pe.extractPlanes(pc_vox, v_cloud_hull, v_hull_polygons, v_coefficients_plane);


  }
  // pc_in should be in a coordinate system with z pointing upwards
  void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    boost::mutex::scoped_lock l2(m_mutex_pointCloudSubCallback);

    ROS_INFO("Extract plane callback");
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
      for(unsigned int i = 0; i < v_cloud_hull.size(); i++)
      {
        publishPolygonArray(v_cloud_hull[i], v_hull_polygons[i], v_coefficients_plane[i], pc_in->header);
        publishPolygons(v_cloud_hull[i], pc_in->header);
        publishMarker(v_cloud_hull[i], pc_in->header, 0, 0, 1);
        ctr_++;
        //ROS_INFO("%d planes published so far", ctr_);
      }
    }

  }

  void
  actionCallback(const cob_env_model_msgs::PlaneExtractionGoalConstPtr &goal)
  {
    boost::mutex::scoped_lock l1(m_mutex_actionCallback);

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

  bool
  srvCallback(cob_env_model_msgs::GetPlane::Request &req, cob_env_model_msgs::GetPlane::Response &res)
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


  void
  publishPolygons(pcl::PointCloud<Point>& cloud_hull,
                  std_msgs::Header header)
  {
    geometry_msgs::PolygonStamped polygon;
    polygon.header = header;
    polygon.polygon.points.resize(cloud_hull.points.size());
    for(unsigned int i = 0; i < cloud_hull.points.size(); i++)
    {
      polygon.polygon.points[i].x = cloud_hull.points[i].x;
      polygon.polygon.points[i].y = cloud_hull.points[i].y;
      polygon.polygon.points[i].z = cloud_hull.points[i].z;
    }
    polygon_pub_.publish(polygon);
  }

  void
  publishPolygonArray(pcl::PointCloud<Point>& cloud_hull,
                      std::vector< pcl::Vertices >& hull_polygons,
                      pcl::ModelCoefficients& coefficients_plane,
                      std_msgs::Header header)
  {
    cob_env_model_msgs::PolygonArray p;
    p.polygons.resize(hull_polygons.size());
    p.header = header;
    p.normal.x = coefficients_plane.values[0];
    p.normal.y = coefficients_plane.values[1];
    p.normal.z = coefficients_plane.values[2];
    p.d.data = coefficients_plane.values[3];
    for(unsigned int i=0; i<hull_polygons.size(); i++)
    {
      p.polygons[i].points.resize(hull_polygons[i].vertices.size());
      for(unsigned int j=0; j<hull_polygons[i].vertices.size(); j++)
      {
        int idx = hull_polygons[i].vertices[j];
        p.polygons[i].points[j].x = cloud_hull.points[idx].x;
        p.polygons[i].points[j].y = cloud_hull.points[idx].y;
        p.polygons[i].points[j].z = cloud_hull.points[idx].z;
      }
      polygon_array_pub_.publish(p);
    }
  }

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
  ros::Publisher chull_pub_;
  ros::Publisher object_cluster_pub_;
  ros::Publisher polygon_array_pub_;
  ros::Publisher polygon_pub_;

  ros::ServiceServer get_plane_;

  actionlib::SimpleActionServer<cob_env_model_msgs::PlaneExtractionAction>* as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  cob_env_model_msgs::PlaneExtractionFeedback feedback_;
  cob_env_model_msgs::PlaneExtractionResult result_;
  boost::mutex mutex_;

  PlaneExtraction pe;
  pcl::PointCloud<Point> pc_cur_;
  pcl::PointCloud<Point> pc_plane_;
  pcl::PointCloud<Point> hull_;
  pcl::ModelCoefficients plane_coeffs_;

  TransformListener tf_listener_;
  int ctr_;
  unsigned int min_cluster_size_;
  std::string file_path_;
  bool save_to_file_;
  bool mode_action_;
  int plane_constraint_;
  std::string target_frame_;
  double passthrough_min_z_;
  double passthrough_max_z_;

  boost::mutex m_mutex_pointCloudSubCallback, m_mutex_actionCallback;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, PlaneExtractionNodelet, PlaneExtractionNodelet, nodelet::Nodelet)

/// Old code

/*void publishMarker(pcl::PointCloud<Point>& cloud_hull, std::string& frame_id)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.lifetime = ros::Duration();
  marker.header.frame_id = frame_id;
  marker.id = ctr_;


  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  geometry_msgs::Point pt1, pt2, pt3;
  pt1.x = cloud_hull.points[0].x;
  pt1.y = cloud_hull.points[0].y;
  pt1.z = cloud_hull.points[0].z;

  for(unsigned int i = 1; i < cloud_hull.points.size()-1; ++i)
  {
    pt2.x = cloud_hull.points[i].x;
    pt2.y = cloud_hull.points[i].y;
    pt2.z = cloud_hull.points[i].z;

    pt3.x = cloud_hull.points[i+1].x;
    pt3.y = cloud_hull.points[i+1].y;
    pt3.z = cloud_hull.points[i+1].z;

    marker.points.push_back(pt1);
    marker.points.push_back(pt2);
    marker.points.push_back(pt3);
  }

  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1.0;

  table_marker_pub_.publish(marker);
}*/
