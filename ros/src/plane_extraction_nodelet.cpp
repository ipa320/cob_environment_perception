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


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "cob_env_model/features/plane_extraction.h"


using namespace tf;

//####################
//#### nodelet class ####
class PlaneExtractionNodelet : public pcl_ros::PCLNodelet
{
public:
  typedef pcl::PointXYZRGB Point;
  // Constructor
  PlaneExtractionNodelet()
    : pe(true)
  {
    ctr_ = 0;
    min_cluster_size_ = 300;
    //file_path_ = "/home/goa/pcl_daten/kitchen_kinect2/planes/";
    //save_to_file_ = false;
    pe.setPlaneConstraint(HORIZONTAL);
  }

  // Destructor
  ~PlaneExtractionNodelet()
  {
    /// void
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
    polygon_array_pub_ = n_.advertise<cob_env_model::PolygonArray>("polygon_array",1);
  }


  // pc_in should be in a coordinate system with z pointing upwards
  void pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    ROS_INFO("Extract plane callback");
    //TODO: transform to /base_link or /map
    std::vector<pcl::PointCloud<Point> > v_cloud_hull;
    std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
    std::vector<pcl::ModelCoefficients> v_coefficients_plane;
    pe.extractPlanes(pc_in, v_cloud_hull, v_hull_polygons, v_coefficients_plane);

    for(unsigned int i = 0; i < v_cloud_hull.size(); i++)
    {
      publishPolygonArray(v_cloud_hull[i], v_hull_polygons[i], v_coefficients_plane[i], pc_in->header);
      publishPolygons(v_cloud_hull[i], pc_in->header);
      publishMarker(v_cloud_hull[i], pc_in->header, 0, 0, 1);
      ctr_++;
      //ROS_INFO("%d planes published so far", ctr_);
    }

    return;
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
    cob_env_model::PolygonArray p;
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

  PlaneExtraction pe;

  TransformListener tf_listener_;
  int ctr_;
  unsigned int min_cluster_size_;
  std::string file_path_;
  bool save_to_file_;

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
