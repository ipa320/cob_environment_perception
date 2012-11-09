/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: cob_env_model
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "cob_table_object_cluster/table_object_cluster.h"

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/io/pcd_io.h>

  //aditional includes
#include <ros/console.h>

struct null_deleter
{
    void operator()(void const *) const
    {
    }
};


void
TableObjectCluster::extractTableRoi(pcl::PointCloud<Point>::Ptr& pc_in,
                                    pcl::PointCloud<Point>::Ptr& hull,
                                    pcl::PointCloud<Point>& pc_roi)
{
  pcl::ExtractPolygonalPrismData<Point> prism;
  // Consider only objects in a given layer above the table
  //TODO: check if valid values
  //TODO: does not work for planes other than horizontal, PrismExtraction has to be modified
  //ROS_INFO("height limits: %f, %f ", height_min_, height_max_);
  prism.setHeightLimits(height_min_, height_max_);
  // ---[ Get the objects on top of the table
  pcl::PointIndices roi_indices;
  prism.setInputCloud(pc_in);
  prism.setInputPlanarHull(hull);
  prism.segment(roi_indices);
  //ROS_INFO("Number of ROI inliers: %d", roi_indices.indices.size());

  pcl::ExtractIndices<Point> extract_roi;
  extract_roi.setInputCloud (pc_in);
  extract_roi.setIndices (boost::make_shared<const pcl::PointIndices> (roi_indices));
  extract_roi.filter (pc_roi);
}

void
TableObjectCluster::extractTableRoi2(pcl::PointCloud<Point>::Ptr& pc_in,
                                    pcl::PointCloud<Point>::Ptr& hull,
                                    Eigen::Vector4f& plane_coeffs,
                                    pcl::PointCloud<Point>& pc_roi)
{
  pcl::PointIndices indices;
  for(unsigned int i=0; i<pc_in->size(); i++)
    indices.indices.push_back(i);
  // Project all points
  pcl::PointCloud<Point> projected_points;
  pcl::SampleConsensusModelPlane<Point> sacmodel (pc_in);
  //Eigen::Vector4f e_plane_coeffs(plane_coeffs.values[0],plane_coeffs.values[1],plane_coeffs.values[2],plane_coeffs.values[3]);
  sacmodel.projectPoints (indices.indices, plane_coeffs, projected_points, false);
  pcl::io::savePCDFileASCII ("/home/goa/tmp/proj.pcd", projected_points);
  pcl::io::savePCDFileASCII ("/home/goa/tmp/hull.pcd", *hull);
  pcl::PointIndices inliers;
  std::cout << "Coeffs:" << plane_coeffs << std::endl;
  int ctr=0, ctr2=0;
  for(unsigned int i=0; i<pc_in->size(); i++)
  {
    double distance = pcl::pointToPlaneDistanceSigned (pc_in->points[i], plane_coeffs);
    if (distance < height_min_ || distance > height_max_)
         continue;
    ctr++;
    if (!pcl::isXYPointIn2DXYPolygon (projected_points.points[i], *hull))
      continue;
    ctr2++;
    ROS_INFO("Point is in polygon");
    inliers.indices.push_back(i);
  }
  ROS_INFO("Pts in height: %d", ctr);
  ROS_INFO("Pts in poly: %d", ctr2);

  pcl::ExtractIndices<Point> extract_roi;
  extract_roi.setInputCloud (pc_in);
  extract_roi.setIndices (boost::make_shared<const pcl::PointIndices> (inliers));
  extract_roi.filter (pc_roi);
}

void
TableObjectCluster::removeKnownObjects(pcl::PointCloud<Point>::Ptr& pc_roi,
                   std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > >& bounding_boxes,
                   pcl::PointCloud<Point>& pc_roi_red)
{
  pcl::copyPointCloud(*pc_roi,pc_roi_red);
  for(unsigned int i=0; i<bounding_boxes.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ>& bb = bounding_boxes[i];
    std::vector<int> indices;
    Eigen::Vector4f min_pt(bb.points[0].x, bb.points[0].y, bb.points[0].z, 1);
    Eigen::Vector4f max_pt(bb.points[1].x, bb.points[1].y, bb.points[1].z, 1);
    pcl::PointIndices obj_indices;
    pcl::getPointsInBox(pc_roi_red, min_pt, max_pt, obj_indices.indices);
    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(pc_roi_red.makeShared());
    extract.setIndices(boost::make_shared<const pcl::PointIndices> (obj_indices));
    extract.setNegative(true);
    extract.filter(pc_roi_red);
  }
}

void
TableObjectCluster::calculateBoundingBoxes(pcl::PointCloud<Point>::Ptr& pc_roi_red,
                                           std::vector<pcl::PointCloud<Point>::Ptr >& object_clusters,
                   std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > >& bounding_boxes)
{
  #ifdef PCL_VERSION_COMPARE //fuerte
    pcl::search::KdTree<Point>::Ptr clusters_tree (new pcl::search::KdTree<Point>());
  #else //electric
    pcl::KdTreeFLANN<Point>::Ptr clusters_tree (new pcl::KdTreeFLANN<Point> ());
  #endif

  pcl::EuclideanClusterExtraction<Point> cluster_obj;

  // Table clustering parameters
  cluster_obj.setClusterTolerance (cluster_tolerance_);
  cluster_obj.setMinClusterSize (min_cluster_size_);
  cluster_obj.setSearchMethod (clusters_tree);
  std::vector<pcl::PointIndices> object_cluster_indices;
  cluster_obj.setInputCloud (pc_roi_red);
  cluster_obj.extract (object_cluster_indices);
  pcl::ExtractIndices<Point> ei;
  ei.setInputCloud(pc_roi_red);
  for(unsigned int i = 0; i < object_cluster_indices.size(); ++i)
  {
    boost::shared_ptr<pcl::PointIndices> ind_ptr(&object_cluster_indices[i], null_deleter());
    ei.setIndices(ind_ptr);
    pcl::PointCloud<Point>::Ptr cluster_ptr(new pcl::PointCloud<Point>);
    ei.filter(*cluster_ptr);
    object_clusters.push_back(cluster_ptr);
    pcl::PointCloud<pcl::PointXYZ> bb;
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*pc_roi_red, object_cluster_indices[i], min_pt, max_pt);
    if(fabs(max_pt(2)-min_pt(2))<0.03) continue;
    pcl::PointXYZ p;
    p.x = min_pt(0);
    p.y = min_pt(1);
    p.z = min_pt(2);
    bb.push_back(p);
    p.x = max_pt(0);
    p.y = max_pt(1);
    p.z = max_pt(2);
    bb.push_back(p);
    bounding_boxes.push_back(bb);
  }
}
