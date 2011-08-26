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
 * Date of creation: 08/2011
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

#include "cob_env_model/table_object_cluster.h"

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

void
TableObjectCluster::extractTableRoi(pcl::PointCloud<Point>::Ptr& pc_in,
                                    pcl::PointCloud<Point>::Ptr& hull,
                                    pcl::PointCloud<Point>& pc_roi)
{
  pcl::ExtractPolygonalPrismData<Point> prism;
  // Consider only objects in a given layer above the table
  //TODO: check if valid values
  prism.setHeightLimits(-0.5, -0.03);
  // ---[ Get the objects on top of the table
  pcl::PointIndices roi_indices;
  prism.setInputCloud(pc_in);
  prism.setInputPlanarHull(hull);
  prism.segment(roi_indices);

  pcl::ExtractIndices<Point> extract_roi;
  extract_roi.setInputCloud (pc_in);
  extract_roi.setIndices (boost::make_shared<const pcl::PointIndices> (roi_indices));
  extract_roi.filter (pc_roi);
}

void
TableObjectCluster::removeKnownObjects(pcl::PointCloud<Point>::Ptr& pc_roi,
                   std::vector<pcl::PointCloud<pcl::PointXYZ> >& bounding_boxes,
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
                   std::vector<pcl::PointCloud<pcl::PointXYZ> >& bounding_boxes)
{
  pcl::KdTree<Point>::Ptr clusters_tree;
  clusters_tree = boost::make_shared<pcl::KdTreeFLANN<Point> > ();

  pcl::EuclideanClusterExtraction<Point> cluster_obj;

  // Table clustering parameters
  cluster_obj.setClusterTolerance (0.03);
  cluster_obj.setMinClusterSize (30);
  cluster_obj.setSearchMethod (clusters_tree);
  std::vector<pcl::PointIndices> object_clusters;
  cluster_obj.setInputCloud (pc_roi_red);
  cluster_obj.extract (object_clusters);
  for(unsigned int i = 0; i < object_clusters.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZ> bb;
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*pc_roi_red, object_clusters[i], min_pt, max_pt);
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
