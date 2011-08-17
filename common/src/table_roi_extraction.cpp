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

#include "cob_env_model/table_roi_extraction.h"

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>

void
TableRoiExtraction::extractTableRoi(pcl::PointCloud<Point>::Ptr& pc_in,
                                    pcl::PointCloud<Point>::Ptr& hull,
                                    pcl::PointCloud<Point>& pc_roi)
{
  pcl::ExtractPolygonalPrismData<Point> prism;
  // Consider only objects in a given layer above the table
  prism.setHeightLimits(-0.5, -0.03);
  // ---[ Get the objects on top of the table
  pcl::PointIndices roi_indices;
  prism.setInputCloud(pc_in);
  prism.setInputPlanarHull(hull);
  prism.segment(roi_indices);

  //pcl::PointCloud<Point> cloud_objects;
  pcl::ExtractIndices<Point> extract_roi;
  extract_roi.setInputCloud (pc_in);
  extract_roi.setIndices (boost::make_shared<const pcl::PointIndices> (roi_indices));
  extract_roi.filter (pc_roi);
  //pcl::PointCloud<Point>::ConstPtr pc_roi_ptr = pc_roi.makeShared();

  /*pcl::KdTree<Point>::Ptr clusters_tree;
  clusters_tree = boost::make_shared<pcl::KdTreeFLANN<Point> > ();

  pcl::EuclideanClusterExtraction<Point> cluster_obj;

  // Table clustering parameters
  cluster_obj.setClusterTolerance (0.06);
  cluster_obj.setMinClusterSize (100);
  cluster_obj.setSearchMethod (clusters_tree);

  // Cluster potential table points
  std::vector<pcl::PointIndices> object_clusters;
  cluster_obj.setInputCloud (cloud_objects.makeShared());
  cluster_obj.extract (object_clusters);

  for(unsigned int i = 0; i < object_clusters.size(); ++i)
  {
    pcl::PointCloud<Point> object;
    extract.setInputCloud (cloud_objects.makeShared());
    extract.setIndices (boost::make_shared<const pcl::PointIndices> (object_clusters[i]));
    extract.setNegative (false);
    extract.filter (object);
    object_cluster_pub_.publish(object);
  }*/
}
