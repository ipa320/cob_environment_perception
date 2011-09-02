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

// ROS includes
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

// external includes
#include <boost/timer.hpp>

// internal includes
#include "cob_env_model/features/plane_extraction.h"


PlaneExtraction::PlaneExtraction()
{
  ctr_ = 0;
  min_cluster_size_ = 300;
  file_path_ = "/home/goa/tmp/";
  save_to_file_ = false;
  plane_constraint_ = NONE;
}

//input should be point cloud that is amplitude filetered, statistical outlier filtered, voxel filtered and the floor cut, coordinate system should be /map
void
PlaneExtraction::extractPlanes(const pcl::PointCloud<Point>::Ptr& pc_in,
                               std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > >& v_cloud_hull,
                               std::vector<std::vector<pcl::Vertices> >& v_hull_polygons,
                               std::vector<pcl::ModelCoefficients>& v_coefficients_plane)
{
  boost::timer t;
  std::stringstream ss;
  ROS_INFO("Extract planes");
  ROS_INFO("Saving files: %d", save_to_file_);
  if(save_to_file_)
  {
    ss.str("");
    ss.clear();
    ss << file_path_ << "/pc_" << ctr_ << ".pcd";
    pcl::io::savePCDFileASCII (ss.str(), *pc_in);
  }
  //ROS_INFO("pc_in size: %d" , pc_in->size());
  // Extract Eucledian clusters
  pcl::KdTree<Point>::Ptr clusters_tree;
  clusters_tree = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  pcl::EuclideanClusterExtraction<Point> cluster;
  // Table clustering parameters
  // TODO: parameter
  cluster.setClusterTolerance (0.06);
  cluster.setMinClusterSize (min_cluster_size_);
  cluster.setSearchMethod (clusters_tree);
  std::vector<pcl::PointIndices> clusters;
  cluster.setInputCloud (pc_in);
  cluster.extract (clusters);
  ROS_INFO ("Number of clusters found: %d", (int)clusters.size ());

  // Go through all clusters and search for planes
  pcl::ExtractIndices<Point> extract;
  for(unsigned int i = 0; i < clusters.size(); ++i)
  {
    ROS_INFO("Processing cluster no. %u", i);
    // Extract cluster points
    pcl::PointCloud<Point> cluster;
    extract.setInputCloud (pc_in);
    extract.setIndices (boost::make_shared<const pcl::PointIndices> (clusters[i]));
    extract.setNegative (false);
    extract.filter (cluster);
    pcl::PointCloud<Point>::Ptr cluster_ptr = cluster.makeShared();
    if(save_to_file_)
    {
      ss.str("");
      ss.clear();
      ss << file_path_ << "/cluster_" << ctr_ << ".pcd";
      pcl::io::savePCDFileASCII (ss.str(), cluster);
    }

    // Estimate point normals
    pcl::NormalEstimation<Point,pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cluster_ptr);
    pcl::KdTreeFLANN<Point>::Ptr tree (new pcl::KdTreeFLANN<Point> ());
    normalEstimator.setSearchMethod(tree);
    //TODO: parameter
    normalEstimator.setRadiusSearch(0.1);
    //normalEstimator.setNumberOfThreads(4);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal> ());
    normalEstimator.compute(*cloud_normals);

    // Find plane
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
    pcl::PointIndices::Ptr consider_in_cluster(new pcl::PointIndices ());
    for(unsigned int idx_ctr=0; idx_ctr < cluster_ptr->size(); idx_ctr++)
      consider_in_cluster->indices.push_back(idx_ctr);
    int ctr = 0;
    // iterate over cluster to find all planes until cluster is too small
    while(consider_in_cluster->indices.size()>min_cluster_size_ /*&& ctr<6*/)
    {
      //ROS_INFO("Cluster size: %d", (int)consider_in_cluster->indices.size());
      ctr++;

      // Do SAC plane segmentation
      pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;
      // Create the segmentation object for the planar model and set all the parameters
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      //seg.setAxis(Eigen::Vector3f(0,0,1));
      //TODO: parameter
      seg.setNormalDistanceWeight (0.05);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (100);
      seg.setDistanceThreshold (0.03);
      seg.setInputCloud (cluster_ptr);
      seg.setIndices(consider_in_cluster);
      seg.setInputNormals (cloud_normals);
      // Obtain the plane inliers and coefficients
      pcl::ModelCoefficients coefficients_plane;
      seg.segment (*inliers_plane, coefficients_plane);

      // Evaluate plane
      float g=1, b=0;
      bool invalidPlane = false;
      if (coefficients_plane.values.size () <=3)
      {
        //ROS_INFO("Failed to detect plane in scan, skipping cluster");
        break;
      }
      if ( inliers_plane->indices.size() < (unsigned int)150)
      {
        //std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;
        //ROS_INFO("Plane detection has %d inliers, below min threshold of %d, skipping cluster", (int)inliers_plane->indices.size(), 150);
        break;
      }
      if(plane_constraint_ == HORIZONTAL)
      {
        if(fabs(coefficients_plane.values[0]) > 0.12 || fabs(coefficients_plane.values[1]) > 0.12 || fabs(coefficients_plane.values[2]) < 0.9)
        {
          std::cout << "Plane is not horizontal: " << coefficients_plane << std::endl;
          invalidPlane = true;
        }
        else
          std::cout << "Plane is horizontal: " << coefficients_plane << std::endl;
      }
      else if(plane_constraint_ == VERTICAL)
      {
        if(fabs(coefficients_plane.values[2]) > 0.1)
          invalidPlane = true;
      }
      else
      {
        if(fabs(coefficients_plane.values[0]) < 0.1 && fabs(coefficients_plane.values[1]) < 0.1 && fabs(coefficients_plane.values[2]) > 0.9)
        {
          //ROS_INFO("Detected plane perpendicular to z axis");
          //std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;
          g=1;
        }
        else if(fabs(coefficients_plane.values[2]) < 0.15) //18 degrees
        {
          //ROS_INFO("Detected plane parallel to z axis");
          //std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;
          b=1;
        }
        /*else
          {
            ROS_INFO("Detected plane not parallel to z axis or perpendicular to z axis");
            //std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;
            break;
          }*/
      }

      if(!invalidPlane)
      {
        // Extract plane points, only needed for storing bag file
        ROS_INFO("Plane has %d inliers", (int)inliers_plane->indices.size());
        pcl::PointCloud<Point> dominant_plane;
        pcl::ExtractIndices<Point> extractIndices;
        extractIndices.setInputCloud(cluster_ptr);
        extractIndices.setIndices(inliers_plane);
        extractIndices.filter(dominant_plane);
        if(save_to_file_)
        {
          ss.str("");
          ss.clear();
          ss << file_path_ << "/plane_" << ctr_ << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), dominant_plane);
          std::cout << ss << std::endl;
        }

        // Project the model inliers
        pcl::PointCloud<Point>::Ptr cloud_projected (new pcl::PointCloud<Point>);
        pcl::ProjectInliers<Point> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cluster_ptr);
        proj.setIndices(inliers_plane);
        proj.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients>(coefficients_plane));
        proj.filter (*cloud_projected);

        // Create a Convex Hull representation of the projected inliers
        pcl::PointCloud<Point> cloud_hull;
        std::vector< pcl::Vertices > hull_polygons;
        pcl::ConcaveHull<Point> chull;
        chull.setInputCloud (cloud_projected);
        //TODO: parameter
        chull.setAlpha (0.2);
        chull.reconstruct (cloud_hull, hull_polygons);
        v_cloud_hull.push_back(cloud_hull);
        v_hull_polygons.push_back(hull_polygons);
        v_coefficients_plane.push_back(coefficients_plane);
        ROS_INFO("v_cloud_hull size: %d", v_cloud_hull.size());


        if(save_to_file_)
        {
          saveHulls(cloud_hull, hull_polygons);
          ss.str("");
          ss.clear();
          ss << file_path_ << "/plane_pr_" << ctr_ << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), *cloud_projected);
        }
      }

      // Remove plane inliers from indices list
      for(unsigned int idx_ctr1=0; idx_ctr1 < consider_in_cluster->indices.size(); idx_ctr1++)
      {
        for(unsigned int idx_ctr2=0; idx_ctr2 < inliers_plane->indices.size(); idx_ctr2++)
        {
          if(consider_in_cluster->indices[idx_ctr1] == inliers_plane->indices[idx_ctr2])
            consider_in_cluster->indices.erase(consider_in_cluster->indices.begin()+idx_ctr1);
        }
      }
      ctr_++;
    }
    if(consider_in_cluster->indices.size()>0)
    {
      if(save_to_file_)
      {
        ss.str("");
        ss.clear();
        ss << file_path_ << "/rem_pts_" << ctr_ << ".pcd";
        if(consider_in_cluster->indices.size() == cluster_ptr->size())
          pcl::io::savePCDFileASCII (ss.str(), *cluster_ptr);
        else
        {
          pcl::PointCloud<Point> remaining_pts;
          extract.setInputCloud (cluster_ptr);
          extract.setIndices (consider_in_cluster);
          extract.filter (remaining_pts);
          pcl::io::savePCDFileASCII (ss.str(), remaining_pts);
        }
      }
    }
  }
  ROS_INFO("Plane extraction took %f", t.elapsed());
  ROS_INFO("v_cloud_hull size: %d", v_cloud_hull.size());
  return;
}

void
PlaneExtraction::saveHulls(pcl::PointCloud<Point>& cloud_hull,
          std::vector< pcl::Vertices >& hull_polygons)
{
  pcl::PointCloud<Point> hull_part;
  for(unsigned int i=0; i<hull_polygons.size(); i++)
  {
    for(unsigned int j=0; j<hull_polygons[i].vertices.size(); j++)
    {
      int idx = hull_polygons[i].vertices[j];
      hull_part.points.push_back(cloud_hull.points[idx]);
    }
    std::stringstream ss;
    ss << file_path_ << "/hull_" << ctr_ << "_" << i << ".pcd";
    pcl::io::savePCDFileASCII (ss.str(), hull_part);
  }
}

void
PlaneExtraction::findClosestTable(std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > >& v_cloud_hull,
                                  std::vector<pcl::ModelCoefficients>& v_coefficients_plane,
                                  Eigen::Vector3f& robot_pose,
                                  unsigned int& idx)
{
  std::vector<unsigned int> table_candidates;
  for(unsigned int i=0; i<v_cloud_hull.size(); i++)
  {
    //if(fabs(v_coefficients_plane[i].values[3])>0.5 && fabs(v_coefficients_plane[i].values[3])<1.2)
      table_candidates.push_back(i);
  }
  if(table_candidates.size()>0)
  {
    for(unsigned int i=0; i<table_candidates.size(); i++)
    {
      double d_min = 1000;
      double d = d_min;
      for(unsigned int j=0; j<v_cloud_hull[i].size(); j++)
      {
        Eigen::Vector3f p = v_cloud_hull[i].points[j].getVector3fMap();
        d += fabs((p-robot_pose).norm());
      }
      d /= v_cloud_hull[i].size();
      ROS_INFO("d: %f", d);
      if(d<d_min)
      {
        d_min = d;
        idx = table_candidates[i];
      }
    }
  }
}



