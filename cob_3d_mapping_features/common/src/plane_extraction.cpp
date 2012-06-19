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
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

// external includes
//#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"

// internal includes
#include "cob_3d_mapping_features/plane_extraction.h"


PlaneExtraction::PlaneExtraction()
: ctr_(0),
  file_path_("/tmp"),
  save_to_file_(false),
  plane_constraint_(NONE),
  cluster_tolerance_(0.05),
  min_plane_size_(50),
  radius_(0.1),
  //normal_distance_weight_(0.05),
  max_iterations_(100),
  distance_threshold_(0.04)
{
  pcl::KdTreeFLANN<Point>::Ptr tree (new pcl::KdTreeFLANN<Point> ());

  // Init clustering of full cloud
  cluster_.setClusterTolerance (cluster_tolerance_);
  cluster_.setMinClusterSize (min_plane_size_);
  cluster_.setSearchMethod (tree);

  // Init clustering of planes
  //TODO: parameter
  pcl::KdTree<Point>::Ptr clusters_plane_tree(new pcl::KdTreeFLANN<Point> ());
  cluster_plane_.setClusterTolerance (cluster_tolerance_);
  cluster_plane_.setMinClusterSize (min_plane_size_);
  cluster_plane_.setSearchMethod (clusters_plane_tree);

  //normal_estimator_.setSearchMethod(tree);

  seg_.setOptimizeCoefficients (true);
  //seg_.setNormalDistanceWeight (normal_distance_weight_);
  //seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setMaxIterations (max_iterations_);
  seg_.setDistanceThreshold (distance_threshold_);

  proj_.setModelType (pcl::SACMODEL_PLANE);

  chull_.setAlpha (alpha_);
}

bool
PlaneExtraction::isValidPlane (const pcl::ModelCoefficients& coefficients_plane)
{
  //TODO: parameters
  bool validPlane = true;
  switch (plane_constraint_)
  {
    case HORIZONTAL:
    {
      if (fabs (coefficients_plane.values[0]) > 0.12 || fabs (coefficients_plane.values[1]) > 0.12
          || fabs (coefficients_plane.values[2]) < 0.9)
      {
        //std::cout << "Plane is not horizontal: " << coefficients_plane << std::endl;
        validPlane = false;
      }
      //else
      //std::cout << "Plane is horizontal: " << coefficients_plane << std::endl;
      break;
    }
    case VERTICAL:
    {
      if (fabs (coefficients_plane.values[2]) > 0.1)
        validPlane = false;
      break;
    }
    case NONE:
    {
      validPlane = true;
      break;
    }
    default:
      break;
  }
  return validPlane;
}

//input should be point cloud that is amplitude filetered, statistical outlier filtered, voxel filtered, coordinate system should be /map
void
PlaneExtraction::extractPlanes(const pcl::PointCloud<Point>::ConstPtr& pc_in,
                               std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > >& v_cloud_hull,
                               std::vector<std::vector<pcl::Vertices> >& v_hull_polygons,
                               std::vector<pcl::ModelCoefficients>& v_coefficients_plane)
{
  static int ctr=0;
  static double time=0;
  PrecisionStopWatch t;
  t.precisionStart();
  std::stringstream ss;
  ROS_DEBUG("Extract planes");
  ROS_DEBUG("Saving files: %d", save_to_file_);
  if(save_to_file_)
  {
    ss.str("");
    ss.clear();
    ss << file_path_ << "/planes/pc_" << ctr_ << ".pcd";
    pcl::io::savePCDFileASCII (ss.str(), *pc_in);
  }
  //ROS_INFO("pc_in size: %d" , pc_in->size());
  // Extract Eucledian clusters

  std::vector<pcl::PointIndices> clusters;
  cluster_.setInputCloud (pc_in);
  cluster_.extract (clusters);
  //extractClusters (pc_in, clusters);
  ROS_DEBUG ("Number of clusters found: %d", (int)clusters.size ());
  //ROS_INFO("Clustering took %f s", t.precisionStop());
  //t.precisionStart();

  // Estimate point normals
  //normal_estimator_.setInputCloud(pc_in);
  //normalEstimator.setNumberOfThreads(4);
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal> ());
  //normal_estimator_.compute(*cloud_normals);
  //ROS_INFO("Normal estimation took %f s", t.precisionStop());
  //t.precisionStart();

  seg_.setInputCloud (pc_in);
  //seg_.setInputNormals (cloud_normals);

  proj_.setInputCloud (pc_in);

  // Go through all clusters and search for planes

  for(unsigned int i = 0; i < clusters.size(); ++i)
  {
    ROS_DEBUG("Processing cluster no. %u", i);
    // Extract cluster points
    /*pcl::PointCloud<Point> cluster;
    extract_.setInputCloud (pc_in);
    extract_.setIndices (boost::make_shared<const pcl::PointIndices> (clusters[i]));
    extract_.setNegative (false);
    extract_.filter (cluster);
    ROS_INFO("Extraction1 took %f s", t.precisionStop());
    t.precisionStart();*/
    /*pcl::PointCloud<Point>::Ptr cluster_ptr = cluster.makeShared();
    if(save_to_file_)
    {
      ss.str("");
      ss.clear();
      ss << file_path_ << "/planes/cluster_" << ctr_ << ".pcd";
      pcl::io::savePCDFileASCII (ss.str(), cluster);
    }*/

    // Find plane
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
    //pcl::PointIndices::Ptr clusters_ptr(&clusters[i]);
    //pcl::PointIndices::Ptr consider_in_cluster(new pcl::PointIndices ());
    //for(unsigned int idx_ctr=0; idx_ctr < clusters[i].indices.size(); idx_ctr++)
    //  consider_in_cluster->indices.push_back(clusters[i].indices[idx_ctr]);
    int ctr = 0;
    // iterate over cluster to find all planes until cluster is too small
    while(clusters[i].indices.size() > min_plane_size_)
    {
      //ROS_INFO("Cluster size: %d", (int)consider_in_cluster->indices.size());

      seg_.setIndices(boost::make_shared<const pcl::PointIndices> (clusters[i]));
      // Obtain the plane inliers and coefficients
      pcl::ModelCoefficients coefficients_plane;
      seg_.segment (*inliers_plane, coefficients_plane);

      // Evaluate plane
      if (coefficients_plane.values.size () <=3)
      {
        //ROS_INFO("Failed to detect plane in scan, skipping cluster");
        break;
      }
      //TODO: parameter
      if ( inliers_plane->indices.size() < min_plane_size_)
      {
        //std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;
        //ROS_INFO("Plane detection has %d inliers, below min threshold of %d, skipping cluster", (int)inliers_plane->indices.size(), 150);
        break;
      }
      bool validPlane = false;
      validPlane = isValidPlane (coefficients_plane);
      //std::cout << " Plane valid = " << validPlane << std::endl;

      if(validPlane)
      {
        // Extract plane points, only needed for storing bag file
        ROS_DEBUG("Plane has %d inliers", (int)inliers_plane->indices.size());
        /*pcl::PointCloud<Point> dominant_plane;
        extract_.setInputCloud(pc_in);
        extract_.setIndices(inliers_plane);
        extract_.filter(dominant_plane);
        if(save_to_file_)
        {
          ss.str("");
          ss.clear();
          ss << file_path_ << "/planes/plane_" << ctr_ << "_" << ctr << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), dominant_plane);
        }*/

        // Project the model inliers
        pcl::PointCloud<Point>::Ptr cloud_projected (new pcl::PointCloud<Point>);

        proj_.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients>(coefficients_plane));
        proj_.setIndices(inliers_plane);
        proj_.filter (*cloud_projected);
        if(save_to_file_)
        {
          ss.str("");
          ss.clear();
          ss << file_path_ << "/planes/plane_pr_" << ctr_ << "_" << ctr << ".pcd";
          pcl::io::savePCDFileASCII (ss.str(), *cloud_projected);
        }

        std::vector<pcl::PointIndices> plane_clusters;
        cluster_plane_.setInputCloud (cloud_projected);
        cluster_plane_.extract (plane_clusters);

        extract_.setInputCloud(cloud_projected);
        for(unsigned int j=0; j<plane_clusters.size(); j++)
        {
          pcl::PointCloud<Point> plane_cluster;
          extract_.setIndices(boost::make_shared<const pcl::PointIndices> (plane_clusters[j]));
          extract_.filter(plane_cluster);
          pcl::PointCloud<Point>::Ptr plane_cluster_ptr = plane_cluster.makeShared();

          if(plane_cluster_ptr->size() < min_plane_size_) continue;
          //else std::cout << "plane cluster has " << plane_cluster_ptr->size() << " points" << std::endl;

          // Create a Concave Hull representation of the projected inliers
          pcl::PointCloud<Point> cloud_hull;
          std::vector< pcl::Vertices > hull_polygons;
          chull_.setInputCloud (plane_cluster_ptr);
          //TODO: parameter

          chull_.reconstruct (cloud_hull, hull_polygons);
          if(hull_polygons.size() > 1)
          {
            ROS_WARN("Extracted Polygon has more than one contour, separating ...");
            pcl::PointCloud<Point>::Ptr cloud_hull_ptr = cloud_hull.makeShared();
            pcl::ExtractIndices<Point> extract_2;
            extract_2.setInputCloud(cloud_hull_ptr);
            for( unsigned int z=0; z<hull_polygons.size(); z++)
            {
              //ROS_WARN("\tC%d size: %d", z,hull_polygons[z].vertices.size());
              pcl::PointCloud<Point> cloud_hull_2;
              std::vector< pcl::Vertices > hull_polygons_2;
              pcl::PointIndices hull_poly_indices;
              for (unsigned int x=0; x<hull_polygons[z].vertices.size(); x++)
                hull_poly_indices.indices.push_back(hull_polygons[z].vertices[x]);
              //ROS_INFO("Size indices: %d", hull_poly_indices.indices.size());
              extract_2.setIndices(boost::make_shared<const pcl::PointIndices> (hull_poly_indices));
              extract_2.filter(cloud_hull_2);
              //ROS_INFO("Hull 2 size: %d", cloud_hull_2.size());
              pcl::Vertices verts;
              for(unsigned int y=0; y<cloud_hull_2.size(); y++)
                verts.vertices.push_back(y);
              verts.vertices.push_back(0);
              //ROS_INFO("Verts size: %d", verts.vertices.size());
              hull_polygons_2.push_back(verts);
              v_cloud_hull.push_back(cloud_hull_2);
              v_hull_polygons.push_back(hull_polygons_2);
              v_coefficients_plane.push_back(coefficients_plane);
            }
          }
          else
          {
            //ROS_INFO("Hull size: %d", cloud_hull.size());
            //ROS_INFO("Verts size: %d", hull_polygons[0].vertices.size());
            v_cloud_hull.push_back(cloud_hull);
            v_hull_polygons.push_back(hull_polygons);
            v_coefficients_plane.push_back(coefficients_plane);
          }
          ROS_DEBUG("v_cloud_hull size: %d", (unsigned int)v_cloud_hull.size());

          if(save_to_file_)
          {
            saveHulls(cloud_hull, hull_polygons, ctr);
          }
          ctr++;
        }

      }

      // Remove plane inliers from indices vector
      for(unsigned int idx_ctr1=0; idx_ctr1 < clusters[i].indices.size(); idx_ctr1++)
      {
        for(unsigned int idx_ctr2=0; idx_ctr2 < inliers_plane->indices.size(); idx_ctr2++)
        {
          if(clusters[i].indices[idx_ctr1] == inliers_plane->indices[idx_ctr2])
            clusters[i].indices.erase(clusters[i].indices.begin()+idx_ctr1);
        }
      }
      //ctr_++;
    }
    //ROS_INFO("Plane estimation took %f s", t.precisionStop());
    //t.precisionStart();
    /*if(clusters[i].indices.size() >0 )
    {
      if(save_to_file_)
      {
        ss.str("");
        ss.clear();
        ss << file_path_ << "/planes/rem_pts_" << ctr_ << "_" << ctr << ".pcd";
        if(consider_in_cluster->indices.size() == cluster_ptr->size())
          pcl::io::savePCDFileASCII (ss.str(), *cluster_ptr);
        else
        {
          pcl::PointCloud<Point> remaining_pts;
          extract_.setInputCloud (cluster_ptr);
          extract_.setIndices (consider_in_cluster);
          extract_.filter (remaining_pts);
          pcl::io::savePCDFileASCII (ss.str(), remaining_pts);
        }
      }
    }*/
    ctr_++;
  }
  double step_time = t.precisionStop();
  ROS_INFO("Plane extraction took %f", step_time);
  time += step_time;
  ROS_INFO("[plane extraction] Accumulated time at step %d: %f s", ctr, time);
  ctr++;
  return;
}

void
PlaneExtraction::dumpToPCDFileAllPlanes (pcl::PointCloud<Point>::Ptr dominant_plane_ptr)
{

  extracted_planes_.header.frame_id = dominant_plane_ptr->header.frame_id;
  extracted_planes_ += *dominant_plane_ptr;
  std::stringstream ss;
  ss << file_path_ << "/planes/all_planes.pcd";
  pcl::io::savePCDFileASCII (ss.str (), extracted_planes_);
  /*
   //pcl::visualization::CloudViewer viewer(" All Extracted planes");
   pcl::PointCloud<Point>::Ptr all_planes (new pcl::PointCloud<Point>);
   pcl::io::loadPCDFile (ss.str (), *all_planes);
   */
}
void
PlaneExtraction::saveHulls(pcl::PointCloud<Point>& cloud_hull,
                           std::vector< pcl::Vertices >& hull_polygons,
                           int plane_ctr)
{
  for(unsigned int i=0; i<hull_polygons.size(); i++)
  {
    pcl::PointCloud<Point> hull_part;
    for(unsigned int j=0; j<hull_polygons[i].vertices.size(); j++)
    {
      int idx = hull_polygons[i].vertices[j];
      hull_part.points.push_back(cloud_hull.points[idx]);
    }
    std::stringstream ss;
    ss << file_path_ << "/planes/hull_" << ctr_ << "_" << plane_ctr << "_" <<  i << ".pcd";
    pcl::io::savePCDFileASCII (ss.str(), hull_part);
  }
}


//TODO: move to semantics
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
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(v_cloud_hull[i], centroid);
      //for(unsigned int j=0; j<v_cloud_hull[i].size(); j++)
      //{
      //  Eigen::Vector3f p = v_cloud_hull[i].points[j].getVector3fMap();
      //  d += fabs((p-robot_pose).norm());
      //}
      //d /= v_cloud_hull[i].size();
      Eigen::Vector3f centroid3 = centroid.head(3);
      d = fabs((centroid3-robot_pose).norm());
      ROS_INFO("d: %f", d);
      if(d<d_min)
      {
        d_min = d;
        idx = table_candidates[i];
      }
    }
  }
}

#include <pcl/filters/voxel_grid.h>

int main()
{
  PlaneExtraction pe;
  std::string file_path("/home/goa/pcl_daten/kitchen_kinect2/");
  pe.setFilePath(file_path);
  pe.setSaveToFile(true);
  std::stringstream ss;
  ss << file_path << "pointclouds/point_cloud.pcd";
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::io::loadPCDFile (ss.str(), cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = cloud.makeShared();
  std::cout << "Pointcloud of size " << cloud_ptr->size() << " loaded" << std::endl;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(cloud_ptr);
  voxel.setLeafSize(0.03,0.03,0.03);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vox = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  voxel.filter(*cloud_vox);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB> > > v_cloud_hull;
  std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
  std::vector<pcl::ModelCoefficients> v_coefficients_plane;
  pe.extractPlanes(cloud_vox, v_cloud_hull, v_hull_polygons, v_coefficients_plane);
  return 0;
}


