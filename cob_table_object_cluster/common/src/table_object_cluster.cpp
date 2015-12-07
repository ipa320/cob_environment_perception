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

#include <cob_3d_mapping_common/minimal_rectangle_2d.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/search/organized.h>

  //aditional includes
#include <ros/console.h>

struct null_deleter
{
    void operator()(void const *) const
    {
    }
};


template<typename Point> void
TableObjectCluster<Point>::extractTableRoi(PointCloudPtr& hull,
                                    pcl::PointIndices& pc_roi)
{
  #if PCL_MINOR_VERSION >= 6
  pcl::ConvexHull<Point> chull;
  chull.setDimension(2);
  chull.setInputCloud(hull);
  PointCloudPtr conv_hull(new pcl::PointCloud<Point>);
  chull.reconstruct(*conv_hull);
  #endif

  pcl::ExtractPolygonalPrismData<Point> prism;
  // Consider only objects in a given layer above the table
  //TODO: check if valid values
  //TODO: does not work for planes other than horizontal, PrismExtraction has to be modified
  //ROS_INFO("height limits: %f, %f ", height_min_, height_max_);
  prism.setHeightLimits(height_min_, height_max_);
  // ---[ Get the objects on top of the table
  //pcl::PointIndices roi_indices;
  prism.setInputCloud(input_);

  #if PCL_MINOR_VERSION >= 6
  prism.setInputPlanarHull(conv_hull);
  #else
  prism.setInputPlanarHull(hull);
  #endif

  prism.segment(pc_roi);
  //ROS_INFO("Number of ROI inliers: %d", roi_indices.indices.size());

  /*pcl::ExtractIndices<Point> extract_roi;
  extract_roi.setInputCloud (pc_in);
  extract_roi.setIndices (boost::make_shared<const pcl::PointIndices> (roi_indices));
  extract_roi.filter (pc_roi);*/
}

template<typename Point> void
TableObjectCluster<Point>::extractTableRoi2(const PointCloudConstPtr& pc_in,
                                    PointCloudPtr& hull,
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

template<typename Point> void
TableObjectCluster<Point>::removeKnownObjects(const PointCloudConstPtr& pc_roi,
                   std::vector<BoundingBox>& bounding_boxes,
                   const PointCloudPtr& pc_roi_red)
{
  //pcl::copyPointCloud(*pc_roi,pc_roi_red);
	boost::shared_ptr<pcl::PointIndices> all_indices(new pcl::PointIndices);
  pcl::CropBox<Point> cb;
  for(unsigned int i=0; i<bounding_boxes.size(); i++)
  {
    std::vector<int> indices;
    cb.setMin(bounding_boxes[i].min_pt);
    cb.setMax(bounding_boxes[i].max_pt);
    cb.setTransform(bounding_boxes[i].pose);
    cb.filter(indices);
    all_indices->indices.insert(all_indices->indices.end(), indices.begin(), indices.end());
  }
  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(pc_roi);
  extract.setIndices(all_indices);
  extract.setNegative(true);
  extract.filter(*pc_roi_red);
}

template<typename Point> void
TableObjectCluster<Point>::calculateBoundingBox(
  const PointCloudPtr& cloud,
  const pcl::PointIndices& indices,
  const Eigen::Vector3f& plane_normal,
  const Eigen::Vector3f& plane_point,
  Eigen::Vector3f& position,
  Eigen::Quaternion<float>& orientation,
  Eigen::Vector3f& size)
{
  // transform to table coordinate frame and project points on X-Y, save max height
  Eigen::Affine3f tf;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(plane_normal.unitOrthogonal(), plane_normal, plane_point, tf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc2d(new pcl::PointCloud<pcl::PointXYZ>);
  float height = 0.0;
  for(std::vector<int>::const_iterator it=indices.indices.begin(); it != indices.indices.end(); ++it)
  {
    Eigen::Vector3f tmp = tf * (*cloud)[*it].getVector3fMap();
    height = std::max<float>(height, fabs(tmp(2)));
    pc2d->push_back(pcl::PointXYZ(tmp(0),tmp(1),0.0));
  }

  // create convex hull of projected points
  #if PCL_MINOR_VERSION >= 6
  pcl::PointCloud<pcl::PointXYZ>::Ptr conv_hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setDimension(2);
  chull.setInputCloud(pc2d);
  chull.reconstruct(*conv_hull);
  #endif

  /*for(int i=0; i<conv_hull->size(); ++i)
    std::cout << (*conv_hull)[i].x << "," << (*conv_hull)[i].y << std::endl;*/

  // find the minimal bounding rectangle in 2D and table coordinates
  Eigen::Vector2f p1, p2, p3;
  cob_3d_mapping::MinimalRectangle2D mr2d;
  mr2d.setConvexHull(conv_hull->points);
  mr2d.rotatingCalipers(p2, p1, p3);
  /*std::cout << "BB: \n" << p1[0] << "," << p1[1] <<"\n"
            << p2[0] << "," << p2[1] <<"\n"
            << p3[0] << "," << p3[1] <<"\n ---" << std::endl;*/

  // compute center of rectangle
  position[0] = 0.5f*(p1[0] + p3[0]);
  position[1] = 0.5f*(p1[1] + p3[1]);
  position[2] = 0.0f;
  // transform back
  Eigen::Affine3f inv_tf = tf.inverse();
  position = inv_tf * position;
  // set size of bounding box
  float norm_1 = (p3-p2).norm();
  float norm_2 = (p1-p2).norm();
  // BoundingBox coordinates: X:= main direction, Z:= table normal
  Eigen::Vector3f direction; // y direction
  if (norm_1 < norm_2)
  {
    direction = Eigen::Vector3f(p3[0]-p2[0], p3[1]-p2[1], 0) / (norm_1);
    size[0] = norm_2 * 0.5f;
    size[1] = norm_1 * 0.5f;
  }
  else
  {
    direction = Eigen::Vector3f(p1[0]-p2[0], p1[1]-p2[1], 0) / (norm_2);
    size[0] = norm_1 * 0.5f;
    size[1] = norm_2 * 0.5f;
  }
  size[2] = -height;


  direction = inv_tf.rotation() * direction;
  orientation = pcl::getTransformationFromTwoUnitVectors(direction, plane_normal).rotation(); // (y, z-direction)

  return;
  Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - plane_normal * plane_normal.transpose();
  Eigen::Vector3f xn = M * Eigen::Vector3f::UnitX(); // X-axis project on normal
  Eigen::Vector3f xxn = M * direction;
  float cos_phi = acos(xn.normalized().dot(xxn.normalized())); // angle between xn and main direction
  cos_phi = cos(0.5f * cos_phi);
  float sin_phi = sqrt(1.0f-cos_phi*cos_phi);
  //orientation.w() = cos_phi;
  //orientation.x() = sin_phi * plane_normal(0);
  //orientation.y() = sin_phi * plane_normal(1);
  //orientation.z() = sin_phi * plane_normal(2);
}

template<typename Point> void
TableObjectCluster<Point>::extractClusters(
  const pcl::PointIndices::Ptr& pc_roi,
  std::vector<PointCloudPtr>& object_clusters,
  std::vector<pcl::PointIndices>& object_cluster_indices)
{
//ROS_INFO("input: %d,%d", input_->width, input_->height);
  #ifdef PCL_VERSION_COMPARE //fuerte
    //typename pcl::search::OrganizedNeighbor<Point>::Ptr clusters_tree( new pcl::search::OrganizedNeighbor<Point>());
    typename pcl::search::KdTree<Point>::Ptr clusters_tree (new pcl::search::KdTree<Point>());
  #else //electric
    typename pcl::KdTreeFLANN<Point>::Ptr clusters_tree (new pcl::KdTreeFLANN<Point> ());
  #endif

  pcl::EuclideanClusterExtraction<Point> cluster_obj;
  cluster_obj.setClusterTolerance (cluster_tolerance_);
  cluster_obj.setMinClusterSize (min_cluster_size_);
  cluster_obj.setInputCloud (input_);
  cluster_obj.setIndices(pc_roi);
  cluster_obj.setSearchMethod (clusters_tree);
  cluster_obj.extract (object_cluster_indices);

  pcl::ExtractIndices<Point> ei;
  ei.setInputCloud(input_);
  for(unsigned int i = 0; i < object_cluster_indices.size(); ++i)
  {
    PointCloudPtr cluster_ptr(new pcl::PointCloud<Point>);
    boost::shared_ptr<pcl::PointIndices> ind_ptr(&object_cluster_indices[i], null_deleter());
    ei.setIndices(ind_ptr);
    ei.filter(*cluster_ptr);
    object_clusters.push_back(cluster_ptr);
  }
}

template<typename Point> void
TableObjectCluster<Point>::calculateBoundingBoxesOld(pcl::PointIndices::Ptr& pc_roi,
                                                  std::vector<PointCloudPtr >& object_clusters,
                                                  std::vector<pcl::PointCloud<pcl::PointXYZ> >& bounding_boxes)
{
  ROS_INFO("roi: %d", pc_roi->indices.size());
  #ifdef PCL_VERSION_COMPARE //fuerte
    typename pcl::search::KdTree<Point>::Ptr clusters_tree (new pcl::search::KdTree<Point>());
    //typename pcl::search::OrganizedNeighbor<Point>::Ptr clusters_tree( new pcl::search::OrganizedNeighbor<Point>());
  #else //electric
    typename pcl::KdTreeFLANN<Point>::Ptr clusters_tree (new pcl::KdTreeFLANN<Point> ());
  #endif

  pcl::EuclideanClusterExtraction<Point> cluster_obj;

  // Table clustering parameters
  cluster_obj.setClusterTolerance (cluster_tolerance_);
  cluster_obj.setMinClusterSize (min_cluster_size_);
  cluster_obj.setInputCloud (input_);
  cluster_obj.setIndices(pc_roi);
  //cluster_obj.setSearchMethod (clusters_tree);
  std::vector<pcl::PointIndices> object_cluster_indices;
  cluster_obj.extract (object_cluster_indices);
  pcl::ExtractIndices<Point> ei;
  ei.setInputCloud(input_);
  for(unsigned int i = 0; i < object_cluster_indices.size(); ++i)
  {
    boost::shared_ptr<pcl::PointIndices> ind_ptr(&object_cluster_indices[i], null_deleter());
    ei.setIndices(ind_ptr);
    PointCloudPtr cluster_ptr(new pcl::PointCloud<Point>);
    ei.filter(*cluster_ptr);
    object_clusters.push_back(cluster_ptr);
    pcl::PointCloud<pcl::PointXYZ> bb;
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*input_, object_cluster_indices[i], min_pt, max_pt);
    //if(fabs(max_pt(2)-min_pt(2))<0.03) continue;
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

template class TableObjectCluster<pcl::PointXYZ>;

template class TableObjectCluster<pcl::PointXYZRGB>;
