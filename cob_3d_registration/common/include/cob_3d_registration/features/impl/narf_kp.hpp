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
 * ROS package name: registration
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-jh
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Nov 29, 2011
 * ToDo:
 *
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


template<typename Point>
void Keypoints_Narf<Point>::extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<NarfKPoint> &narf_descriptors) {
  float angular_resolution = 0.002f;
  float support_size = 0.3f;
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  bool setUnseenToMaxRange = false;

  // ------------------------------
  // -----Setting viewpoint up-----
  // ------------------------------
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

  scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                      Eigen::Affine3f (point_cloud.sensor_orientation_);

  scene_sensor_pose = scene_sensor_pose.Identity();

  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();

  ROS_INFO("range image %d %d", range_image.height, range_image.width);

  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  narf_keypoint_detector.getParameters ().min_interest_value/=2;
  narf_keypoint_detector.getParameters ().do_non_maximum_suppression=true;

  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute (keypoint_indices);
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

  {
#ifdef PCL_DEPRECATED
      boost::shared_ptr<pcl::search::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
#else
      boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
#endif
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

      pcl::NormalEstimation<Point, pcl::Normal> ne;
      ne.setSearchMethod (tree);
      ne.setInputCloud (point_cloud.makeShared());
      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.05);

      std::vector<int> kp_indices;
      for(int i=0; i<(int)keypoint_indices.size(); i++)
        kp_indices.push_back(keypoint_indices.points[i]);

      ROS_INFO("normal...");
      // Compute the features
      ne.compute (*normals);

      // Create the FPFH estimation class, and pass the input dataset+normals to it
      pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud (point_cloud.makeShared());
      fpfh.setInputNormals (normals);

      fpfh.setSearchMethod (tree);
      fpfh.setRadiusSearch (radius_);

      pcl::PointCloud<pcl::FPFHSignature33> pc_fpfh;

      ROS_INFO("fpfh...");
      // Compute the features
      fpfh.compute (pc_fpfh);

      ROS_INFO("finishing...");


      tree.reset(new pcl::KdTreeFLANN<Point>);
      tree->setInputCloud(point_cloud.makeShared());

      narf_descriptors.clear();
      for(int i=0; i<(int)keypoint_indices.size(); i++) {
        int ind=keypoint_indices.points[i];

        Point pp;
        NarfKPoint p;
        pp.x = p.x = range_image.points[ind].x;
        pp.y = p.y = range_image.points[ind].y;
        pp.z = p.z = range_image.points[ind].z;

        std::vector<int> k_indices; std::vector<float> k_sqr_distances;
        if(!tree->nearestKSearch(pp, 1, k_indices,k_sqr_distances))
          continue;

        int ind2 = k_indices[0];
        p.fpfh = pc_fpfh[ind2];
        narf_descriptors.push_back(p);
      }

  }
}
