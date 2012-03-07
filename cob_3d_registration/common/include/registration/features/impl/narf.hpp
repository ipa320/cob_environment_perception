/*
 * narf.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: goa-jh
 */

#ifndef NARF_HPP_
#define NARF_HPP_


template<typename Point>
void Feature_NARF<Point>::extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<pcl::Narf36> &narf_descriptors) {
  float angular_resolution = 0.0005f;
  float support_size = 0.3f;
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  bool setUnseenToMaxRange = false;
  bool rotation_invariant = true;



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

  // ------------------------------------------------------
  // -----Extract NARF descriptors for interest points-----
  // ------------------------------------------------------
  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize (keypoint_indices.points.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) { // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices.points[i];
  }
  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
  narf_descriptor.getParameters ().support_size = support_size;
  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
  narf_descriptor.compute (narf_descriptors);

}

#endif /* NARF_HPP_ */
