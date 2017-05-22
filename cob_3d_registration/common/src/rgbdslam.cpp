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
 *  ROS package name: registration
 *
 * \author
 *  Author: goa-jh
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: Nov 9, 2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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

//Documentation see header file
#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
#include "pcl/common/transform.h"
#include "pcl_ros/transforms.h"
#include <registration/registration_rgbdslam.h>
#include <cv_bridge/CvBridge.h>
#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ctime>
#include "node.h"
#include <sensor_msgs/PointCloud2.h>


Registration_RGBDSLAM::Registration_RGBDSLAM()
{

}

OpenNIListener::OpenNIListener(ros::NodeHandle nh, const char* visual_topic,
                               const char* depth_topic, const char* info_topic,
                               const char* cloud_topic, const char* detector_type,
                               const char* extractor_type)
: graph_mgr(nh),
  visual_sub_ (nh, visual_topic, 3),
  depth_sub_(nh, depth_topic, 3),
  info_sub_(nh, info_topic, 3),
  cloud_sub_(nh, cloud_topic, 3),
  sync_(MySyncPolicy(10),  visual_sub_, depth_sub_, info_sub_, cloud_sub_),
  depth_mono8_img_(cv::Mat()),
  callback_counter_(0),
  pause_(true),
  first_frame_(true)
  //pc_pub(nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 2))
{
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  sync_.registerCallback(boost::bind(&OpenNIListener::cameraCallback, this, _1, _2, _3, _4));
  ROS_INFO_STREAM("OpenNIListener listening to " << visual_topic << ", " << depth_topic \
                   << ", " << info_topic << " and " << cloud_topic << "\n");
  detector_ = this->createDetector(detector_type);
  ROS_FATAL_COND(detector_.empty(), "No valid detector, directly after creation!");
  extractor_ = this->createDescriptorExtractor(extractor_type);
  matcher_ = new cv::BruteForceMatcher<cv::L2<float> >() ;
  pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
            "/rgbdslam/cloud", 5);
  pub_transf_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
            "/rgbdslam/transformed_slowdown_cloud", 5);
  pub_ref_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
            "/rgbdslam/first_frame", 1);
}

void OpenNIListener::cameraCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,
                                     const sensor_msgs::CameraInfoConstPtr& cam_info,
                                     const sensor_msgs::PointCloud2ConstPtr& point_cloud) {
  // ROS_INFO("Received data from kinect");
  if(++callback_counter_ % 3 != 0 || pause_) return;
  //Get images into correct format
        sensor_msgs::CvBridge bridge;
        cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg);
        cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg, "mono8");
  if(visual_img.rows != depth_float_img.rows || visual_img.cols != depth_float_img.cols){
    ROS_ERROR("Depth and Visual image differ in size!");
    return;
  }

  //get pinhole camera model to backproject image locations later
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cam_info); //pinhole camera model
  depthToCV8UC1(depth_float_img, depth_mono8_img_);


  // create new node an add it to the graph
  bool has_been_added = graph_mgr.addNode(Node(&graph_mgr.nh_,visual_img, depth_float_img, cam_model, detector_, extractor_, matcher_,point_cloud, point_cloud->header.seq,0,depth_mono8_img_));
  graph_mgr.drawFeatureFlow(visual_img);

  emit newVisualImage(cvMat2QImage(visual_img));
  ROS_DEBUG("Sending PointClouds");
  //if node position was optimized: publish received pointcloud
  if (has_been_added && graph_mgr.freshlyOptimized_ && (pub_cloud_.getNumSubscribers() > 0)){
      ROS_INFO("Sending Original PointCloud in new Frame");
      sensor_msgs::PointCloud2 msg = *point_cloud;
      msg.header.stamp = graph_mgr.time_of_last_transform_;
      msg.header.frame_id = "/slam_transform";
      pub_cloud_.publish(msg);
  }
  //Very slow, but only done if subscribed to: Transform pointcloud to fixed coordinate system and resend
  if (has_been_added && graph_mgr.freshlyOptimized_ && (pub_transf_cloud_.getNumSubscribers() > 0)){
      ROS_INFO("Sending Transformed PointCloud in fixed Frame");
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::fromROSMsg(*point_cloud, pcl_cloud);
      pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud, graph_mgr.world2cam_ );
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(pcl_cloud, msg);
      msg.header.frame_id = "/openni_camera";
      msg.header.stamp = ros::Time::now();
      pub_transf_cloud_.publish(msg);
  }
  //when first node has been added,  send it out unchanged as reference
  if (first_frame_ && has_been_added && (pub_ref_cloud_.getNumSubscribers() > 0)){
      ROS_INFO("Sending Reference PointCloud");
      pub_ref_cloud_.publish(*point_cloud);
      first_frame_ = false;
  }
  //emit newDepthImage(cvMat2QImage(depth_mono8_img_));//overwrites last cvMat2QImage
}

using namespace cv;
FeatureDetector* OpenNIListener::createDetector( const string& detectorType ) {
    FeatureDetector* fd = 0;
    if( !detectorType.compare( "FAST" ) ) {
        //fd = new FastFeatureDetector( 20/*threshold*/, true/*nonmax_suppression*/ );
        fd = new DynamicAdaptedFeatureDetector (new FastAdjuster(20,true), 300, 600, 10);
    }
    else if( !detectorType.compare( "STAR" ) ) {
        fd = new StarFeatureDetector( 16/*max_size*/, 5/*response_threshold*/, 10/*line_threshold_projected*/,
                                      8/*line_threshold_binarized*/, 5/*suppress_nonmax_size*/ );
    }
    else if( !detectorType.compare( "SIFT" ) ) {
        fd = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                     SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
    }
    else if( !detectorType.compare( "SURF" ) ) {
        fd = new DynamicAdaptedFeatureDetector(new SurfAdjuster(), 300, 600, 5);
        //fd = new SurfFeatureDetector( 300./*hessian_threshold*/, 3 /*octaves*/, 4/*octave_layers*/ );
    }
    else if( !detectorType.compare( "MSER" ) ) {
        fd = new MserFeatureDetector( 5/*delta*/, 60/*min_area*/, 14400/*_max_area*/, 0.25f/*max_variation*/,
                0.2/*min_diversity*/, 200/*max_evolution*/, 1.01/*area_threshold*/, 0.003/*min_margin*/,
                5/*edge_blur_size*/ );
    }
    else if( !detectorType.compare( "GFTT" ) ) {
        fd = new GoodFeaturesToTrackDetector( 200/*maxCorners*/, 0.001/*qualityLevel*/, 1./*minDistance*/,
                                              5/*int _blockSize*/, true/*useHarrisDetector*/, 0.04/*k*/ );
    }
    else {
      ROS_ERROR("No valid detector-type given: %s. Using SURF.", detectorType.c_str());
      fd = createDetector("SURF"); //recursive call with correct parameter
    }
    ROS_ERROR_COND(fd == 0, "No detector could be created");
    return fd;
}

DescriptorExtractor* OpenNIListener::createDescriptorExtractor( const string& descriptorType ) {
    DescriptorExtractor* extractor = 0;
    if( !descriptorType.compare( "SIFT" ) ) {
        extractor = new SiftDescriptorExtractor();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
    }
    else if( !descriptorType.compare( "SURF" ) ) {
        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
    else {
      ROS_ERROR("No valid descriptor-matcher-type given: %s. Using SURF", descriptorType.c_str());
      extractor = createDescriptorExtractor("SURF");
    }
    return extractor;
}

void OpenNIListener::togglePause(){
  pause_ = !pause_;
}
// Old helper functions from earlier times. Could be useful lateron

QImage OpenNIListener::cvMat2QImage(const cv::Mat& image){
  ROS_DEBUG_STREAM("Converting Matrix of type " << openCVCode2String(image.type()) << " to RGBA");
  ROS_DEBUG_STREAM("Target Matrix is of type " << openCVCode2String(rgba_buffer_.type()));
  /*if(image.channels() >= 3){ //Add alpha channel if it is a color image
    cv::Mat alpha( image.rows, image.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
    cv::Mat in[] = { image , alpha };
    // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
    // rgba[2] -> bgr[0], rgba[3] -> alpha[0]
    int from_to[] = { 2,0,  1,1,  0,2,  3,3 };
    mixChannels( in , 2, &rgba_buffer, 1, from_to, 4 );
  }else if(image.channels() == 1){   // Convert visual image to rgba */
  if(image.rows != rgba_buffer_.rows || image.cols != rgba_buffer_.cols){
    ROS_INFO("Created new rgba_buffer");
    rgba_buffer_ = cv::Mat( image.rows, image.cols, CV_8UC4);
    printMatrixInfo(rgba_buffer_);
  }
  cv::Mat alpha( image.rows, image.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  cv::Mat in[] = { image, alpha };
  // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
  // rgba[2] -> bgr[0], rgba[3] -> alpha[0]
  int from_to[] = { 0,0,  0,1,  0,2,  1,3 };
  mixChannels( in , 2, &rgba_buffer_, 1, from_to, 4 );
  printMatrixInfo(rgba_buffer_);
  //cv::cvtColor(image, rgba_buffer_, CV_GRAY2RGBA);}
  //}
  return QImage((unsigned char *)(rgba_buffer_.data),
                rgba_buffer_.cols, rgba_buffer_.rows,
                rgba_buffer_.step, QImage::Format_RGB32 );
}

void transformPointCloud (const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in,
                          sensor_msgs::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    ROS_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 ||
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 ||
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32) {
    ROS_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }
  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");
  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i) {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;

    bool max_range_point = false;
    int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2])) {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr)) { // Invalid point
        pt_out = pt;
      } else { // max range point
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    } else {
      pt_out = transform * pt;
    }

    if (max_range_point) {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
      //std::cout << __PRETTY_FUNCTION__<<": "<<i << "is a max range point.\n";
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));

    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1) {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i) {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  //The following doesn't work due to NaNs
  //double minVal, maxVal;
  //minMaxLoc(float_img, &minVal, &maxVal);
  //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  //mono8_img = cv::Scalar(0);
  //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}

//Little debugging helper functions
std::string openCVCode2String(unsigned int code){
  switch(code){
    case 0 : return std::string("CV_8UC1" );
    case 8 : return std::string("CV_8UC2" );
    case 16: return std::string("CV_8UC3" );
    case 24: return std::string("CV_8UC4" );
    case 2 : return std::string("CV_16UC1");
    case 10: return std::string("CV_16UC2");
    case 18: return std::string("CV_16UC3");
    case 26: return std::string("CV_16UC4");
    case 5 : return std::string("CV_32FC1");
    case 13: return std::string("CV_32FC2");
    case 21: return std::string("CV_32FC3");
    case 29: return std::string("CV_32FC4");
  }
  return std::string("Unknown");
}

void printMatrixInfo(cv::Mat& image){
  ROS_DEBUG_STREAM("Matrix Type:" << openCVCode2String(image.type()) <<  " rows: " <<  image.rows  <<  " cols: " <<  image.cols);
}
