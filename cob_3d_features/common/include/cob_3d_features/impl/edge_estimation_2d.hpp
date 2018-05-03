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
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 10/2011
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

#ifndef __IMPL_EDGE_ESTIMATION_2D_H__
#define __IMPL_EDGE_ESTIMATION_2D_H__

#include "cob_3d_features/edge_estimation_2d.h"
//#include "highgui.h"

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeEstimation2D<PointInT,PointOutT>::getColorImage(
  cv::Mat &color_image)
{
  color_image.create(input_->height, input_->width, CV_8UC3);
  unsigned char* c_ptr = 0;
  int pc_pt_idx=0;
  for (int row = 0; row < color_image.rows; row++)
  {
    c_ptr = color_image.ptr<unsigned char>(row);
    for (int col = 0; col < color_image.cols; col++, pc_pt_idx++)
    {
      memcpy(&c_ptr[3*col], &input_->points[pc_pt_idx].rgb, 3*sizeof(unsigned char));
    }
  }
}

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeEstimation2D<PointInT,PointOutT>::getRangeImage(
  cv::Mat &range_image, const float &th_min, const float &th_max)
{
  range_image.create(input_->height, input_->width, CV_32FC1);
  float* r_ptr = 0;
  int pc_pt_idx=0;
  for (int row = 0; row < range_image.rows; row++)
  {
    r_ptr = range_image.ptr<float>(row);
    for (int col = 0; col < range_image.cols; col++, pc_pt_idx++)
    {
      memcpy(&r_ptr[col], &input_->points[pc_pt_idx].z, sizeof(float));
    }
  }
  if(th_max != 0.0)
    cv::threshold(range_image, range_image, th_max, th_max, cv::THRESH_TRUNC);
  if(th_min != 0.0)
  {
    range_image = range_image - th_min;
    cv::threshold(range_image, range_image, 0, 0, cv::THRESH_TOZERO);
  }
  cv::normalize(range_image, range_image, 0, 1, cv::NORM_MINMAX);
}

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeEstimation2D<PointInT,PointOutT>::extractEdgesSobel(
  cv::Mat &input_image,
  cv::Mat &sobel_image)
{
  cv::Mat sobel_res;
  if( input_image.type() == CV_8UC3)
  {
    std::vector<cv::Mat> image_channels;
    cv::split(input_image, image_channels);
    std::vector<cv::Mat> tmp(3);
    for (int i = 0; i<3; i++)
    {
      cv::Mat tmp_x, tmp_y;
      cv::Sobel(image_channels[i], tmp_x, CV_32FC1, 1, 0, 3);
      tmp_x = abs(tmp_x);
      cv::Sobel(image_channels[i], tmp_y, CV_32FC1, 0, 1, 3);
      tmp_y = abs(tmp_y);
      tmp[i] = tmp_x + tmp_y;
    }
    sobel_res = tmp[0] + tmp[1] + tmp[2];
  }
  else if(input_image.type() == CV_8UC1 || input_image.type() == CV_32FC1)
  {
    cv::Mat tmp_x, tmp_y;
    cv::Sobel(input_image, tmp_x, CV_32FC1, 1, 0, 3);
    tmp_x = abs(tmp_x);
    cv::Sobel(input_image, tmp_y, CV_32FC1, 0, 1, 3);
    tmp_y = abs(tmp_y);
    sobel_res = tmp_x + tmp_y;
  }
  /*cv::Mat test2(cv::Size(640,480),CV_8UC1);
  cv::convertScaleAbs(sobel_res, test2);
  cv::imshow("test", test2);
  cv::waitKey();*/

  cv::normalize(sobel_res, sobel_image, 0, 1, cv::NORM_MINMAX);
}

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeEstimation2D<PointInT,PointOutT>::extractEdgesLaPlace(
  cv::Mat &input_image,
  cv::Mat &laplace_image)
{
  cv::Mat laplace_res;
  if( input_image.type() == CV_8UC3)
  {
    std::vector<cv::Mat> image_channels;
    cv::split(input_image, image_channels);
    std::vector<cv::Mat> laplace_tmp(3);
    for (int i=0; i<3; i++)
    {
      cv::Laplacian(image_channels[i], laplace_tmp[i], CV_32FC1, 1);
      laplace_tmp[i] = abs(laplace_tmp[i]);
    }
    laplace_res = laplace_tmp[0] + laplace_tmp[1] + laplace_tmp[2];
  }
  else if( input_image.type() == CV_8UC1 || input_image.type() == CV_32FC1)
  {
    cv::Laplacian(input_image, laplace_res, CV_32FC1, 1);
    laplace_res = abs(laplace_res);
  }
  cv::normalize(laplace_res, laplace_res, 0, 1, cv::NORM_MINMAX);
  /*cv::threshold(laplace_res, laplace_res, 0.1, 1, cv::THRESH_TOZERO);
  laplace_res *= 2;
  cv::threshold(laplace_res, laplace_res, 1, 1, cv::THRESH_TRUNC);*/
  laplace_image = laplace_res;
}

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeEstimation2D<PointInT,PointOutT>::computeEdges(
  PointCloudOut &output)
{
  cv::Mat color_image(input_->height, input_->width, CV_8UC3);
  getColorImage(color_image);
  cv::Mat img_sobel, img_laplace, img_combined;
  extractEdgesSobel(color_image, img_sobel);
  //cv::Mat test2(cv::Size(640,480),CV_8UC1);
  //cv::convertScaleAbs(img_sobel, test2);
  //cv::imshow("test", img_sobel);
  //cv::waitKey();
  extractEdgesLaPlace(color_image, img_laplace);

  img_combined = img_sobel + img_laplace;
  cv::normalize(img_combined, img_combined, 0, 1, cv::NORM_MINMAX);
  //cv::threshold(img_combined, img_combined, 1, 1, cv::THRESH_TRUNC);

  output.height = input_->height;
  output.width = input_->width;
  output.points.resize(output.height * output.width);

  for (unsigned int i=0; i < output.height; i++)
  {
    for (unsigned int j=0; j < output.width; j++)
    {
      output.points[i*output.width+j].strength = img_combined.at<float>(i,j);
    }
  }
}

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeEstimation2D<PointInT,PointOutT>::computeEdges(
  cv::Mat &sobel_out, cv::Mat &laplace_out, cv::Mat &combined_out)
{
  cv::Mat color_image;
  getColorImage(color_image);
  //std::vector<cv::Mat> img_channels(3);
  //cv::split(color_image, img_channels);
  cv::Mat img_sobel, img_laplace, img_combined;
  extractEdgesSobel(color_image, img_sobel);
  extractEdgesLaPlace(color_image, img_laplace);

  img_combined = img_sobel + img_laplace;
  cv::normalize(img_combined, img_combined, 0, 1, cv::NORM_MINMAX);
  //cv::threshold(img_combined, img_combined, 1, 1, cv::THRESH_TRUNC);

  sobel_out = img_sobel;
  laplace_out = img_laplace;
  combined_out = img_combined;
}

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeEstimation2D<PointInT,PointOutT>::computeEdgesFromRange(
  PointCloudOut &output)
{
  cv::Mat range_image, img_sobel, img_laplace, img_combined;
  getRangeImage(range_image);
  extractEdgesSobel(range_image, img_sobel);
  extractEdgesLaPlace(range_image, img_laplace);

  img_combined = img_sobel + img_laplace;
  cv::threshold(img_combined, img_combined, 1, 1, cv::THRESH_TRUNC);

  output.height = input_->height;
  output.width = input_->width;
  output.points.resize(output.height * output.width);

  for (unsigned int i=0; i < output.height; i++)
  {
    for (unsigned int j=0; j < output.width; j++)
    {
      output.points[i*output.width+j].strength = img_combined.at<float>(i,j);
    }
  }
}

template <typename PointInT, typename PointOutT> void
cob_3d_features::EdgeEstimation2D<PointInT,PointOutT>::computeEdgesFromRange(
  cv::Mat &sobel_out, cv::Mat &laplace_out, cv::Mat &combined_out)
{
  cv::Mat range_image, img_sobel, img_laplace, img_combined;
  getRangeImage(range_image);
  extractEdgesSobel(range_image, img_sobel);
  extractEdgesLaPlace(range_image, img_laplace);

  img_combined = img_sobel + img_laplace;
  cv::threshold(img_combined, img_combined, 1, 1, cv::THRESH_TRUNC);

  sobel_out = img_sobel;
  laplace_out = img_laplace;
  combined_out = img_combined;
}

#define PCL_INSTANTIATE_EdgeEstimation2D(T,OutT) template class PCL_EXPORTS cob_3d_features::EdgeEstimation2D<T,OutT>;

#endif
