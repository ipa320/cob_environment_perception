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

#ifndef __EDGE_ESTIMATION_2D_H__
#define __EDGE_ESTIMATION_2D_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace cob_3d_features
{
  template <typename PointInT, typename PointOutT> 
  class EdgeEstimation2D
  {
  public:
    EdgeEstimation2D () { };
    ~EdgeEstimation2D () { };

    typedef pcl::PointCloud<PointInT> PointCloudIn;
    typedef boost::shared_ptr<PointCloudIn> PointCloudInPtr;
    typedef boost::shared_ptr<const PointCloudIn> PointCloudInConstPtr;
    typedef pcl::PointCloud<PointOutT> PointCloudOut;

    /**
     * @brief Sets the input cloud.
     *
     * @param cloud The cloud to set as input_.
     */
    void
    setInputCloud (const PointCloudInConstPtr &cloud)
    {
      input_ = cloud;
    }

    /**
     * @brief Creates color image from RGB channel of input_.
     *
     * @param color_image The color image returned.
     */
    void
    getColorImage(cv::Mat& color_image);

    /**
     * @brief Creates range image from XYZ channels of input_.
     *
     * @param range_image The range image returned.
     * @param th_min Minimum range threshold (default 0.0).
     * @param th_max Maximum range threshold (default 0.0).
     */
    void
    getRangeImage(cv::Mat& range_image, const float &th_min, const float &th_max);
    /**
     * @brief Extracts Sobel edges channel-wise
     *
     * @param input_image The input color or grey image (CV_8UC3 or CV_8UC1).
     * @param sobel_image Sobel output image (CV_32FC1).
     */
    void
    extractEdgesSobel(cv::Mat &input_image, cv::Mat& sobel_image);
    /**
     * @brief Extracts La Place edges from grey image.
     *
     * @param image The input color or grey image (CV_8UC3 or CV_8UC1).
     * @param laplace_image La Place output image (CV_32FC1).
     */
    void
    extractEdgesLaPlace(cv::Mat &input_image, cv::Mat& laplace_image);
    /**
     * @brief Computes 2D edges from a color image.
     *
     * @param output Edge point cloud.
     */
    void
    computeEdges(PointCloudOut &output);
    /**
     * @brief Computes 2D edges from a color image.
     *
     * @param sobel_out Sobel output image (CV_32FC1).
     * @param laplace_out La Place output image (CV_32FC1).
     * @param combined_out Combined Sobel and La Place output image (CV_32FC1).
     */
    void
    computeEdges(cv::Mat &sobel_out, cv::Mat &laplace_out, cv::Mat &combined_out);
    /**
     * @brief Computes 2D edges from a range image.
     *
     * @param output Edge point cloud.
     */
    void
    computeEdgesFromRange(PointCloudOut &output);
    /**
     * @brief Computes 2D edges from a range image.
     *
     * @param sobel_out Sobel output image (CV_32FC1).
     * @param laplace_out La Place output image (CV_32FC1).
     * @param combined_out Combined Sobel and La Place output image (CV_32FC1).
     */
    void
    computeEdgesFromRange(cv::Mat &sobel_out, cv::Mat &laplace_out, cv::Mat &combined_out);

  protected:
    PointCloudInConstPtr input_;

  };
}

#endif //__EDGE_ESTIMATION_2D_H__
