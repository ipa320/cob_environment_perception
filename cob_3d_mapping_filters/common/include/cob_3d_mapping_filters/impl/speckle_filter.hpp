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
 *  ROS package name: cob_3d_mapping_filters
 *
 * \author
 *  Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 05/2011
 *
 * \brief
 * Description:
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
#ifndef SPECKLE_FILTER_HPP_
#define SPECKLE_FILTER_HPP_

//##################
//#### includes ####

// cob_3d_mapping_filters includes
#include "cob_3d_mapping_filters/speckle_filter.h"

//TODO: get rid of opencv
//#include <opencv2/opencv.hpp>
#include "pcl/point_types.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

template<typename PointT>
  void
  cob_3d_mapping_filters::SpeckleFilter<PointT>::applyFilter (PointCloud &pc_out)
  {
    pc_out.points.resize (input_->points.size ());
    pc_out.header = input_->header;

    points_to_remove_->indices.clear ();
    float newVal = std::numeric_limits<float>::quiet_NaN ();
    int width = input_->width, height = input_->height, npixels = width * height;
    size_t bufSize = npixels * (int)(sizeof(Eigen::Vector2i) + sizeof(int) + sizeof(unsigned char));

    unsigned char* buf = new unsigned char[bufSize];
    unsigned char* rem_buf = buf;
    int i, j, dstep = input_->width;
    int* labels = (int*)buf;
    buf += npixels * sizeof(labels[0]);
    Eigen::Vector2i* wbuf = (Eigen::Vector2i*)buf;
    buf += npixels * sizeof(wbuf[0]);
    unsigned char* rtype = (unsigned char*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset (labels, 0, npixels * sizeof(labels[0]));

    for (i = 0; i < height; i++)
    {
      const PointT * const ds = &input_->points[i * input_->width];
      int* ls = labels + width * i;

      for (j = 0; j < width; j++)
      {
        if (!pcl_isnan (ds[j].y)) // != newVal )    // not a bad disparity
        {
          if (ls[j]) // has a label, check for bad label
          {
            if (rtype[ls[j]]) // small region, zero out disparity
            {
              points_to_remove_->indices.push_back (j + i * input_->width);
            }
          }
          // no label, assign and propagate
          else
          {
            Eigen::Vector2i* ws = wbuf; // initialize wavefront
            Eigen::Vector2i p (j, i); // current pixel
            curlabel++; // next label
            int count = 0; // current region size
            ls[j] = curlabel;

            // wavefront propagation
            while (ws >= wbuf) // wavefront not empty
            {
              count++;
              // put neighbors onto wavefront
              const PointT * const dpp = &input_->points[p(0) + p(1) * input_->width];
              const PointT dp = *dpp;
              int* lpp = labels + width * p(1) + p(0);

              if (p(0) < width - 1 && !lpp[+1] && !pcl_isnan (dpp[+1].z)
                  && std::abs (dp.z - dpp[+1].z) <= speckle_range_)
              {
                lpp[+1] = curlabel;
                *ws++ = Eigen::Vector2i (p(0) + 1, p(1));
                //*ws++ = cv::Point_<short> (p.x + 1, p.y);
              }

              if (p(0) > 0 && !lpp[-1] && !pcl_isnan (dpp[-1].z) && std::abs (dp.z - dpp[-1].z) <= speckle_range_)
              {
                lpp[-1] = curlabel;
                *ws++ = Eigen::Vector2i (p(0) - 1, p(1));
              }

              if (p(1) < height - 1 && !lpp[+width] && !pcl_isnan (dpp[+dstep].z)
                  && std::abs (dp.z - dpp[+dstep].z) <= speckle_range_)
              {
                lpp[+width] = curlabel;
                *ws++ = Eigen::Vector2i (p(0), p(1) + 1);
              }

              if (p(1) > 0 && !lpp[-width] && !pcl_isnan (dpp[-dstep].z)
                  && std::abs (dp.z - dpp[-dstep].z) <= speckle_range_)
              {
                lpp[-width] = curlabel;
                *ws++ = Eigen::Vector2i (p(0), p(1) - 1);
              }

              // pop most recent and propagate
              // NB: could try least recent, maybe better convergence
              p = *--ws;

            }

            // assign label type
            if (count <= speckle_size_) // speckle region
            {
              points_to_remove_->indices.push_back (j + i * input_->width);
              rtype[ls[j]] = 1; // small region label
            }
            else
              rtype[ls[j]] = 0; // large region label
          }
        }
        else
          points_to_remove_->indices.push_back (j + i * input_->width);
      }
    }

    delete[] rem_buf;

    if (&pc_out != input_.get ())
      pc_out = *input_;

    for (size_t i = 0; i < points_to_remove_->indices.size (); i++)
    {
      size_t j = points_to_remove_->indices[i];
      pc_out[j].x = pc_out[j].y = pc_out[j].z = std::numeric_limits<float>::quiet_NaN ();
    }
    points_to_remove_->indices.clear ();

//    //TODO: add param to actually not remove the points
//    //TODO: add param to keep the point cloud ordered
//    pcl::ExtractIndices<PointT> extractIndices;
//    extractIndices.setInputCloud (input_);
//    extractIndices.setIndices (points_to_remove_);
//    extractIndices.setNegative (true);
//    extractIndices.filter (pc_out);

  }

#define PCL_INSTANTIATE_SpeckleFilter(T) template class cob_3d_mapping_filters::SpeckleFilter<T>;
#endif /* SPECKLE_FILTER_HPP_ */
