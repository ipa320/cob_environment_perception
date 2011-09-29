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
 * Author: Waqas Tanveer, email:waqas.informatik@googlemail.com
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2011

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
#ifndef SPECKLE_FILTER_HPP_
#define SPECKLE_FILTER_HPP_

//##################
//#### includes ####

// cob_env_model includes
#include "cob_env_model/filters/speckle_filter.h"

template<typename PointT>
void
cob_env_model::SpeckleFilter<PointT>::applyFilter (PointCloud &pc_out)
{
  //std::cout << " Entered apply filter method " << std::endl;
  cv::Mat xyz_mat_32F3 = cv::Mat (input_->height, input_->width, CV_32FC3);
  pc_out.points.resize(input_->points.size());
  pc_out.header = input_->header;

  for (unsigned int i = 0; i < input_->points.size(); i++)
    pc_out.points[i] = input_->points[i];

  float* f_ptr = 0;
  int pc_msg_idx = 0;
  for (int row = 0; row < xyz_mat_32F3.rows; row++)
  {
    f_ptr = xyz_mat_32F3.ptr<float> (row);
    for (int col = 0; col < xyz_mat_32F3.cols; col++, pc_msg_idx++)
    {
      memcpy (&f_ptr[3 * col], &input_->points[pc_msg_idx].x, 3 * sizeof(float));
    }
  }

  cv::Mat buf;
  this->filterSpeckles(xyz_mat_32F3, speckle_size_, speckle_range_, buf);
  pc_msg_idx = 0;
  for (int row = 0; row < xyz_mat_32F3.rows; row++)
  {
    f_ptr = xyz_mat_32F3.ptr<float> (row);
    for (int col = 0; col < xyz_mat_32F3.cols; col++, pc_msg_idx++)
    {
      memcpy (&pc_out.points[pc_msg_idx].x, &f_ptr[3 * col], 3 * sizeof(float));
    }
  }
}

template<typename PointT>
void
cob_env_model::SpeckleFilter<PointT>::filterSpeckles(cv::Mat& img, int maxSpeckleSize, double maxDiff, cv::Mat& _buf )
{
  CV_Assert( img.type() == CV_32FC3 );


  float newVal = 0;
  int width = img.cols, height = img.rows, npixels = width*height;
  size_t bufSize = npixels*(int)(sizeof(cv::Point_<short>) + sizeof(int) + sizeof(uchar));
  if( !_buf.isContinuous() || !_buf.data || _buf.cols*_buf.rows*_buf.elemSize() < bufSize )
    _buf.create(1, bufSize, CV_8U);

  uchar* buf = _buf.data;
  int i, j, dstep = img.step/sizeof(cv::Vec3f);
  int* labels = (int*)buf;
  buf += npixels*sizeof(labels[0]);
  cv::Point_<short>* wbuf = (cv::Point_<short>*)buf;
  buf += npixels*sizeof(wbuf[0]);
  uchar* rtype = (uchar*)buf;
  int curlabel = 0;

  // clear out label assignments
  memset(labels, 0, npixels*sizeof(labels[0]));

  for( i = 0; i < height; i++ )
  {
    cv::Vec3f* ds = img.ptr<cv::Vec3f>(i);
    int* ls = labels + width*i;

    for( j = 0; j < width; j++ )
    {
      if( ds[j][2] != newVal )    // not a bad disparity
          {
        if( ls[j] )             // has a label, check for bad label
        {
          if( rtype[ls[j]] ) // small region, zero out disparity
          {
            ds[j][0] = (float)newVal;
            ds[j][1] = (float)newVal;
            ds[j][2] = (float)newVal;
          }
        }
        // no label, assign and propagate
        else
        {
          cv::Point_<short>* ws = wbuf;       // initialize wavefront
          cv::Point_<short> p((short)j, (short)i);    // current pixel
          curlabel++; // next label
          int count = 0;      // current region size
          ls[j] = curlabel;

          // wavefront propagation
          while( ws >= wbuf ) // wavefront not empty
          {
            count++;
            // put neighbors onto wavefront
            cv::Vec3f* dpp = &img.at<cv::Vec3f>(p.y, p.x);
            cv::Vec3f dp = *dpp;
            int* lpp = labels + width*p.y + p.x;

            if( p.x < width-1 && !lpp[+1] && dpp[+1][2] != newVal && std::abs(dp[2] - dpp[+1][2]) <= maxDiff )
            {
              lpp[+1] = curlabel;
              *ws++ = cv::Point_<short>(p.x+1, p.y);
            }

            if( p.x > 0 && !lpp[-1] && dpp[-1][2] != newVal && std::abs(dp[2] - dpp[-1][2]) <= maxDiff )
            {
              lpp[-1] = curlabel;
              *ws++ = cv::Point_<short>(p.x-1, p.y);
            }

            if( p.y < height-1 && !lpp[+width] && dpp[+dstep][2] != newVal && std::abs(dp[2] - dpp[+dstep][2]) <= maxDiff )
            {
              lpp[+width] = curlabel;
              *ws++ = cv::Point_<short>(p.x, p.y+1);
            }

            if( p.y > 0 && !lpp[-width] && dpp[-dstep][2] != newVal && std::abs(dp[2] - dpp[-dstep][2]) <= maxDiff )
            {
              lpp[-width] = curlabel;
              *ws++ = cv::Point_<short>(p.x, p.y-1);
            }

            // pop most recent and propagate
            // NB: could try least recent, maybe better convergence
            p = *--ws;
          }

          // assign label type
          if( count <= maxSpeckleSize )       // speckle region
          {
            rtype[ls[j]] = 1;       // small region label
            ds[j][0] = (float)newVal;
            ds[j][1] = (float)newVal;
            ds[j][2] = (float)newVal;
          }
          else
            rtype[ls[j]] = 0;       // large region label
        }
          }
    }
  }
  return;
}

#define PCL_INSTANTIATE_SpeckleFilter(T) template class cob_env_model::SpeckleFilter<T>;
#endif /* SPECKLE_FILTER_HPP_ */
