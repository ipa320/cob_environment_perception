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
 *  ROS package name: cob_3d_mapping_tools
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2012
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

#ifndef COB_3D_MAPPING_TOOLS_GUI_TOOLS_HPP_
#define COB_3D_MAPPING_TOOLS_GUI_TOOLS_HPP_

#include <limits>

template<typename PointT>
void Gui::Tools::convertPointCloud2Cv(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, cvImagePtr& image,
                                                     boost::function<cv::Vec3b (int)>& converter)
{
  image = new cvImage(cloud->height, cloud->width, CV_8UC3);
  int idx=0;
  for (int row = 0; row < image->rows; row++)
  {
    for (int col = 0; col < image->cols; col++, idx++)
    {
      (*image)(row, col) = converter(idx);
    }
  }
}

template<typename PointT>
void Gui::Tools::getMinMaxZ(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, float& z_min, float& z_max)
{
  z_min = std::numeric_limits<float>::max();
  z_max = std::numeric_limits<float>::min();
  for(typename pcl::PointCloud<PointT>::const_iterator it=cloud->begin(); it!=cloud->end(); ++it)
  {
    z_min = std::min(z_min,it->z);
    z_max = std::max(z_max,it->z);
  }
}



uint32_t Gui::Tools::getGradientColor(float z_value, float z_min, float z_max, cv::Vec3b& bgr)
{
  return getGradientColor( (z_value - z_min) / (z_max - z_min), bgr );
}

// color is proportional to position  <0;1>
// position means position of color in color gradient
uint32_t Gui::Tools::getGradientColor(double position, cv::Vec3b& bgr)
{
  //if (position > 1) position = position - int(position);
  if (position > 1) position = 1;
  if (position < 0) position = 0;
  // if position > 1 then we have repetition of colors
  // it maybe useful
  int n_bars_max = 4;
  double m=n_bars_max * position;
  int n=int(m); // integer of m
  double f=m-n;  // fraction of m
  uint8_t t=int(f*255);

  switch (n)
  {
  case 0:
  { bgr[2] = 255; bgr[1] = t; bgr[0] = 0; break; }
  case 1:
  { bgr[2] = 255 - t; bgr[1] = 255; bgr[0] = 0; break; }
  case 2:
  { bgr[2] = 0; bgr[1] = 255; bgr[0] = t; break; }
  case 3:
  { bgr[2] = 0; bgr[1] = 255 - t; bgr[0] = 255; break; }
  case 4:
  { bgr[2] = t; bgr[1] = 0; bgr[0] = 255; break; }
  case 5:
  { bgr[2] = 255; bgr[1] = 0; bgr[0] = 255 - t; break; }
  };
  return (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
}

#endif
