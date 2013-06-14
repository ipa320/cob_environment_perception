/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2012 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_environment_perception
 * \note
 * ROS package name: cob_3d_mapping_common
 *
 * \author
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 05/2013
 *
 * \brief
 * Class representing a dominant color
 *
 *****************************************************************
 *polygon.h
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
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

#include <cob_3d_mapping_common/dominant_color.h>

namespace cob_3d_mapping
{
  void DominantColor::addColor(uint8_t r, uint8_t g, uint8_t b, int weight)
  {
    /*if(id_==0) std::cout << "rgb_in" << (int)r << "," << (int)g << "," << (int)b << "\tweight:" << weight << std::endl;
      int h,s,v;
      rgb2hsv(r,g,b,h,s,v);
      if(id_==0) std::cout << "hsv_in" << (int)h << "," << (int)s << "," << (int)v << std::endl;*/
    for(int i=0; i< weight; i++)
    {
      /*int pos = incrBin(h);
        sat_values_[pos] += s;
        sum_sat_ += s;
        sum_val_ += v;
        ++sum_colors_;*/
      sum_r_ += r;
      sum_g_ += g;
      sum_b_ += b;
      ++sum_colors_;
    }
  }

  void DominantColor::getColor(uint8_t& r, uint8_t& g, uint8_t& b) const
  {
    if(!sum_colors_) { r=0; g=0; b=0; return; }
    /*int pos = getMaxBin();
      if(id_==0) std::cout << "hsv_out" << round(pos*bin_size+bin_center) << "," << sat_values_[pos]/hue_histogram_[pos] << "," << sum_val_/sum_colors_ << std::endl;
      hsv2rgb( round(pos*bin_size+bin_center), sat_values_[pos]/hue_histogram_[pos], sum_val_/sum_colors_, r,g,b );
      if(id_==0) std::cout << "rgb_out" << (int)r << "," << (int)g << "," << (int)b << std::endl;*/
    r = sum_r_/sum_colors_;
    g = sum_g_/sum_colors_;
    b = sum_b_/sum_colors_;
  }

  int DominantColor::incrBin(int h)
  { int bin = h*inv_bin_size; hue_histogram_[bin] += 1; return bin; }

  int DominantColor::getMaxBin() const
  {
    int max_bin=0, max_size=0;
    for(int i=0;i<HIST_SIZE;++i)
    {
      if (hue_histogram_[i] >= max_size)
      {
        max_size = hue_histogram_[i];
        max_bin = i;
      }
    }
    //std::cout<<"max_(bin/size): "<<max_bin<<"/"<<max_size<<std::endl;
    return max_bin;
  }

  void DominantColor::rgb2hsv(uint8_t r, uint8_t g, uint8_t b, int& h, int& s, int& v) const
  {
    int rgb_min = min3(r,g,b);
    int rgb_max = max3(r,g,b);
    int delta = rgb_max - rgb_min;
    v = rgb_max;
    if (v == 0) { s = h = 0; return; }
    s = round(float(delta) / float(v) * 100.0f);
    v /= 2.55f;
    float h_tmp;
    if (s == 0) { h = 0; return; }
    if      ((int)r == rgb_max)   h_tmp =     (float(g - b)) / (float(delta));
    else if ((int)g == rgb_max)   h_tmp = 2.0f + (float(b - r)) / (float(delta));
    else                          h_tmp = 4.0f + (float(r - g)) / (float(delta));

    h = h_tmp * 60.0f;
    if(h<0) h+=360;
  }

  void DominantColor::hsv2rgb(int h, int s, int v, uint8_t& r, uint8_t& g, uint8_t& b) const
  {
    if (s == 0) { r = g = b = v*2.55f; return; }

    float hh = h / 60.0f; // sector 0..5
    int i = floor(hh);
    float f = hh - i;
    v = round(v*2.55f);
    int p = v * (100 - s) * 0.01f;
    int q = v * (100 - s * f) * 0.01f;
    int t = v * (100 - s * (1.0f - f)) * 0.01f;

    switch(i)
    {
      case 0:
      { r = v; g = t; b = p; break; }
      case 1:
      { r = q; g = v; b = p; break; }
      case 2:
      { r = p; g = v; b = t; break; }
      case 3:
      { r = p; g = q; b = v; break; }
      case 4:
      { r = t; g = p; b = v; break; }
      default:// case 5:
      { r = v; g = p; b = q; break; }
    }
  }

  //inline void setID(unsigned int id) {id_=id;}

  void DominantColor::reset()
  {
    sum_colors_ = sum_sat_ = sum_val_ = 0;
    sum_r_ = sum_b_ = sum_g_ = 0;
    hue_histogram_.clear();
    hue_histogram_.resize(HIST_SIZE,0);
    sat_values_.clear();
    sat_values_.resize(HIST_SIZE,0);
  }
}
