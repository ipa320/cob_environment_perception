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

#ifndef DOMINANT_COLOR_H_
#define DOMINANT_COLOR_H_

#include <stdint.h>
#include <math.h>
#include <algorithm>
#include <vector>

namespace cob_3d_mapping
{
  template<typename T>
    inline T
    min3 (const T& a, const T& b, const T& c)
    {
      return (a < b ? std::min<T> (a, c) : std::min<T> (b, c));
    }
  template<typename T>
    inline T
    max3 (const T& a, const T& b, const T& c)
    {
      return (a > b ? std::max<T> (a, c) : std::max<T> (b, c));
    }

  class DominantColor
  {
  public:
    DominantColor () :
        sum_colors_ (0), sum_sat_ (0), sum_val_ (0), sum_r_ (0), sum_g_ (0), sum_b_ (0), hue_histogram_ (HIST_SIZE, 0), sat_values_ (
            HIST_SIZE, 0)
    {
    }

    ~DominantColor ()
    {
    }

    void
    addColor (uint8_t r, uint8_t g, uint8_t b, int weight);
    void
    getColor (uint8_t& r, uint8_t& g, uint8_t& b) const;
    int
    incrBin (int h);
    int
    getMaxBin () const;
    void
    rgb2hsv (uint8_t r, uint8_t g, uint8_t b, int& h, int& s, int& v) const;
    void
    hsv2rgb (int h, int s, int v, uint8_t& r, uint8_t& g, uint8_t& b) const;
    void
    reset ();

  private:
    enum
    {
      HIST_SIZE = 180
    }; // uint8_t limits hue to 0..255
    int sum_colors_;
    int sum_sat_;
    int sum_val_;
    int sum_r_, sum_g_, sum_b_;
    std::vector<int> hue_histogram_;
    std::vector<int> sat_values_;

    static const float inv_bin_size = 1.0f / 360.0f * HIST_SIZE;
    static const float bin_size = 360.0f / HIST_SIZE;
    static const float bin_center = ((360.0f / HIST_SIZE) - 1.0f) * 0.5f;
  };
}
#endif /* DOMINANT_COLOR_H_ */
