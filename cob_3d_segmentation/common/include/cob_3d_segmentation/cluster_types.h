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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2012
 * ToDo:
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

#ifndef __COB_3D_SEGMENTATION_CLUSTER_TYPES_H__
#define __COB_3D_SEGMENTATION_CLUSTER_TYPES_H__

#include <pcl/common/eigen.h>
#include <set>
#include <cob_3d_segmentation/polygon_extraction/polygon_types.h>

namespace cob_3d_segmentation
{
  template<typename T>
    inline T min3(const T& a, const T& b, const T& c) { return ( a < b ? std::min<T>(a,c) : std::min<T>(b,c) ); }
  template<typename T>
    inline T max3(const T& a, const T& b, const T& c) { return ( a > b ? std::max<T>(a,c) : std::max<T>(b,c) ); }

    class DominantColor
    {
      private:
      enum { HIST_SIZE = 180 }; // uint8_t limits hue to 0..255

      public:
      DominantColor() : sum_colors_(0), sum_sat_(0), sum_val_(0), hue_histogram_(HIST_SIZE,0), sat_values_(HIST_SIZE,0)
      { }

      ~DominantColor() { }

      void addColor(uint8_t r, uint8_t g, uint8_t b)
      {
        int h,s,v;
        rgb2hsv(r,g,b,h,s,v);
        int pos = incrBin(h);
        sat_values_[pos] += s;
        sum_sat_ += s;
        sum_val_ += v;
        ++sum_colors_;
      }

      inline void getColor(uint8_t& r, uint8_t& g, uint8_t& b) const
      {
        if(!sum_colors_) { r=0; g=0; b=0; return; }
        int pos = getMaxBin();
        hsv2rgb( round(pos*bin_size+bin_center), sat_values_[pos]/hue_histogram_[pos], sum_val_/sum_colors_, r,g,b );
      }

      inline int incrBin(int h)
      { int bin = h*inv_bin_size; hue_histogram_[bin] += 1; return bin; }

      inline int getMaxBin() const
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
        std::cout<<"max_(bin/size): "<<max_bin<<"/"<<max_size<<std::endl;
        return max_bin;
      }

      inline void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, int& h, int& s, int& v) const
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

      inline void hsv2rgb(int h, int s, int v, uint8_t& r, uint8_t& g, uint8_t& b) const
      {
        if (s == 0) { r = g = b = v; return; }

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

      private:
      int sum_colors_;
      int sum_sat_;
      int sum_val_;
      std::vector<int> hue_histogram_;
      std::vector<int> sat_values_;

      static const float inv_bin_size = 1.0f / 360.0f * HIST_SIZE;
      static const float bin_size = 360.0f / HIST_SIZE;
      static const float bin_center = ((360.0f / HIST_SIZE) - 1.0f) * 0.5f;
    };


  class ClusterBase
  {
  public:
    typedef std::vector<int>::iterator iterator;
    typedef std::vector<int>::size_type size_type;
    typedef std::vector<int>::value_type value_type;

  public:
    explicit ClusterBase(int id)
      : id_(id)
      , indices_()
    { };
    virtual ~ClusterBase() { }

    inline const int id() const { return id_; }

    inline value_type& operator[](size_type idx) { return indices_.at(idx); }
    inline iterator begin() { return indices_.begin(); }
    inline iterator end() { return indices_.end(); }
    inline size_type size() const { return indices_.size(); }

    inline void addIndex(int idx) { indices_.push_back(idx); }

    std::vector<int> indices_;
  protected:
    int id_;

  };

  class DepthCluster : public ClusterBase
  {
    template <typename LabelT, typename PointT, typename PointNT> friend class DepthClusterHandler;

  public:
    DepthCluster(int id)
      : ClusterBase(id)
      , type(I_UNDEF)
      , type_probability(1.0)
      , is_save_plane(false)
      , max_curvature(0.0)
      , min_curvature(0.0)
      , pca_point_comp1(0.0, 0.0, 0.0)
      , pca_point_comp2(0.0, 0.0, 0.0)
      , pca_point_comp3(0.0, 0.0, 0.0)
      , pca_point_values(0.0, 0.0, 0.0)
      , pca_inter_centroid(0.0, 0.0, 0.0)
      , pca_inter_comp1(0.0, 0.0, 0.0)
      , pca_inter_comp2(0.0, 0.0, 0.0)
      , pca_inter_comp3(0.0, 0.0, 0.0)
      , pca_inter_values(0.0, 0.0, 0.0)
      , border_points()
      , sum_points_(0.0, 0.0, 0.0)
      , sum_orientations_(0.0, 0.0, 0.0)
      , sum_rgb_(0,0,0)
    { }

    inline Eigen::Vector3f getCentroid() const { return sum_points_ / (float)indices_.size(); }
    inline Eigen::Vector3f getOrientation() const { return sum_orientations_ / (float)indices_.size(); }
    inline Eigen::Vector3i getMeanColorVector() const { return sum_rgb_ / indices_.size(); }
    inline int getMeanColorValue() const { Eigen::Vector3i c = getMeanColorVector(); return (c(0) << 16 | c(1) << 8 | c(2)); }
    inline Eigen::Vector3i computeDominantColorVector() const
    { uint8_t r,g,b; color_.getColor(r,g,b); return Eigen::Vector3i(r,g,b); }

  public:
    int type;
    float type_probability;
    bool is_save_plane;
    float max_curvature;
    float min_curvature;

    Eigen::Vector3f pca_point_comp1;
    Eigen::Vector3f pca_point_comp2;
    Eigen::Vector3f pca_point_comp3;
    Eigen::Vector3f pca_point_values;

    Eigen::Vector3f pca_inter_centroid;
    Eigen::Vector3f pca_inter_comp1;
    Eigen::Vector3f pca_inter_comp2;
    Eigen::Vector3f pca_inter_comp3;
    Eigen::Vector3f pca_inter_values;

    std::vector<PolygonPoint> border_points;

  private:
    Eigen::Vector3f sum_points_;
    Eigen::Vector3f sum_orientations_;
    Eigen::Vector3i sum_rgb_;
    DominantColor color_;
  };

  inline const bool operator< (const ClusterBase& lhs, const ClusterBase& rhs){return lhs.size() < rhs.size();}
  inline const bool operator> (const ClusterBase& lhs, const ClusterBase& rhs){return  operator< (rhs, lhs);}
  inline const bool operator<=(const ClusterBase& lhs, const ClusterBase& rhs){return !operator> (lhs, rhs);}
  inline const bool operator>=(const ClusterBase& lhs, const ClusterBase& rhs){return !operator< (lhs, rhs);}
}

#endif
