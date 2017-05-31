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
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_fov_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 09/2013
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

#ifndef SCANLINE_POLYGON_FILL_H
#define SCANLINE_POLYGON_FILL_H

namespace cob_3d_mapping
{
  /* used to perform z-calculations at each pixel */
  class ScanlinePolygon
  {
  public:
    ScanlinePolygon(const Eigen::Vector3f& n_, float d_)
    {
      float invN2 = 1.0f / n_(2);
      c(0) = d_ * invN2;
      c(1) = n_(0) * invN2;
      c(2) = n_(1) * invN2;
    }

    inline float intersection(float x, float y) const
    { return c(0) - c(1)*x - c(2)*y; }

    Eigen::Vector3f c;
  };

  /* data structure used by ScanlinePolygonFill
   * represents a line of a 2D polygon
   */
  template<typename T = int>
  class ScanlineEdge
  {
  public:
    ScanlineEdge(T id_, float x1, float y1, float x2, float y2)
      : id(id_)
      , x1_(x1)
      , y1_(y1)
      , x2_(x2)
      , y2_(y2)
    {
      if(y1>y2) { ymax = y1; ymin = y2; }
      else { ymax = y2; ymin = y1; }
      mInv = (x2-x1)/(y2-y1);
      t = mInv*y1/x1;
    }

    inline float intersection(float y) const { return mInv*(y-y1_)+x1_; }

    T id;
    float ymin;
    float ymax;
    float mInv;
    float t;
    float x1_;
    float y1_;
    float x2_;
    float y2_;
  };

  template<typename T>
  inline const bool operator< (const ScanlineEdge<T>& lhs,
                               const ScanlineEdge<T>& rhs)
  { return lhs.ymin < rhs.ymin; }

  template<typename T>
  inline const bool operator> (const ScanlineEdge<T>& lhs,
                               const ScanlineEdge<T>& rhs)
  { return  operator< (rhs, lhs); }

  template<typename T>
  inline const bool operator<= (const ScanlineEdge<T>& lhs,
                                const ScanlineEdge<T>& rhs)
  { return !operator> (lhs, rhs); }

  template<typename T>
  inline const bool operator>= (const ScanlineEdge<T>& lhs,
                                const ScanlineEdge<T>& rhs)
  { return !operator< (lhs, rhs); }


  /* Algorithm for rasterization of 2D images consisting of polygon lines
   * UPDATE: now modified to perform x,y,z clipping
   * Source: http://en.wikipedia.org/wiki/Flood_fill
   *         common practice in computer graphics
   * Output: image of which each pixel consists of a vector of ids
   */
  template<typename T = int>
  class ScanlinePolygonFill
  {
  public:
    ScanlinePolygonFill(int width, int height)
      : w(width)
      , h(height)
      , xmin(-1.0f)
      , xmax( 1.0f)
      , ymin(-1.0f)
      , ymax( 1.0f)
      , zmin(-1.0f)
      , zmax( 1.0f)
      , zthr( 0.2f)
    { }

    inline void addEdge(T id, float x1, float y1, float x2, float y2)
    {
      yque.push_back(ScanlineEdge<T>(id,x1,y1,x2,y2));
    }

    inline bool addPolygon(T id, const Eigen::Vector3f& normal, float d)
    {
      /*std::cout << "addPolygon: " << id <<": "<<normal(0)
        <<" x + "<<normal(1)<<" y + "<<normal(2)<<" z = "<<d<<std::endl;*/
      return polys.insert( std::make_pair(id, ScanlinePolygon(normal,d)) ).second;
    }

    inline int x2w(float x) const
    {
      return int((x - xmin) / (xmax - xmin) * float(w));
    }

    inline int y2h(float y) const
    {
      return int((y - ymin) / (ymax - ymin) * float(h));
    }

    // only draws lines of polygons
    void draw(std::vector<std::vector<T> >& out);

    // fills areas of polygons
    void fill(std::vector<std::vector<T> >& out);

  private:
    int w;
    int h;
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;
    float zthr;
    std::list<ScanlineEdge<T> > yque;
    std::map<T,ScanlinePolygon> polys;
  };
}

#include "cob_3d_fov_segmentation/impl/scanline_polygon_fill.hpp"

#endif
