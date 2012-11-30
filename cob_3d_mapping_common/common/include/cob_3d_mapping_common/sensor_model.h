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
 *  ROS package name: cob_3d_mapping_common
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
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

#ifndef __SENSOR_MODEL_H__
#define __SENSOR_MODEL_H__

namespace cob_3d_mapping
{
  class ScanlineEdge
  {
    public:
    ScanlineEdge(int id_, float x1, float y1, float x2, float y2)
      : id(id_)
    {
      if(y1>y2) { ymax = y1; ymin = y2; }
      else { ymax = y2; ymin = y1; }
      mInv = (x2-x1)/(y2-y1);
      t = mInv*y1/x1;
    }

    inline float intersection(float y) const { return mInv*y-t; }

    int id;
    float ymin;
    float ymax;
    float mInv;
    float t;
  };

  inline const bool operator<  (const ScanlineEdge& lhs, const ScanlineEdge& rhs) { return lhs.ymin < rhs.ymin; }
  inline const bool operator>  (const ScanlineEdge& lhs, const ScanlineEdge& rhs) { return  operator< (rhs, lhs); }
  inline const bool operator<= (const ScanlineEdge& lhs, const ScanlineEdge& rhs) { return !operator> (lhs, rhs); }
  inline const bool operator>= (const ScanlineEdge& lhs, const ScanlineEdge& rhs) { return !operator< (lhs, rhs); }

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
    { }

    inline void addEdge(int id, float x1, float y1, float x2, float y2) {
      yque.push_back(ScanlineEdge(id,x1,y1,x2,y2));
    }

    inline int x2w(float x) const {
      return int((x - xmin) / (xmax - xmin) * w)
    }

    inline int y2h(float y) const {
      return int((y - ymin) / (ymax - ymin) * h)
    }

    void fill(std::vector<std::vector<int> >& out)
    {
      out.resize(w*h);
      yque.sort();

      float xstep = (xmax - xmin) / float(w);
      float ystep = (ymax - ymin) / float(h);
      int idx_y = 0;
      for(float y=ymin + 0.5f * ystep; y<ymax; y+=ystep, ++idx_y)
      {
        std::list<std::pair<float,int> > xque;
        std::list<ScanlineEdge>::iterator ycurr = yque.begin();
        while (ycurr->ymin < y)
        {
          if(ycurr->ymax < y) {
            ycurr = yque.erase(curr);
          }
          else {
            xque.push_back(std::pair<float,int>(ycurr.intersection(y),ycurr.id));
            ++curr;
          }
        }

        std::list<std::pair<float,int> >::iterator xcurr = xque.begin();
        std::list<std::pair<float,int> >::iterator xprev = xcurr++;
        std::set<int> curr_ids;
        while (xcurr!=xque.end())
        {
          std::pair<std::set<int>::iterator, bool> res = curr_ids.insert(xprev.second);
          if( !res.second ) curr_ids.erase(res.first);
          if( curr_ids.size() )
          {
            for(float x=xprev->first; x<xcurr->first; x+=xstep)
            {
              for(std::set<int>::iterator id = curr_ids.begin(); id != curr_ids.end(); ++id) {
                out[ y * h + x2w(x) ].push_back(*id);
              }
            }
          }
          xprev = xcurr++;
        }
      }
    }

    int w;
    int h;
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    std::list<ScanlineEdge> yque;
  };

  class PrimeSense
  {
    public:
    PrimeSense() {}
    ~PrimeSense() {}

    static bool areNeighbors(float query, float neighbor, float tolerance = 2.0f)
    {
      float dist_th = tolerance * 0.003f * query * query;
      //if (query < 1.2) dist_th += 0.01f;
      return (fabs(query - neighbor) < dist_th);
    }

    // specialized for cob_3d_mapping::Polygon
    static void perspectiveProjection(const Eigen::Matrix4f& camera_transform, // transformation from world to camera
                                      const std::vector<cob_3d_mapping::Polygon::Ptr>& polygons, // input polygons
                                      int w, int h, // size of projected image
                                      std::vector<std::vector<int> >& projection) // image vector with polygon ids
    {
      // create transformation matrix from sensor position and camera parameters
      Eigen::Matrix4f proj = Eigen::Matrix4f( &PrimeSense::mat ) * camera_transform;
      // transform polygons to clipping space and reject outliers ( space: x,y,z E [-1,1] )
      for(int i=0; i<polygons.size(); ++i)
      {
        for(int c=0; c<polygons[i]->contours.size(); ++c)
        {
          Eigen::Vector4f pprev = proj * Eigen::Vector4f(polygons[i]->contours[c][polygons[i]->contours[c].size()-1], 1);
          for(int p=0; p<polygons[i]->contours[c].size(); ++p)
          {
            Eigen::Vector4f pcurr = proj * Eigen::Vector4f(polygons[i]->contours[c][p], 1);
            if(pcurr(3) != 1.0) std::cout << pcurr(3) << std::endl;
            //if(pcurr(
          }
        }
      }
      // rasterize remaining polygons using scanline rendering algorithm
    }

    // frustum parameters:
    static const float horizontal; // fov horizontal (IR) , from wiki and ros.org
    static const float vertical; // fov vertical (IR)
    static const float f; // far plane
    static const float n; // near plane
    static const float l; // left
    static const float r; // right
    static const float t; // top
    static const float b; // bottom
    static const float mat[]; // transformation from view frustum to unit cube
  };

  const float PrimeSense::horizontal = 57.0f / 180.0f * M_PI;
  const float PrimeSense::vertical = 43.0f / 180.0f * M_PI;
  const float PrimeSense::f = 5.0f;
  const float PrimeSense::n = 0.8f;
  const float PrimeSense::r = PrimeSense::n * tan(PrimeSense::horizontal * 0.5f);
  const float PrimeSense::l = - PrimeSense::r;
  const float PrimeSense::t = PrimeSense::n * tan(PrimeSense::vertical * 0.5f);
  const float PrimeSense::b = - PrimeSense::t;

  const float PrimeSense::mat[] = {
    2*PrimeSense::n/(PrimeSense::r-PrimeSense::l), 0, - (PrimeSense::r+PrimeSense::l)/(PrimeSense::r-PrimeSense::l), 0,
    0, 2*PrimeSense::n/(PrimeSense::t-PrimeSense::b), - (PrimeSense::t+PrimeSense::b)/(PrimeSense::t-PrimeSense::b), 0,
    0, 0, (PrimeSense::n+PrimeSense::f)/(PrimeSense::n-PrimeSense::f), -2*PrimeSense::f*PrimeSense::n/(PrimeSense::n-PrimeSense::f),
    0, 0, 1, 0
  };
}

#endif
