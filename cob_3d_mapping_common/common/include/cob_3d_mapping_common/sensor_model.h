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

#include "cob_3d_mapping_common/polygon.h"
#include <list>

namespace cob_3d_mapping
{
  /* data structure used by ScanlinePolygonFill
   * represents a line of a 2D polygon
   */
  class ScanlineEdge
  {
    public:
    ScanlineEdge(int id_, float x1, float y1, float x2, float y2)
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

    int id;
    float ymin;
    float ymax;
    float mInv;
    float t;
    float x1_;
    float y1_;
    float x2_;
    float y2_;
  };

  inline const bool operator<  (const ScanlineEdge& lhs, const ScanlineEdge& rhs) { return lhs.ymin < rhs.ymin; }
  inline const bool operator>  (const ScanlineEdge& lhs, const ScanlineEdge& rhs) { return  operator< (rhs, lhs); }
  inline const bool operator<= (const ScanlineEdge& lhs, const ScanlineEdge& rhs) { return !operator> (lhs, rhs); }
  inline const bool operator>= (const ScanlineEdge& lhs, const ScanlineEdge& rhs) { return !operator< (lhs, rhs); }

  /* Algorithm for rasterization of 2D images consisting of polygon lines
   * Source: http://en.wikipedia.org/wiki/Flood_fill
   *         common practice in computer graphics
   * Output: image of which each pixel consists of a vector of ids
   */
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
      return int((x - xmin) / (xmax - xmin) * float(w));
    }

    inline int y2h(float y) const {
      return int((y - ymin) / (ymax - ymin) * float(h));
    }

    void draw(std::vector<std::vector<int> >& out)
    {
      out.resize(w*h);
      yque.sort();

      float xstep = (xmax - xmin) / float(w);
      float ystep = (ymax - ymin) / float(h);
      for(float y=ymin + 0.5f * ystep; y<ymax; y+=ystep)
      {
        //std::cout << "Line: "<< y2h(y) <<" y="<< y << std::endl;
        int yidx = y2h(y)*w;
        std::list<std::pair<float,int> > xque;
        std::list<ScanlineEdge>::iterator ycurr = yque.begin();
        while (ycurr != yque.end() && ycurr->ymin < y)
        {
          if(ycurr->ymax <= y) {
            ycurr = yque.erase(ycurr);
          }
          else {
            out[ yidx + x2w(ycurr->intersection(y)) ].push_back(ycurr->id);
            ++ycurr;
          }
        }
      }
    }

    void fill(std::vector<std::vector<int> >& out)
    {
      out.resize(w*h);
      yque.sort();

      float xstep = (xmax - xmin) / float(w);
      float ystep = (ymax - ymin) / float(h);
      for(float y=ymin + 0.5f * ystep; y<ymax; y+=ystep)
      {
        //std::cout << "Line: "<< y2h(y) <<" y="<< y << std::endl;
        int yidx = y2h(y)*w;
        std::list<std::pair<float,int> > xque;
        std::list<ScanlineEdge>::iterator ycurr = yque.begin();
        while (ycurr != yque.end() && ycurr->ymin < y)
        {
          if(ycurr->ymax <= y) {
            ycurr = yque.erase(ycurr);
          }
          else {
            //std::cout << ycurr->x1_ <<","<< ycurr->y1_ <<" --- "<< ycurr->x2_ <<","<< ycurr->y2_
            //          << " -> " << ycurr->intersection(y) << "," << y << std::endl;
            xque.push_back(std::pair<float,int>(ycurr->intersection(y),ycurr->id));
            ++ycurr;
          }
        }
        xque.sort();
        std::list<std::pair<float,int> >::iterator xcurr = xque.begin();
        std::list<std::pair<float,int> >::iterator xprev = xcurr++;
        std::set<int> curr_ids;
        while (xcurr!=xque.end())
        {
          std::pair<std::set<int>::iterator, bool> res = curr_ids.insert(xprev->second);
          if( !res.second ) curr_ids.erase(res.first);
          if( curr_ids.size() )
          {
            for(float x=xprev->first; x<xcurr->first; x+=xstep)
            {
              for(std::set<int>::iterator id = curr_ids.begin(); id != curr_ids.end(); ++id) {
                out[ yidx + x2w(x) ].push_back(*id);
              }
            }
          }
          xprev = xcurr++;
        }
      }
    }

    private:
    int w;
    int h;
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    std::list<ScanlineEdge> yque;
  };

  /* 2D/3D line clipping algorithm
   * Source: http://en.wikipedia.org/wiki/Cohen-Sutherland (did not find a paper about it)
   */
  class CohenSutherlandClipping
  {
    public:
    CohenSutherlandClipping()
      : xmin(-1.0f)
      , xmax( 1.0f)
      , ymin(-1.0f)
      , ymax( 1.0f)
      , zmin(-1.0f)
      , zmax( 1.0f)
    { }

    void setBorders(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
    {
      xmin = x_min; xmax = x_max; ymin = y_min; ymax = y_max; zmin = z_min; zmax = z_max;
    }

    int computeCode(float x, float y, float z=0.0f)
    {
      int code = 0;
      if( x < xmin )     code |= LEFT;
      else if( x > xmax) code |= RIGHT;
      if( y < ymin )     code |= TOP;
      else if( y > ymax) code |= BOTTOM;
      if( z < zmin )     code |= NEAR;
      else if( z > zmax) code |= FAR;

      return code;
    }

    bool clip(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, Eigen::Vector3f& q0, Eigen::Vector3f& q1)
    {
      int code0 = computeCode(p0(0),p0(1),p0(2));
      int code1 = computeCode(p1(0),p1(1),p1(2));
      bool accept = false;

      if( !(code0 | code1) ) { // trivial accept
        accept = true;
        q0 = p0;
        q1 = p1;
      }
      else if( !(code0 & code1) ) {
        /* Line:  q = (p1 - p0) * d + p0
         * Plane: (q - c).dot(n)           c: point on plane, n: normal of plane
         *
         *           (p0 - c).dot(n)
         * => d =  ------------------
         *          (p1 - p0).dot(n)
         */
        float x, y, z, d;
        int outcode;
        Eigen::Vector3f *pnew;
        const Eigen::Vector3f *pold;
        if(code0) { // check which point lies outside
          outcode = code0;
          pnew = &q0;
          pold = &p1;
        }
        else {
          outcode = code1;
          pold = &p0;
          pnew = &q1;
        }

        if(outcode & FAR) {
          d = (zmax - p0(2)) / (p1(2) - p0(2));
          (*pnew)(0) = (p1(0) - p0(0)) * d + p0(0);
          (*pnew)(1) = (p1(1) - p0(1)) * d + p0(1);
          (*pnew)(2) = zmax;
        }
        else if(outcode & NEAR) {
          d = (zmin - p0(2)) / (p1(2) - p0(2));
          (*pnew)(0) = (p1(0) - p0(0)) * d + p0(0);
          (*pnew)(1) = (p1(1) - p0(1)) * d + p0(1);
          (*pnew)(2) = zmin;
        }
        else if(outcode & BOTTOM) {
          d = (ymax - p0(1)) / (p1(1) - p0(1));
          (*pnew)(0) = (p1(0) - p0(0)) * d + p0(0);
          (*pnew)(1) = ymax;
          (*pnew)(2) = (p1(2) - p0(2)) * d + p0(2);
        }
        else if(outcode & TOP) {
          d = (ymin - p0(1)) / (p1(1) - p0(1));
          (*pnew)(0) = (p1(0) - p0(0)) * d + p0(0);
          (*pnew)(1) = ymin;
          (*pnew)(2) = (p1(2) - p0(2)) * d + p0(2);
        }
        else if(outcode & RIGHT) {
          d = (xmax - p0(0)) / (p1(0) - p0(0));
          (*pnew)(0) = xmax;
          (*pnew)(1) = (p1(1) - p0(1)) * d + p0(1);
          (*pnew)(2) = (p1(2) - p0(2)) * d + p0(2);
        }
        else /*(outcode & LEFT)*/ {
          d = (xmin - p0(0)) / (p1(0) - p0(0));
          (*pnew)(0) = xmin;
          (*pnew)(1) = (p1(1) - p0(1)) * d + p0(1);
          (*pnew)(2) = (p1(2) - p0(2)) * d + p0(2);
        }

        accept = clip(*pold,*pnew,q0,q1);
      }

      return accept;
    }

    private:
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;

    static const int FAR;    // 10 00 00
    static const int NEAR;   // 01 00 00
    static const int TOP;    // 00 10 00
    static const int BOTTOM; // 00 01 00
    static const int RIGHT;  // 00 00 10
    static const int LEFT;   // 00 00 01
  };

  const int CohenSutherlandClipping::FAR    = 32;
  const int CohenSutherlandClipping::NEAR   = 16;
  const int CohenSutherlandClipping::TOP    =  8;
  const int CohenSutherlandClipping::BOTTOM =  4;
  const int CohenSutherlandClipping::RIGHT  =  2;
  const int CohenSutherlandClipping::LEFT   =  1;

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
      Eigen::Matrix4f proj = Eigen::Map<const Eigen::Matrix<float,4,4,Eigen::RowMajor> >( PrimeSense::mat ) * camera_transform;
      // transform polygons to clipping space and reject outliers ( space: x,y,z E [-1,1] )
      CohenSutherlandClipping csc;
      ScanlinePolygonFill spf(w,h);
      Eigen::Vector3f p0, p1, q0, q1;
      for(int i=0; i<polygons.size(); ++i)
      {
        int id = polygons[i]->id;
        for(int c=0; c<polygons[i]->contours.size(); ++c)
        {
          std::vector<Eigen::Vector3f>* ptr_c = &polygons[i]->contours[c];
          Eigen::Vector4f pprev = proj * Eigen::Vector4f( (*ptr_c)[ ptr_c->size()-1 ](0),
                                                          (*ptr_c)[ ptr_c->size()-1 ](1),
                                                          (*ptr_c)[ ptr_c->size()-1 ](2), 1);
          pprev /= pprev(3);
          bool was_clipped = false;
          Eigen::Vector3f last_clip;
          for(int p=0; p<ptr_c->size(); ++p)
          {
            Eigen::Vector4f pcurr = proj * Eigen::Vector4f( (*ptr_c)[p](0), (*ptr_c)[p](1), (*ptr_c)[p](2), 1);
            //if(pcurr(3) != 1.0) std::cout << pcurr(3) << std::endl;
            pcurr /= pcurr(3);
            p0 << pprev(0), pprev(1), pprev(2);
            p1 << pcurr(0), pcurr(1), pcurr(2);
            if(csc.clip(p0,p1,q0,q1)) // TODO: polygon clipping might be better here 
            { /*
              if(q0 != p0)
              {
                if(was_clipped)
                {
                  was_clipped = false;
                  spf.addEdge(id, q0(0), q0(1), last_clip(0), last_clip(1));
                }
                else
                {
                  was_clipped = true;
                  last_clip = q0;
                  spf.addEdge(id, q0(0), q0(1), q1(0), q1(1));
                }
              }
              else if(q1 != p1)
              {
                if(was_clipped)
                {
                  was_clipped = false;
                  spf.addEdge(id, last_clip(0), last_clip(1), q1(0), q1(1));
                }
                else
                {
                  was_clipped = true;
                  last_clip = q1;
                  spf.addEdge(id, q0(0), q0(1), q1(0), q1(1));
                }
              }
              else
              {
                spf.addEdge(id, q0(0), q0(1), q1(0), q1(1));
                }*/
              spf.addEdge(id, q0(0), q0(1), q1(0), q1(1));
            }
            pprev = pcurr;
          }
        }
      }
      // rasterize remaining polygons using scanline rendering algorithm
      //spf.addEdge(10, -1.0f, -1.0f, 0.0f, 1.0f);
      //spf.addEdge(10,  0.0f,  1.0f, 1.0f, -1.0f);
      spf.fill(projection);
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
    static const float w;
    static const float h;
    static const float aspect;
    static const float mat[]; // transformation from view frustum to unit cube
  };

  const float PrimeSense::horizontal = 49.0f / 180.0f * M_PI;
  const float PrimeSense::vertical = 43.0f / 180.0f * M_PI;
  const float PrimeSense::f = 5.0f;
  const float PrimeSense::n = 0.5f;
  const float PrimeSense::r = PrimeSense::n * tan(PrimeSense::horizontal * 0.5f);
  const float PrimeSense::l = - PrimeSense::r;
  const float PrimeSense::t = PrimeSense::n * tan(PrimeSense::vertical * 0.5f);
  const float PrimeSense::b = - PrimeSense::t;
  const float PrimeSense::w = 640.0f;
  const float PrimeSense::h = 480.0f;
  const float PrimeSense::aspect = PrimeSense::w / PrimeSense::h;

/*
  const float PrimeSense::mat[] = {
    2.0f*PrimeSense::n/(PrimeSense::r-PrimeSense::l), 0, - (PrimeSense::r+PrimeSense::l)/(PrimeSense::r-PrimeSense::l), 0,
    0, 2.0f*PrimeSense::n/(PrimeSense::t-PrimeSense::b), - (PrimeSense::t+PrimeSense::b)/(PrimeSense::t-PrimeSense::b), 0,
    0, 0, (PrimeSense::n+PrimeSense::f)/(PrimeSense::n-PrimeSense::f), -2.0f*PrimeSense::f*PrimeSense::n/(PrimeSense::n-PrimeSense::f),
    0, 0, 1, 0
    };
*/

  const float PrimeSense::mat[] = {
    1.0f / ( PrimeSense::aspect * tan(0.5f * PrimeSense::horizontal) ), 0, 0, 0,
    0, 1.0f / ( tan(0.5f * PrimeSense::horizontal) ), 0, 0,
    0, 0, (PrimeSense::n+PrimeSense::f)/(PrimeSense::n-PrimeSense::f), -2.0f*PrimeSense::f*PrimeSense::n/(PrimeSense::n-PrimeSense::f),
    0, 0, 1, 0
  };

}

#endif
