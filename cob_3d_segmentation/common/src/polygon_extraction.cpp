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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
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

#ifndef __IMPL_POLYGON_EXTRACTION__
#define __IMPL_POLYGON_EXTRACTION__

#include <cstring>
#include <iostream>
#include <cob_3d_segmentation/polygon_extraction/polygon_extraction.h>
#include <cob_3d_segmentation/polygon_extraction/impl/poly2d.hpp>
#include <cob_3d_segmentation/polygon_extraction/polygon_types.h>

#include <cstdio>

cob_3d_segmentation::PolygonExtraction::PolygonExtraction()
 : ch_(NULL),ch_size_(0)
 , outline_check_(NULL)
 , outline_check_size_(0)
{
  Contour2D::generateSpline2D();
}


template <typename TPoint>
int cob_3d_segmentation::PolygonExtraction::getPos(int *ch, const int xx, const int yy, const int w, const int h) {
  int p = 0;
  //const int i = 0;
  for(int x = -1; x <= 1; x++) {
    for(int y = -1; y <= 1; y++) {
      if( xx + x >= 0 && yy + y >= 0 && xx + x < w && yy + y < h && (x || y) && ch[xx + x + (yy + y ) * w] > 0)
      {
        p |= (1 << Contour2D::SplineMap[(y + 1) * 3 + x + 1]);
      }
    }
  }
  return p;
}

template<typename TPoint, typename TPolygon>
void cob_3d_segmentation::PolygonExtraction::outline(const int w, const int h, std::vector<TPoint> out, TPolygon &poly)
{
  //sort border points ascendingly by y and x
  std::sort(out.begin(), out.end());
  //image size has changed, re-initialize
  if(ch_size_ < w*h)
  {
    delete [] ch_;
    ch_ = new int[w*h];
    ch_size_=w*h;
    memset(ch_,0,sizeof(int)*ch_size_);
  }

  //fill ch_ with border points
  for(size_t j=0; j<out.size(); j++) {
    ch_[ out[j].x + out[j].y * w ] = 1;//(int)j+1;
  }

  /*if(outline_check_size_<out.size()) {
    delete [] outline_check_;
    outline_check_ = new bool[out.size()];
    outline_check_size_=out.size();
  }
  memset(outline_check_,false,out.size());*/

  //ppm("/tmp/outline.ppm",w,h,ch_);
  //int n=-1;
  for(int n = 0; n < (int)out.size(); n++)
  //while(n+1<(int)out.size())
  {
    //if(poly.polys_.size() > 0) break; //why?
    //++n;
    /*if(outline_check_[n])
      continue;*/

    int x = out[n].x;
    int y = out[n].y;
    if(ch_[ x + y * w ] < 1) continue;
    int bf = 8; //neighborhood before (8 bit)
    int v = 0;
    int start_x = x, start_y = y;

    poly.addPolygon();
    poly.addPoint(x,y);
    //int num=0;
    std::stack<std::pair<int,Contour2D::spline2D> > forked_states;
    std::stack<typename std::vector<TPoint>::size_type> forked_points;
    while(1)
    {
      //out of image range or point is no border point
      if(x < 0 || y < 0 || x >= w || y >= h || ch_[ x + y * w ] < 1)
      {
        /*std::cout << "no points left" << std::endl;*/break;
      }

      int ch_old = ch_[ x+y*w ];
      //outline_check_[ch_[ x+y*w ]-1]=true;
      ch_[ x+y*w ] = 0;//-2;

      int p = getPos<TPoint>(ch_, x, y, w, h);

      if(p == 0 || (!Contour2D::g_Splines[bf][p].x && !Contour2D::g_Splines[bf][p].y))
      {
        // There is no valid next point:
        if (forked_states.size() == 0 || (std::abs(x-start_x)+std::abs(y-start_y)) <= 4) { /*std::cout << "no forks left" << std::endl;*/break; }
        //std::cout << "--> Back to fork!" << std::endl;
        // Go back to last forked state
        v = forked_states.top().second.v;
        x = forked_states.top().second.x;
        y = forked_states.top().second.y;
        //std::cout << "fork: " << x << "," << y << std::endl;
        bf = forked_states.top().second.bf;
        ch_ [ x+y*w ] = forked_states.top().first;
        forked_states.pop();
        /*std::cout << "poly points before: ";
        for(unsigned int i=0; i<poly.polys_.back().size(); i++)
          std::cout << poly.polys_.back()[i].x << "," << poly.polys_.back()[i].y << "->";
        std::cout << std::endl;
        std::cout << "to remove: " << forked_points.top() << std::endl;*/
        poly.removeLastPoints(forked_points.top());
        /*std::cout << "remaining poly points: ";
        for(unsigned int i=0; i<poly.polys_.back().size(); i++)
          std::cout << poly.polys_.back()[i].x << "," << poly.polys_.back()[i].y << "->";
        std::cout << std::endl;*/
        forked_points.pop();
        continue;
      }
      //create new fork
      if (hasMultiplePositions((unsigned int)p))
      {
        Contour2D::spline2D s = {v, x, y, bf};
        forked_states.push(std::pair<int,Contour2D::spline2D>(ch_old, s));
        forked_points.push(0);
      }

      v += v + Contour2D::g_Splines[bf][p].v;
      x += Contour2D::g_Splines[bf][p].x;
      y += Contour2D::g_Splines[bf][p].y;
      bf = Contour2D::g_Splines[bf][p].bf;
      //++num;

      if(std::abs(v) > 5)
      {
        v = 0;
        ++(forked_points.top());
        poly.addPoint(x,y);
        /*std::cout << "adding poly point: ";
        for(unsigned int i=0; i<poly.polys_.back().size(); i++)
          std::cout << poly.polys_.back()[i].x << "," << poly.polys_.back()[i].y << "->";
        std::cout << std::endl;*/
      }
    }

    //std::cout << "abs: " << x << "," << start_x << ";" << y << "," << start_y << std::endl;
    if(poly.polys_.back().size() < 4 || (std::abs(x - start_x) + std::abs(y - start_y)) > 4 )
    {
      poly.removePolygon();
      //std::cout << "No contour found, aborting" << std::endl;
      break;
    }

  }

  for(size_t j = 0; j < out.size(); j++) {
    ch_[ out[j].x + out[j].y * w ] = 0;
  }

}

void cob_3d_segmentation::PolygonExtraction::ppm(const char *fn, const int w, const int h, const int *ch) {
  //ppm
  FILE *fp = fopen(fn,"w");

  char buffer[128];
  sprintf(buffer,"P3\n%d %d\n255\n",h,w);
  fputs(buffer,fp);

  for(int x=0; x<w; x++) {
    for(int y=0; y<h; y++) {
      int v=0;
      if(ch[(x+w*y)])
        v=255;

      if(ch[(x+w*y)]<-1) {
        sprintf(buffer,"%d %d %d  ",0,255,0);
      }
      else if(ch[(x+w*y)]<0) {
        sprintf(buffer,"%d %d %d  ",255,0,0);
      }
      else
        sprintf(buffer,"%d %d %d  ",v,v,v);
      fputs(buffer,fp);
    }
  }
  fclose(fp);
}

// explicit instantiation of template member functions:

template void cob_3d_segmentation::PolygonExtraction::outline<cob_3d_segmentation::PolygonPoint, cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint> >(const int w, const int h, std::vector<cob_3d_segmentation::PolygonPoint> out, cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint> &poly);

template int cob_3d_segmentation::PolygonExtraction::getPos<cob_3d_segmentation::PolygonPoint>(int *ch, const int xx, const int yy, const int w, const int h);

#endif
