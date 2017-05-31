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

#ifndef SCANLINE_POLYGON_FILL_HPP
#define SCANLINE_POLYGON_FILL_HPP

#include <set>
#include <list>
#include <algorithm>

namespace cob_3d_mapping
{
  template<typename T>
  void ScanlinePolygonFill<T>::draw(std::vector<std::vector<T> >& out)
  {
    out.resize(w*h);
    yque.sort();

    float xstep = (xmax - xmin) / float(w);
    float ystep = (ymax - ymin) / float(h);
    for(float y=ymin + 0.5f * ystep; y<ymax; y+=ystep)
    {
      //std::cout << "Line: "<< y2h(y) <<" y="<< y << std::endl;
      int yidx = y2h(y)*w;
      std::list<std::pair<float,T> > xque;
      typename std::list<ScanlineEdge<T> >::iterator ycurr = yque.begin();
      while (ycurr != yque.end() && ycurr->ymin < y)
      {
        if(ycurr->ymax <= y) {
          ycurr = yque.erase(ycurr);
        }
        else {
          float x = ycurr->intersection(y);
          if(x>xmin && x<xmax) {
            out[ yidx + x2w(x) ].push_back(ycurr->id);
          }
          ++ycurr;
        }
      }
    }
  }

  template<typename T>
  void ScanlinePolygonFill<T>::fill(std::vector<std::vector<T> >& out)
  {
    out.resize(w*h);
    yque.sort();

    float xstep = (xmax - xmin) / float(w);
    float ystep = (ymax - ymin) / float(h);
    // process each row
    for(float y=ymin + 0.5f * ystep; y<ymax; y+=ystep)
    {
      int yidx = y2h(y)*w;
      std::list<std::pair<float,T> > xque;
      typename std::list<ScanlineEdge<T> >::iterator ycurr = yque.begin();

      // compute intersections of all edges with the current row
      while (ycurr != yque.end() && ycurr->ymin < y)
      {
        if(ycurr->ymax <= y) {
          ycurr = yque.erase(ycurr);
        }
        else {
          xque.push_back(std::pair<float,T>(ycurr->intersection(y),ycurr->id));
          ++ycurr;
        }
      }

      xque.sort(); // create consecutive intersection pairs
      typename std::list<std::pair<float,T> >::iterator xcurr = xque.begin();
      typename std::list<std::pair<float,T> >::iterator xprev = xcurr++;
      std::set<T> curr_ids;

      // run ahead until xcurr is in picture, ( x-clipping )
      while(xcurr->first < xmin && xcurr!=xque.end()) {
        std::pair<typename std::set<T>::iterator, bool> res
          = curr_ids.insert(xprev->second);
        if( !res.second ) curr_ids.erase(res.first);
        xprev = xcurr++;
      }

      // run while xprev is in picture
      // process each consecutive intersection pair
      while (xprev->first < xmax && xcurr!=xque.end())
      {
        // add on first encounter, remove on second
        std::pair<typename std::set<T>::iterator, bool> res
          = curr_ids.insert(xprev->second);
        if( !res.second ) curr_ids.erase(res.first);
        if( curr_ids.size() )
        {
          float xstart = (xprev->first<xmin ? xmin : xprev->first);
          float xstop  = (xcurr->first>xmax ? xmax : xcurr->first);
          for(float x=xstart; x<=xstop; x+=xstep) // fill row
          {
            std::list<std::pair<float,T> > zque;

            // find min z value at curr x,y ( + z-clipping )
            typename std::set<T>::iterator id = curr_ids.begin();
            for(; id != curr_ids.end(); ++id) {
              float z = polys.find(*id)->second.intersection(x,y);
              if(z > zmin && z < zmax) zque.push_back(std::pair<float,T>(z, *id));
            }

            zque.sort();
            typename std::list<std::pair<float,T> >::iterator zcurr = zque.begin();

            // add ids withing a threshold of min z
            while(zcurr != zque.end()) {
              if(zcurr->first < (zque.begin()->first + zthr)) {
                out[ yidx + x2w(x) ].push_back(zcurr->second);
              }
              else {
                break; // done
              }
              ++zcurr;
            }
          }
        }
        xprev = xcurr++;
      } // done with current intersection pair
    } // done with current row
  }
}

#endif
