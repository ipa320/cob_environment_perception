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
/*
 * list.h
 *
 *  Created on: 11.06.2012
 *      Author: josh
 */

#ifndef LIST_H_
#define LIST_H_




/**
 * contains x,y coordinate alias index (v)
 * hops gives number of possible uses (rest)
 */
template<typename Point>
struct SVALUE
{
  unsigned int v,hops;
  Point pt;

  SVALUE(){}

  SVALUE(const unsigned int v, const unsigned int hops, const Point &pt)
  :v(v), hops(hops), pt(pt)
  {}

};

/**
 * fast visited list (pre-allocate values)
 */
template<typename VALUE>
struct VISITED_LIST {
  int size;
  int pos;
  std::vector<VALUE> vals;
  std::set<int> lookup;

  VISITED_LIST():size(0),pos(-1), vals(1000) {
  }

  inline void init()
  {
    size=(0);
    pos=(-1);
  }

  inline void add(const VALUE &v) {
    if(lookup.find(v.v)!=lookup.end()) {
      for(int i=0; i<size; i++)	//prevent double entries
        if(vals[i].v==v.v) {
          vals[i].hops = std::max(vals[i].hops, v.hops);
          return;
        }
    }
    if((size_t)size>=vals.size())
      vals.push_back(v);
    else
      vals[size]=v;
    ++size;
    lookup.insert(v.v);
  }

  inline void move() {++pos;}

  inline void remove() {
    --size;
    lookup.erase(vals[pos].v);
    if(pos<size) vals[pos]=vals[size];
    --pos;
  }

  inline void replace(const VALUE &v) {
    lookup.erase(vals[pos].v);
    lookup.insert(v.v);
    vals[pos]=v;
  }
};


/**
 * used as parameter for outline calculation
 * point with additional information if point was in front of other object
 */
template<typename Point>
struct SXY {
  int x,y;
  bool back;
  Point pt;
};

/**
 * sort 2D points from left to right, top to down
 */
template<typename Point>
struct SXYcmp {
  inline bool operator() (const SXY<Point>& lhs, const SXY<Point>& rhs) const
  {if(lhs.y==rhs.y)
    return lhs.x<rhs.x;
  return lhs.y<rhs.y;}
};

#endif /* LIST_H_ */
