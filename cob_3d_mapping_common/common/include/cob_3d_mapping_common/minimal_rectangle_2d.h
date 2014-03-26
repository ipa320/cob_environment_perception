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
 * \date Date of creation: 04/2013
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

#ifndef MINIMAL_RECTANGLE_2D_H__
#define MINIMAL_RECTANGLE_2D_H__

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <iostream>

namespace cob_3d_mapping
{
  /**
   * \brief Class for computing a minimal rectangle in 2D.
   * \details Ref: Solving Geometric Problems with the Rotating Calipers [Toussaint 1983] http://geomalgorithms.com/a08-_containers.html
  */
  class MinimalRectangle2D
  {
    private:
    typedef Eigen::Vector2f PT;

    class Line
    {
      public:
      Line(int point, const PT& dir)
        : idx(point)
        , direction(dir)
        , stop(false)
      { }

      float getDot(const std::vector<PT>& edges)
      { return fabs(direction.dot(edges[idx])); }

      void rotate(const std::vector<PT>& edges)
      {
        direction = edges[idx];
        ++idx;
        if( idx >= edges.size() ) idx = 0;
        stop = true;
        next->makeOrthogonalTo(direction);
      }

      void makeOrthogonalTo(const PT& dir)
      {
        if (stop) { stop = false; return; }
        direction[0] = dir[1];
        direction[1] = -dir[0];
        next->makeOrthogonalTo(direction);
      }

      int idx;
      PT direction;
      bool stop;
      Line* next;
    };

    public:
    MinimalRectangle2D() { }

    /* set input, convex hull is required, will be copied to local lists of edges and vertices to
     * save some computation */
    template<typename T> void setConvexHull(const T& hull)
    {
      vertices.push_back(PT(hull[0].x, hull[0].y));
      PT tmp;
      for(int i=1; i<hull.size(); ++i)
      {
        tmp = PT(hull[i].x - hull[i-1].x, hull[i].y - hull[i-1].y);
        edges.push_back(tmp.normalized());
        vertices.push_back(PT(hull[i].x, hull[i].y));
      }
      tmp = PT(hull[0].x - hull[hull.size()-1].x, hull[0].y - hull[hull.size()-1].y);
      edges.push_back(tmp.normalized());
    }

    /* set input, convex hull is required, will be copied to local lists of edges and vertices to
     * save some computation */
    template<typename T> void setConvexHullList(const T& hull)
    {
      vertices.push_back(PT(hull[0][0], hull[0][1]));
      PT tmp;
      for(int i=1; i<hull.size(); ++i)
      {
        tmp = PT(hull[i][0] - hull[i-1][0], hull[i][1] - hull[i-1][1]);
        edges.push_back(tmp.normalized());
        vertices.push_back(PT(hull[i][0], hull[i][1]));
      }
      tmp = PT(hull[0][0] - hull[hull.size()-1][0], hull[0][1] - hull[hull.size()-1][1]);
      edges.push_back(tmp.normalized());
    }

    inline Line* minAngle(Line* lhs, Line* rhs)
    { return ( lhs->getDot(edges) < rhs->getDot(edges) ? rhs : lhs ); }

    inline Line* minAngle(Line* i, Line* j, Line* k, Line* l)
    { return minAngle( minAngle(i,j), minAngle(k,l) ); }

    inline void getIntersection(const Line& l1, const Line& l2, PT& point)
    { point = (vertices[l2.idx] - vertices[l1.idx]).dot(l1.direction) * l1.direction + vertices[l1.idx]; }

    // cross product: (p1-p2) x (p3-p2)
    inline float computeArea(const PT& p1, const PT& p2, const PT& p3)
    { return fabs( ((p1[0]-p2[0]) * (p3[1]-p2[1])) - ((p3[0]-p2[0]) * (p1[1]-p2[1])) ); }

    void rotatingCalipers(PT& origin, PT& ccw, PT& cw)
    {
      float xmin = std::numeric_limits<float>::max();
      float ymax = - std::numeric_limits<float>::max();
      float xmax = - std::numeric_limits<float>::max();
      float ymin = std::numeric_limits<float>::max();
      float amin = std::numeric_limits<float>::max();

      int i, j, k, l;
      for (int it=0; it<vertices.size(); ++it)
      {
        if     (vertices[it][0] < xmin) { xmin = vertices[it][0]; i = it; }
        else if(vertices[it][0] > xmax) { xmax = vertices[it][0]; k = it; }
        if     (vertices[it][1] < ymin) { ymin = vertices[it][1]; l = it; }
        else if(vertices[it][1] > ymax) { ymax = vertices[it][1]; j = it; }
      }

      // create inital rectangle parallel to xy-axis
      Line vi(i, PT(0,1)); // min x, parallel to y-axis
      Line vj(j, PT(1,0)); // max y, parallel to x-axis
      Line vk(k, PT(0,-1)); // max x, parallel to y-axis
      Line vl(l, PT(-1,0)); // min y, parallel to x-axis
      vi.next = &vj;
      vj.next = &vk;
      vk.next = &vl;
      vl.next = &vi;

      PT ref(0,1);
      // the rectangle keeps rotating clock-wise until vi is parallel to x-axis
      while(vi.direction.dot(ref) > 0)
      {
        Line* ls = minAngle(&vi, &vj, &vk, &vl); // get the new supporting line
        ls->rotate(edges); // rotate line and move one point forward
        /*
        std::cout << "i: " << vertices[vi.idx][0] <<"," << vertices[vi.idx][1] << std::endl;
        std::cout << "j: " << vertices[vj.idx][0] <<"," << vertices[vj.idx][1] << std::endl;
        std::cout << "k: " << vertices[vk.idx][0] <<"," << vertices[vk.idx][1] << std::endl;
        std::cout << "l: " << vertices[vl.idx][0] <<"," << vertices[vl.idx][1] << std::endl;
        */
        PT p1, p2, p3; // p1 -> p2 -> p3
        getIntersection(vl, vi, p1);
        getIntersection(vi, vj, p2);
        getIntersection(vj, vk, p3);
        float area = computeArea(p1, p2, p3);
        /*std::cout << "l-i: " << p1(0) <<","<<p1(1)<<std::endl;
        std::cout << "i-j: " << p2(0) <<","<<p2(1)<<std::endl;
        std::cout << "j-k: " << p3(0) <<","<<p3(1)<<std::endl;
        std::cout << area << std::endl;*/
        if (area < amin)
        {
          amin = area;
          origin = p2;
          ccw = p1;
          cw = p3;
        }
      }
    }

    private:
    std::vector<PT> edges;
    std::vector<PT> vertices;
  };
}

#endif // MINIMAL_RECTANGLE_2D_H__
