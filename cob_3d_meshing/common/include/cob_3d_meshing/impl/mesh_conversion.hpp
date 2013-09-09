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
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_meshing
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

#ifndef COB_MESH_CONVERSION_HPP
#define COB_MESH_CONVERSION_HPP

namespace cob_3d_meshing
{
  template<typename SensorT>
  template<typename PointT, typename MeshT>
  void MeshConversion<SensorT>::fromPointCloud(
    const typename pcl::PointCloud<PointT>::ConstPtr& pc, MeshT& mesh)
  {
    typedef typename MeshT::VertexHandle VertexHandle;

    int rows = pc->height - 1; // last row
    int cols = pc->width - 1; // last column
    int row_offset;
    // [h]orizontal, [v]ertical, [l]eft, [r]ight edge check
    std::vector<std::vector<bool> > h(rows+1, std::vector<bool>(cols,true));
    std::vector<std::vector<bool> > v(rows, std::vector<bool>(cols+1,true));
    std::vector<std::vector<bool> > l(rows, std::vector<bool>(cols,true));
    std::vector<std::vector<bool> > r(rows, std::vector<bool>(cols,true));
    std::vector<std::vector<VertexHandle> >
      vh(rows+1, std::vector<VertexHandle>(cols+1)); // vertex handles

    /*
     * +--+--+   p00  h00  p01  h01  p02
     * |  |  |   v00 lr00  v01 lr01  v02
     * +--+--+   p10  h10  p11  h11  p12
     * |  |  |   v10 lr10  v11 lr11  v12
     * +--+--+   p20  h20  p21  h21  p22
     */

    // corners
    h.front().front() = v.front().front() = r.front().front() = false;
    h.front().back()  = v.front().back()  = l.front().back()  = false;
    h.back().front() = v.back().front() = l.back().front() = false;
    h.back().back()  = v.back().back()  = r.back().back()  = false;

    // first and last row
    for(int x = 1; x<cols; ++x)
    {
      h.front()[x-1] = false;
      h.front()[x  ] = false;
      v.front()[x  ] = false;
      l.front()[x-1] = false;
      r.front()[x  ] = false;
      h.back ()[x-1] = false;
      h.back ()[x  ] = false;
      v.back ()[x  ] = false;
      r.back ()[x-1] = false;
      l.back ()[x  ] = false;
    }

    for(int y = 1; y<rows; ++y)
    {
      // left column and right column
      h[y  ].front() = false;
      v[y-1].front() = false;
      v[y  ].front() = false;
      l[y-1].front() = false;
      r[y  ].front() = false;
      h[y  ].back()  = false;
      v[y-1].back()  = false;
      v[y  ].back()  = false;
      r[y-1].back()  = false;
      l[y  ].back()  = false;

      row_offset = y*(cols+1);
      // iterate remaining
      for(int x=1; x<cols; ++x)
      {
        const PointT* p = &(*pc)[row_offset+x];
        if( p->z != p->z )
        {
          v[y-1][x  ] = false;
          v[y  ][x  ] = false;
          h[y  ][x-1] = false;
          h[y  ][x  ] = false;
          l[y-1][x  ] = false;
          l[y  ][x-1] = false;
          r[y-1][x-1] = false;
          r[y  ][x  ] = false;
        }
        else
        {
          vh[y][x] = mesh.addVertex(y*pc->width+x, *p);
        }
      }
    }

    // iterate h and v to check if edge is valid
    typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
      ::const_iterator pii = pc->points.begin();
    typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
      ::const_iterator pij = pii + 1; // right
    typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
      ::const_iterator pji = pii + 1 + cols; // below
    typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
      ::const_iterator pjj = pji + 1; // below right

    for(int y=0; y<rows; ++y)
    {
      for(int x=0; x<cols; ++x)
      {
        // check horizontal and vertical
        if (h[y][x])
          h[y][x] = isNeighbor(pii->getVector3fMap(), pij->getVector3fMap());
        if (v[y][x])
          v[y][x] = isNeighbor(pii->getVector3fMap(), pji->getVector3fMap());

        // check diagonal
        unsigned char status = (l[y][x] << 1) | r[y][x];
        switch(status)
        {
        case 0b00:
          break;
        case 0b01:
          r[y][x] = isNeighbor(pii->getVector3fMap(), pjj->getVector3fMap());
          break;
        case 0b10:
          l[y][x] = isNeighbor(pij->getVector3fMap(), pji->getVector3fMap());
          break;
        case 0b11:
          if( (pij->z - pji->z) > (pii->z - pjj->z) )
          {
            r[y][x] = false;
            l[y][x] = isNeighbor(pij->getVector3fMap(), pji->getVector3fMap());
          }
          else
          {
            l[y][x] = false;
            r[y][x] = isNeighbor(pii->getVector3fMap(), pjj->getVector3fMap());
          }
          break;
        }
        ++pii; ++pij; ++pji; ++pjj;
      }
      // skip the last column
      // note that in the very last iteration, pjj points beyond end()
      ++pii; ++pij; ++pji; ++pjj;
    }

    for(int y=0; y<rows; ++y)
    {
      for(int x=0; x<cols; ++x)
      {
        /* ii-ji-ij | ij-ji-jj | ii-jj-ij | ii-ji-jj
         *  +-+     |    +     |  +-+     |  +
         *  |/      |   /|     |   \|     |  |\
         *  +       |  +-+     |    +     |  +-+     */
        if(l[y][x])
        {
          if (h[y  ][x] && v[y][x  ])
            mesh.addFace(vh[y][x  ], vh[y+1][x], vh[y  ][x+1]);
          if (h[y+1][x] && v[y][x+1])
            mesh.addFace(vh[y][x+1], vh[y+1][x], vh[y+1][x+1]);
        }
        else if (r[y][x])
        {
          if (h[y][x] && v[y][x+1])
            mesh.addFace(vh[y][x], vh[y+1][x+1], vh[y][x+1]);
          if (v[y][x] && h[y+1][x])
            mesh.addFace(vh[y][x], vh[y+1][x], vh[y+1][x+1]);
        }
      }
    }
  }
}

#endif
