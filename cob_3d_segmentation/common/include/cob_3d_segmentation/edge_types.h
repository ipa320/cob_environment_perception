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

#ifndef __COB_3D_SEGMENTATION_EDGE_TYPES_H__
#define __COB_3D_SEGMENTATION_EDGE_TYPES_H__

#include <list>
#include <map>
#include <utility>

namespace cob_3d_segmentation
{
  class BoundaryPoint
  {
  public:
    BoundaryPoint(int idx=0) : brother(idx), normal() { }
    
    int brother;
    Eigen::Vector3f normal;
  };

  class BoundaryPointsEdge
  {
  public:
    BoundaryPointsEdge() : width(1)
      , boundary_pairs()
      , smoothness(0.0) { }

    inline std::pair<std::list<int>::iterator, std::list<int>::iterator> getBoundaryPairs()
    { return std::make_pair(boundary_pairs.begin()->second.begin(), boundary_pairs.begin()->second.end()); }

    inline std::pair<std::list<int>::iterator, std::list<int>::iterator> getBoundaryPairsOf(int cid)
    { return std::make_pair(boundary_pairs[cid].begin(), boundary_pairs[cid].end()); }

    inline std::list<int>::size_type size() const { return boundary_pairs.begin()->second.size(); }

    inline void addBoundaryIndices(const int cid1, const int cid2, const int idx1, const int idx2)
    {
      boundary_pairs[cid1].push_back(idx2);
      boundary_pairs[cid2].push_back(idx1);
    }

    inline bool isAttachedTo(int cid) { return (boundary_pairs.find(cid) != boundary_pairs.end()); }
    
    int width;
    std::map<int,std::list<int> > boundary_pairs;
    float smoothness;
  };
}

#endif
