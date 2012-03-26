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
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
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

#ifndef __COB_3D_MAPPING_COMMON_CLUSTER_H__
#define __COB_3D_MAPPING_COMMON_CLUSTER_H__

#include "cob_3d_mapping_common/label_defines.h"

namespace cob_3d_mapping_common
{
  class Cluster
  {
  public:
    Cluster () : indices(), orientation(), type(I_UNDEF)
    { };

    ~Cluster () { };

    void
    updateCluster(const int idx, const Eigen::Vector3f& new_normal)
    {
      indices.push_back(idx);
      orientation += new_normal / (indices.size());
      orientation = orientation.normalized();
    }

    Eigen::Vector3f orientation;
    std::vector<int> indices;
    int type;

  protected:

  };

  inline bool operator< (const Cluster& lhs, const Cluster& rhs){return lhs.indices.size() < rhs.indices.size();}
  inline bool operator> (const Cluster& lhs, const Cluster& rhs){return  operator< (rhs, lhs);}
  inline bool operator<=(const Cluster& lhs, const Cluster& rhs){return !operator> (lhs, rhs);}
  inline bool operator>=(const Cluster& lhs, const Cluster& rhs){return !operator< (lhs, rhs);}
}

#endif //__COB_3D_MAPPING_COMMON_CLUSTER_H__
