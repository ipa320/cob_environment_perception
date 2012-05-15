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

#ifndef __IMPL_CLUSTER_CLASSIFIER_HPP__
#define __IMPL_CLUSTER_CLASSIFIER_HPP__

#include "cob_3d_mapping_features/cluster_classifier.h"

template <typename ClusterHandlerT, typename PointT, typename NormalT, typename LabelT> void
cob_3d_mapping_features::ClusterClassifier<ClusterHandlerT,PointT,NormalT,LabelT>::classify()
{
  ClusterPtr c_it, c_end;
  for ( boost::tie(c_it,c_end) = clusters_->getClusters(); c_it != c_end; ++c_it)
  {
    clusters_->computeClusterComponents(c_it);

    if (!c_it->is_save_plane)
    {
      clusters_->recomputeClusterNormals(c_it);
      clusters_->computeCurvature(c_it);
      if (c_it->max_curvature < 0.02)
	c_it->type = I_PLANE;
      else if (c_it->max_curvature < 9.0 * c_it->min_curvature)
	c_it->type = I_SPHERE;
      else
	c_it->type = I_CYL;
    }
    else c_it->type = I_PLANE;
  }
}

#endif
