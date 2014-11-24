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
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de, Richard Bormann
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2011
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

#ifndef __ORGANIZED_NORMAL_ESTIMATION_EDGE_OMP_H__
#define __ORGANIZED_NORMAL_ESTIMATION_EDGE_OMP_H__

#include "cob_3d_features/organized_normal_estimation_edge.h"

namespace cob_3d_features
{
template<typename PointInT, typename PointOutT, typename LabelOutT>
class OrganizedNormalEstimationEdgeOMP: public OrganizedNormalEstimationEdge<PointInT, PointOutT, LabelOutT>
{
public:

	using OrganizedFeatures<PointInT, PointOutT>::pixel_search_radius_;
	using OrganizedFeatures<PointInT, PointOutT>::input_;
	using OrganizedFeatures<PointInT, PointOutT>::indices_;
	using OrganizedFeatures<PointInT, PointOutT>::surface_;
	using OrganizedFeatures<PointInT, PointOutT>::feature_name_;
	using OrganizedNormalEstimation<PointInT, PointOutT, LabelOutT>::labels_;

	typedef typename OrganizedNormalEstimation<PointInT, PointOutT, LabelOutT>::PointCloudOut PointCloudOut;

	OrganizedNormalEstimationEdgeOMP()
	{
		feature_name_ = "OrganizedNormalEstimationEdgeOMP";
	}
	;

protected:

	void
	computeFeature(PointCloudOut &output);

};
}

#endif
