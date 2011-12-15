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
 * Date of creation: 12/2011
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

#ifndef __ORGANIZED_CURVATURE_ESTIMATION_OMP_H__
#define __ORGANIZED_CURVATURE_ESTIMATION_OMP_H__

#include "cob_3d_mapping_features/organized_curvature_estimation.h"

namespace cob_3d_mapping_features
{
  template <typename PointInT, typename PointNT, typename PointLabelT, typename PointOutT>
    class OrganizedCurvatureEstimationOMP : public OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>
  {
    public:

    using OrganizedFeatures<PointInT,PointOutT>::input_;
    using OrganizedFeatures<PointInT,PointOutT>::surface_;
    using OrganizedFeatures<PointInT,PointOutT>::indices_;
    using OrganizedFeatures<PointInT,PointOutT>::feature_name_;
    using OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>::label_list_;
    using OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>::labels_;
    using OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>::normals_;

    typedef typename OrganizedCurvatureEstimation<PointInT,PointNT,PointLabelT,PointOutT>::PointCloudOut PointCloudOut;

    OrganizedCurvatureEstimationOMP ()
    {
      feature_name_ = "OrganizedCurvatureEstimationOMP";
      label_list_[0] = 1;
      label_list_[1] = 2;
      label_list_[2] = 3;
    };

    protected:

    void
      computeFeature (PointCloudOut &output);
    
  };
}

#endif
