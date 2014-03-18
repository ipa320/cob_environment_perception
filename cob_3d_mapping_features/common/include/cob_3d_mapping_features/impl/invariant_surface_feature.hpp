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
 * Author: Joshua Hampp, email:joshua.hampp@ipa.fraunhofer.de
 *
 * Date of creation: 03/2014
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

#pragma once

#include "cob_3d_mapping_features/invariant_surface_feature.h"

void cob_3d_mapping_features::InvariantSurfaceFeature<>::compute() {
  result_.reset(new ResultVector);

  //generate keypoints (e.g. reduce number of points by area)
  std::vector<TVector> keypoints;
  generateKeypoints(keypoints);

  result_->resize(keypoints.size());
  for(size_t i=0; i<keypoints.size(); i++) {
    (*result_)[i].v = keypoints[i];
    (*result_)[i].ft.resize(radii_.size());
    for(size_t j=0; j<radii_.size(); j++) {
      (*result_)[i].ft[j].resize(num_radius_*num_angle_*num_angle_,0);

      //generate sub map
      std::vector<VectorWithParams> pts;
      subsample(keypoints[i], radii_[j], pts);

      for(int inclination=0; inclination<num_angle_; inclination++) {
        Scalar _inclination = M_PI*inclination/num_angle_;
        for(int azimuth=0; azimuth<num_angle_; azimuth++) {
          Scalar _azimuth = M_PI*azimuth/num_angle_;
          for(int radius=0; radius<num_radius_; radius++) {
            Scalar _radius = radius*radii_[j]/num_radius_;

            /*
             * x=_radius*std::sin(_inclination)*std::cos(_azimuth);
             * y=_radius*std::sin(_inclination)*std::sin(_azimuth);
             * z=_radius*std::cos(_inclination);
             */
            kernel();
          }
        }
      }

      (*result_)[i].ft;
    }
  }
}

void cob_3d_mapping_features::InvariantSurfaceFeature<>::generateKeypoints(std::vector<TVector> &keypoints) {
  //TODO: reduce points

  for(size_t i=0; i<input_->size(); i++) {                         //surfaces
    for(size_t j=0; j<(*input_)[i].segments_.size(); j++) {        //outline/holes
      for(size_t k=0; k<(*input_)[i].segments_[j].size(); k++) {   //points
        TVector v;
        v(0) = (*input_)[i].segments_[j][k](0);
        v(1) = (*input_)[i].segments_[j][k](1);
        v(2) = (*input_)[i].model_.model( v(0), v(1) );
        keypoints.push_back(v);
      }
    }
  }

}

void cob_3d_mapping_features::InvariantSurfaceFeature<>::subsample(const TVector &at, std::vector<VectorWithParams> &pts) {

}
//#define PCL_INSTANTIATE_OrganizedCurvatureEstimationOMP(T,NT,LabelT,OutT) template class PCL_EXPORTS cob_3d_mapping_features::OrganizedCurvatureEstimationOMP<T,NT,LabelT,OutT>;

