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
 * ROS package name: cob_3d_mapping_tools
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 01/2012
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


#include "cob_3d_mapping_features/most_discriminating_data_points.h"
#include <float.h>
#include <math.h>
#include <iostream>
#include <algorithm>

// TODO: change data type to Eigen::VectorXF or something

int
cob_3d_mapping_features::MostDiscriminatingDataPoints::eStep()
{
  int n_chg = 0; // number of changed data points
  int k_min;
  float d_min,d;
  for (size_t k=0;k<k_;k++) count_[k] = 0;
  //std::cout << "cleared count_" << std::endl;
  
  for (size_t n=0;n<n_;n++) // iterate data points
  {
    d_min = FLT_MAX;
    for (size_t k=0;k<k_;k++) // iterate mean clusters
    {
      // calc distance to each mean cluster and save k with min distance
      d = 0.0;
      //std::cout << "compute distance: " << k<<"/"<<k_<< std::endl;
      for (size_t m=0;m<m_;m++) d += pow( (pdata_->at(n)).at(m) - (pmeans_->at(k)).at(m), 2 );
      //for (size_t m=0;m<m_;m++) d += std::min( (pdata_->at(n)).at(m),(pmeans_->at(k)).at(m) );
      //std::cout << "distance: " << d << std::endl;
      if (d < d_min) {d_min = d; k_min = k;}
    }
    //std::cout << "n: " << n << " dmin: " << d_min << " kmin: " << k_min << std::endl;
    //std::cout << assigned_classes_.size() << std::endl;
    if (k_min != assigned_classes_[n]) n_chg++;
    assigned_classes_[n] = k_min;
    count_[k_min]++;
  }
  return n_chg;
}

void
cob_3d_mapping_features::MostDiscriminatingDataPoints::mStep()
{
  for (size_t k=0;k<k_;k++)
    for (size_t m=0;m<m_;m++)
      (pmeans_->at(k)).at(m) = 0.0;
  for (size_t n=0;n<n_;n++)
    for (size_t m=0;m<m_;m++)
      (pmeans_->at(assigned_classes_[n])).at(m) += (pdata_->at(n)).at(m);
  for (size_t k=0;k<k_;k++)
  {
    if (count_[k] > 0)
      for (size_t m=0;m<m_;m++) (pmeans_->at(k)).at(m) /= count_[k];
  }
}

void
cob_3d_mapping_features::MostDiscriminatingDataPoints::computeKmeans()
{
  // fill initial means:
  for (size_t k=0;k<k_;k++)
  {
    pmeans_->at(k) = pdata_->at(init_indices_->at(k));
  }
  //std::cout << "Start computing k-means..." << std::endl;
  int steps = 0;
  int changed_points = -1;
  while (changed_points != 0)
  {
    changed_points = eStep();
    //std::cout << "Re-assigned Points: " << changed_points << std::endl;
    mStep();
    steps++;
  }
  //std::cout<<std::endl<< "K-Means needed " << steps << " steps to find cluster centers!" << std::endl;
}

void
cob_3d_mapping_features::MostDiscriminatingDataPoints::computeInitialMeans(std::vector<int> * const output_init_indices)
{
  if (!pdata_)
  {
    std::cout << "[computeInitialMeans]: No input data points defined" << std::endl;
    return;
  }
  std::vector<float> mean(m_,0.0);
  for (size_t n=0;n<n_;n++) 
    for (size_t m=0;m<m_;m++) 
      mean.at(m) += (pdata_->at(n)).at(m);

  //std::cout << "Created mean data: | ";
  for (size_t m=0;m<m_;m++) 
  {
    mean.at(m) /= n_;
  }

  std::vector<SortableDataPoint> sorted_by_dist;
  cob_3d_mapping_features::SortableDataPoint p;
  for (size_t n=0;n<n_;n++)
  {
    p.idx = n;
    p.dist = 0.0;
    // histogram intersection kernel:
    for (size_t m=0;m<m_;m++) p.dist += std::min((pdata_->at(n)).at(m), mean.at(m));
    // manhatten distance:
    //for (size_t m=0;m<m_;m++) p.dist += pow((pdata_->at(n)).at(m) - mean.at(m), 2);
    sorted_by_dist.push_back(p);
  }
  sort(sorted_by_dist.begin(), sorted_by_dist.end());
  //std::cout << "Sorted input data by distance to mean data..." << std::endl;
  output_init_indices->clear();
  output_init_indices->resize(k_);


  size_t bin_size = floor((float)n_ / (float)k_);
  size_t n_idx = round(0.5f*(float)bin_size);
  //std::cout << "Select initial indices... " << std::endl;
  for (size_t k=0;k<k_;k++)
  {
    output_init_indices->at( k ) = (sorted_by_dist.at(n_idx)).idx;
    n_idx += bin_size;
  }
  //std::cout << std::endl;
  return;
}

void
cob_3d_mapping_features::MostDiscriminatingDataPoints::computeDataPoints(std::vector<std::vector<float> > * const k_means)
{
  if (!pdata_)
  {
    std::cout << "[computeDataPoints]: No input data points defined" << std::endl;
    return;
  }

  pmeans_ = k_means;
  pmeans_->clear();
  pmeans_->resize(k_);
  assigned_classes_.clear();
  assigned_classes_.resize(n_);
  count_.resize(k_);
  
  if (predefined_initial_centers_)
  {
    computeKmeans();
  }
  else
  {
    init_indices_ = new std::vector<int>(k_);
    computeInitialMeans(init_indices_);
    computeKmeans();
  }
}
