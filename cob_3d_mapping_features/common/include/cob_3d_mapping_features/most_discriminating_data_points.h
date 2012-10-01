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
 *  ROS package name: cob_3d_mapping_tools
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 01/2012
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


#ifndef __MOST_DISCRIMINATING_DATA_POINTS_H__
#define __MOST_DISCRIMINATING_DATA_POINTS_H__

#include <cstddef>
#include <vector>

namespace cob_3d_mapping_features
{

  struct SortableDataPoint
  {
  public:
  SortableDataPoint() : idx(-1), dist(0.0) { }
  
  SortableDataPoint(const int &index, const float &mean_distance) : idx(index), dist(mean_distance) 
      { }
  
    inline bool
    operator<( const SortableDataPoint& other) const { return (dist < other.dist); }

    inline bool
    operator>( const SortableDataPoint& other) const { return (dist > other.dist); }

    inline bool
    operator<=( const SortableDataPoint& other) const { return (dist <= other.dist); }

    inline bool
    operator>=( const SortableDataPoint& other) const { return (dist >= other.dist); }


    int idx;
    float dist;
  
  };

  class MostDiscriminatingDataPoints
  {
    public:
    MostDiscriminatingDataPoints () : predefined_initial_centers_(false), k_(1) { }


    inline void
      setInputData(const std::vector<std::vector<float> > * const pdata)
    {
      pdata_ = pdata;
      m_ = (pdata->at(0)).size();
      n_ = pdata->size();
    }

    inline void
      setK(const int k)
    {
      k_ = k;
    }

    inline void
      setInitialMeans(std::vector<int> * const pindices)
    {
      init_indices_ = pindices;
      predefined_initial_centers_ = true;
      k_ = pindices->size();
    }

    inline void
      resetInitialMeans()
    {
      predefined_initial_centers_ = false;
    }

    void
      computeInitialMeans(std::vector<int> * const output_init_indices);

    void
      computeDataPoints(std::vector<std::vector<float> > * const k_means);

    protected:
    void
      computeKmeans();

    int
      eStep();
   
    void
      mStep();

    bool predefined_initial_centers_;
    size_t k_;
    size_t m_; // size of a data point
    size_t n_; // number of data points
    const std::vector<std::vector<float> > *pdata_; // n x m
    std::vector<std::vector<float> > *pmeans_; // k x m

    std::vector<int> *init_indices_;
    std::vector<int> assigned_classes_;
    std::vector<int> count_;
   
  };
}

#endif // __MOST_DISCRIMINATING_DATA_POINTS_H__
