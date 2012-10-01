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
 * ROS stack name: cob_vision
 * ROS package name: dynamic_tutorials
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Joshua Hampp
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Oct 26, 2011
 * ToDo:
 *
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


#include <pcl/common/transformation_from_correspondences.h>


template <typename Point>
void TransformationEstimationMultipleCorrespondences<Point>::computeTransformation
(PointCloudSource &output)
{
  final_transformation_ = findTF_fast(*target_, output, rmax_, tmax_);
  converged_ = true;
}

template <typename Point>
Eigen::Matrix4f TransformationEstimationMultipleCorrespondences<Point>::compute(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new)
{
  return findTF_fast(pc_old, pc_new, rmax_, tmax_);
}

template <typename Point>
Eigen::Matrix4f TransformationEstimationMultipleCorrespondences<Point>::findTF_fast
(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
 const float rmax, const float tmax, Eigen::Matrix4f tf)
 {
  std::vector<SORT_S2> tv;
  std::vector<COR_S> cors;

  for(size_t i=0; i<pc_new.size(); i++) {
    SORT_S2 s;
    s.dis=pc_new.points[i].getVector3fMap().norm();
    s.ind=(int)i;
    tv.push_back(s);
  }

  SORT_S2 _s_;
  std::sort(tv.begin(),tv.end(), _s_);

  std::vector<int> weight_o, weight_n;

  weight_o.resize(pc_old.size(), 0);
  weight_n.resize(pc_new.size(), 0);

  float rmax_ = sinf(rmax);
  rmax_*=rmax_;

  weight_o.resize(pc_old.size(), 0);
  weight_n.resize(pc_new.size(), 0);

  //get all possible correspondences
  COR_S c;
  for(size_t i=0; i<pc_old.size(); i++) {
    float t = pc_old.points[i].getVector3fMap().norm();
    Eigen::Vector3f vo = pc_old.points[i].getVector3fMap();

    //int num=0;
    for(int j=search_sorted_vector(tv,t-tmax); j<tv.size(); j++) {

      float dT = std::abs(tv[j].dis-t);
      if( dT < tmax) {
        Eigen::Vector3f vn = pc_new.points[tv[j].ind].getVector3fMap();

        float dA = (((vn-vo).squaredNorm()/vo.squaredNorm()));

        if(dA>=rmax_) continue;

        /*if(num>0 &&
            (vn-vo)
            .dot
            (pc_new[cors.back().ind_n].getVector3fMap()-vo)<0. )
        {
          cors.resize(cors.size()-num);
          break;
        }*/

        //++num;

        c.ind_o = (int)i;
        c.ind_n = tv[j].ind;
        cors.push_back(c);

        weight_o[i]++;
        weight_n[c.ind_n]++;
      }
      else if(tv[j].dis-t > tmax)
        break;
    }

  }

  //weight correspondences
  pcl::TransformationFromCorrespondences transFromCorr;
  for(size_t i=0; i<cors.size(); i++) {
    transFromCorr.add(pc_old[cors[i].ind_o].getVector3fMap(), pc_new[cors[i].ind_n].getVector3fMap(), 1./std::min(weight_o[cors[i].ind_o],weight_n[cors[i].ind_n]));
  }
  Eigen::Matrix4f tf2 = transFromCorr.getTransformation().matrix();

  //break condition
  if(tmax<0.01)
    return tf*tf2;

  pcl::PointCloud<Point> tpc;
  pcl::transformPointCloud(pc_old,tpc,tf2);

  return findTF_fast(tpc,pc_new, rmax*0.5, tmax*0.5, tf*tf2);
 }


template <typename Point>
int TransformationEstimationMultipleCorrespondences<Point>::search_sorted_vector(const std::vector<SORT_S2> &tv, const float val) {
  int i=tv.size()-1;
  int step=tv.size()/2-1;
  while(i&&step&&i<(int)tv.size()) {
    if(tv[i].dis>=val) {
      i-=step;
    }
    else if(tv[i].dis<val)
      i+=step;
    else
      break;
    step/=2;
  }
  return i;
}
