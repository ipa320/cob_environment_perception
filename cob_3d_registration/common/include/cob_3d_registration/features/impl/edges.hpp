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
 * ROS package name: registration
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-jh
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Nov 18, 2011
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


template<typename Point>
void Feature_Edges<Point>::extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<Point> &edges) {
  pcl::PointCloud<Normal>::Ptr n(new pcl::PointCloud<Normal>);
  pcl::PointCloud<InterestPoint>::Ptr ip(new pcl::PointCloud<InterestPoint>);

  ROS_INFO("normals...");

  pcl::IntegralImageNormalEstimation<Point, pcl::Normal> ne;

  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.2f);
  ne.setRectSize(10,10);
  ne.setNormalSmoothingSize(10.0f);
  ne.setDepthDependentSmoothing(true);
  ne.setInputCloud(point_cloud.makeShared());

  ne.compute(*n);

  ROS_INFO("edges...");
#ifdef PCL_DEPRECATED
  boost::shared_ptr<pcl::search::OrganizedNeighbor<Point> > oTree (new pcl::search::OrganizedNeighbor<Point> );
#else
  boost::shared_ptr<pcl::OrganizedDataIndex<Point> > oTree (new pcl::OrganizedDataIndex<Point> );
#endif
  cob_3d_mapping_features::FastEdgeEstimation3DOMP<Point, Normal, InterestPoint> ee;
  /*ee.setRadiusSearch(radius_);
  ee.dist_threshold_ = dist_threshold_;
  ee.setSearchMethod(oTree);*/
  ee.setInputCloud(point_cloud.makeShared());
  ee.setInputNormals(n);
  ee.compute(*ip);

  edges.points.clear();
  for (size_t i = 0; i < ip->points.size(); i++)
  {
    if(ip->points[i].strength>thr_ && ip->points[i].strength<=1.f) {
      edges.points.push_back(point_cloud.points[i]);
    }
  }

  edges.width=edges.size();
  edges.height=1;
}
