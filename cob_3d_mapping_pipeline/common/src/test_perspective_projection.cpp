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
 *  ROS package name: cob_3d_mapping_pipeline
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2012
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

#include <pcl/io/pcd_io.h>

#include <cob_3d_mapping_common/point_types.h>

#include <cob_3d_mapping_features/organized_normal_estimation_omp.h>
#include <cob_3d_segmentation/impl/fast_segmentation.hpp>
#include <cob_3d_segmentation/polygon_extraction/polygon_extraction.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<PointLabel> LabelCloud;
typedef cob_3d_segmentation::FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel>::ClusterPtr ClusterPtr;

int main (int argc, char** argv)
{
  PointCloud::Ptr p(new PointCloud);
  NormalCloud::Ptr n(new NormalCloud);
  LabelCloud::Ptr l(new LabelCloud);

  pcl::PCDReader r;
  r.read(argv[1], p);

  cob_3d_mapping_features::OrganizedNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> one;
  one.setOuputLabels(l);
  one.setPixelSearchRadius(8,2,2);
  one.setSkipDistantPointThreshold(8);
  one.setInputCloud(p);
  one.compute(*n);

  cob_3d_segmentation::FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel> seg;
  seg.setNormalCloudIn(n);
  seg.setLabelCloudInOut(l);
  seg.setSeedMethod(cob_3d_segmentation::SEED_RANDOM);
  seg.setInputCloud(p);
  seg.compute();

  PolygonExtraction pe;
  std::vector<cob_3d_mapping::Polygon::Ptr> polygons;

  for(ClusterPtr c = seg.clusters()->begin(); c != seg.clusters()->end(); ++c)
  {
    if(c->size() < 100) continue;
    Eigen::Vector3f centroid = c->getCentroid();
    if(centroid(2) > 5.0f) continue;
    if(c->size() <= ceil(1.1f * (float)c->border_points.size())) continue;

    seg.clusters()->computeClusterComponents(c);
    if(!c->is_save_plane) continue;

    PolygonContours<PolygonPoint> poly;
    pe.outline(p->width,p->height,c->border_points,poly);
    if(!poly.polys_.size()) continue;
    int max_idx=0, max_size=0;
    for (int i=0; i<(int)poly.polys_.size(); ++i)
    {
      if( (int)poly.polys_[i].size() > max_size) {
        max_idx = i;
        max_size = poly.polys_[i].size():
      }
    }

    cob_3d_mapping::Polygon::Ptr p(new cob_3d_mapping::Polygon);
    p->id = c->id();
    for(int i=0; i<(int)poly.polys_.size(); ++i)
    {
      if (i==max_idx)
      {
        p->holes.push_back(false);
        p->contours.push_back(std::vector<Eigen::Vector3f>);
        for(std::vector<PolygonPoint>::iterator it = poly.polys_[i].begin(); it!=poly.polys_[i].end(); ++it)
        {
          p->contours.back().push_back( (*p)[it->x + it->y * p->width].getVector3fMap() );
        }
      }
      else
      {
        p->holes.push_back(true);
        p->contours.push_back(std::vector<Eigen::Vector3f>);
        for(std::vector<PolygonPoint>::reverse_iterator it = poly.polys_[i].rbegin(); it!=poly.polys_[i].rend(); ++it)
        {
          p->contours.back().push_back( (*p)[it->x + it->y * p->width].getVector3fMap() );
        }
      }
    }
    p->centroid = centroid;
    p->normal = c->pca_point_comp3;
    p->d = fabs(centroid.dot(c->pca_point_comp3));
    polygons.push_back(p);
  }

  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
  std::vector<std::vector<int> > projection;
  //PrimeSense::perspectiveProjection(tf, polygons, p->width, p->height, projection);

  // TODO: save projected results as image
  return 0;
}
