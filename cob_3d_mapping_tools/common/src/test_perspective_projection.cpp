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
#include <cob_3d_mapping_common/stop_watch.h>
#include <cob_3d_fov_segmentation/projection.h>

#include <cob_3d_features/organized_normal_estimation_omp.h>
#include <cob_3d_segmentation/impl/fast_segmentation.hpp>
#include <cob_3d_segmentation/cluster_conversion.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointC;
typedef pcl::PointCloud<pcl::Normal> NormalC;
typedef pcl::PointCloud<PointLabel> LabelC;
typedef cob_3d_segmentation::FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel>::ClusterHdlPtr ClusterHdlPtr;
typedef cob_3d_segmentation::FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel>::ClusterPtr ClusterPtr;

int main (int argc, char** argv)
{
  PrecisionStopWatch t;

  PointC::Ptr p(new PointC);
  PointC::Ptr s(new PointC);
  NormalC::Ptr n(new NormalC);
  LabelC::Ptr l(new LabelC);

  pcl::PCDReader r;
  r.read(argv[1], *p);
  *s = *p;

  cob_3d_features::OrganizedNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> one;
  one.setOutputLabels(l);
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
  seg.mapSegmentColor(s);

  std::vector<cob_3d_mapping::Polygon::Ptr> polygons;
  cob_3d_segmentation::ClusterConversion<> cc;
  cc.setClusterHandler(seg.clusters());
  cc.setInputCloud(p);
  cc.setMinClusterSize(100);
  cc.setMaxCentroidDistance(5.0f);
  cc.convertToPolygons(polygons);

  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
  std::vector<std::vector<int> > projection;
  t.precisionStart();
  cob_3d_mapping::PerspectiveProjection<cob_3d_mapping::PrimeSense>
    ::compute(tf, polygons, p->width, p->height, projection)
;
  std::cout << "Projection: "<< t.precisionStop() << std::endl;

  for(int i=0; i<projection.size(); ++i)
  {
    if(projection[i].size() > 1)
    {
      //for(int j=0; j<projection[i].size(); ++j) std::cout << projection[i][j] << ", ";
      //std::cout << std::endl;
    }
    if(projection[i].size())
      (*p)[i].rgb = (*s)[ seg.clusters()->getCluster(projection[i][0])->indices_[0] ].rgb;
  }

  pcl::PCDWriter w;
  w.write("segmented.pcd", *s);
  w.write("projected.pcd", *p);

  return 0;
}
