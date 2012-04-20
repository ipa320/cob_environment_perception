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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
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

// Boost:
#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>

// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// Stack Includes:
#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_mapping_features/organized_normal_estimation.h"
#include "cob_3d_mapping_features/organized_curvature_estimation.h"
#include "cob_3d_mapping_features/extended_segmentation.h"

using namespace std;
using namespace pcl;

string file_in_;
float d_th_, upper_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<string>(&file_in_), "input pcd file")
    ("skip_distant_point,d", value<float>(&d_th_)->default_value(8), "threshold to ignore distant points in neighborhood")
    ("upper,u", value<float>(&upper_)->default_value(6.0), "upper curvature threshold (edge)")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || argc == 1)
  {
    cout << "\"pns\" stands for \"pixel neighborhood size\" and refers to N x N mask for nearest neighbor search\n"
	 << "\t where N = 2 * pns + 1\n" << endl;
    cout << options << endl;
    exit(0);
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr cp(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr cp2nd(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr tp(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<Normal>::Ptr n2(new PointCloud<Normal>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);
  PointCloud<PointLabel>::Ptr l(new PointCloud<PointLabel>);

  PointCloud<PointXYZ>::Ptr centroids1st(new PointCloud<PointXYZ>);
  PointCloud<Normal>::Ptr normals1st(new PointCloud<Normal>);

  cob_3d_mapping_features::ClusterList cluster_list;

  PCDReader r;
  if (r.read(file_in_, *p) == -1) return(0);
  *tp = *cp2nd = *cp = *p;

  PrecisionStopWatch t;
  t.precisionStart();
  cob_3d_mapping_features::OrganizedNormalEstimation<PointXYZRGB, Normal, PointLabel>one;
  one.setInputCloud(p);
  one.setOutputLabels(l);
  one.setPixelSearchRadius(8,2,2); //radius,pixel,circle
  one.setSkipDistantPointThreshold(d_th_);
  one.compute(*n);
  cout << t.precisionStop() << "s\t for Organized Normal Estimation" << endl;
  *n2 = *n;
  /*
  t.precisionStart();
  cob_3d_mapping_features::OrganizedCurvatureEstimation<PointXYZRGB,Normal,PointLabel,PrincipalCurvatures>oce;
  oce.setInputCloud(p);
  oce.setInputNormals(n);
  oce.setOutputLabels(l);
  oce.setPixelSearchRadius(8,2,2);
  oce.setSkipDistantPointThreshold(d_th_);
  oce.setEdgeClassThreshold(upper_);
  oce.compute(*pc);
  cout << t.precisionStop() << "s\t for Organized Curvature Estimation" << endl;
  */

  
  t.precisionStart();
  cob_3d_mapping_features::ExtendedSegmentation<PointXYZRGB,Normal,PrincipalCurvatures,PointLabel> eseg;
  eseg.setInputPoints(p);
  eseg.setInputNormals(n);
  //eseg.setInputCurvatures(pc);
  eseg.setOutputLabels(l);
  eseg.propagateWavefront(cluster_list);
  eseg.getColoredCloud(cluster_list, cp);
  //eseg.propagateWavefront2ndPass(cluster_list);
  centroids1st->points.resize(cluster_list.size());
  normals1st->points.resize(cluster_list.size());
  centroids1st->width = normals1st->width = 1;
  centroids1st->height = normals1st->height = cluster_list.size();
  int i2 = 0;
  for (cob_3d_mapping_features::ClusterPtr c = cluster_list.begin(); c != cluster_list.end(); ++c, ++i2)
  {
    centroids1st->points[i2].getVector3fMap() = c->getCentroid();
    normals1st->points[i2].getNormalVector3fMap() = c->getOrientation();
  }

  eseg.analyseClusters(cluster_list);
  eseg.getColoredCloud(cluster_list, cp2nd);
  eseg.getColoredCloudByType(cluster_list, tp);
  cout << t.precisionStop() << "s\t for extended clustering" << endl;
  cout << "Found " << cluster_list.size() << " clusters." << endl;

  PointCloud<PointXYZ>::Ptr centroids(new PointCloud<PointXYZ>);
  PointCloud<Normal>::Ptr comp1(new PointCloud<Normal>);
  PointCloud<Normal>::Ptr comp2(new PointCloud<Normal>);
  PointCloud<Normal>::Ptr comp3(new PointCloud<Normal>);

  PointCloud<PointXYZRGB>::Ptr ints_centroids(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr ints_comp1(new PointCloud<Normal>);
  PointCloud<Normal>::Ptr ints_comp2(new PointCloud<Normal>);
  PointCloud<Normal>::Ptr ints_comp3(new PointCloud<Normal>);
  PointCloud<Normal>::Ptr connection(new PointCloud<Normal>);
  

  centroids->points.resize(cluster_list.size());
  centroids->width = comp1->width = comp2->width = comp3->width = 1;
  centroids->height = cluster_list.size();
  comp1->points.resize(cluster_list.size());
  comp1->height = cluster_list.size();
  comp2->points.resize(cluster_list.size());
  comp2->height = cluster_list.size();
  comp3->points.resize(cluster_list.size());
  comp3->height = cluster_list.size();

  ints_centroids->width = ints_comp1->width = ints_comp2->width = ints_comp3->width = connection->width = 1;
  ints_centroids->height = ints_comp1->height = ints_comp2->height = ints_comp3->height = connection->height = cluster_list.size();
  ints_centroids->points.resize(cluster_list.size());
  ints_comp1->points.resize(cluster_list.size());
  ints_comp2->points.resize(cluster_list.size());
  ints_comp3->points.resize(cluster_list.size());
  connection->points.resize(cluster_list.size());


  int i = 0;
  for (cob_3d_mapping_features::ClusterPtr c = cluster_list.begin(); c != cluster_list.end(); ++c)
  {
    if (c->indices.size() < 100) continue;
    centroids->points[i].getVector3fMap() = c->getCentroid();
    comp1->points[i].getNormalVector3fMap() = c->pca_point_comp1 * c->pca_point_values(2);
    comp2->points[i].getNormalVector3fMap() = c->pca_point_comp2 * c->pca_point_values(1);
    comp3->points[i].getNormalVector3fMap() = c->pca_point_comp3 * c->pca_point_values(0);


    ints_centroids->points[i].getVector3fMap() = c->pca_inter_centroid;
    ints_centroids->points[i].r = cp2nd->points[c->indices[0]].r;
    ints_centroids->points[i].g = cp2nd->points[c->indices[0]].g;
    ints_centroids->points[i].b = cp2nd->points[c->indices[0]].b;
    ints_comp1->points[i].getNormalVector3fMap() = c->pca_inter_comp1 * c->pca_inter_values(2);
    ints_comp2->points[i].getNormalVector3fMap() = c->pca_inter_comp2 * c->pca_inter_values(1);
    ints_comp3->points[i].getNormalVector3fMap() = c->pca_inter_comp3 * c->pca_inter_values(0);

    connection->points[i].getNormalVector3fMap() = c->pca_inter_centroid - c->getCentroid();

    ++i;
  }

  visualization::PCLVisualizer v;
  visualization::PointCloudColorHandlerRGBField<PointXYZRGB> col_hdl(cp);
  visualization::PointCloudColorHandlerRGBField<PointXYZRGB> col_hdl_2nd(cp2nd);
  visualization::PointCloudColorHandlerRGBField<PointXYZRGB> type_col_hdl(tp);
  visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ints_hdl(ints_centroids);
  visualization::PointCloudColorHandlerCustom<PointXYZ> blue_hdl (centroids, 0,0,255);
  visualization::PointCloudColorHandlerCustom<PointXYZRGB> white_hdl (p, 255,255,255);
  visualization::PointCloudColorHandlerCustom<PointXYZRGB> black_hdl (p, 0, 0, 0);

  /* --- Viewports: ---
   *  1y
   *    | 1 | 3 |
   * .5 ----+----
   *    | 2 | 4 |
   *  0    .5    1x
   * 1:
   */
  // xmin, ymin, xmax, ymax
  int v1(0);
  v.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
  v.setBackgroundColor(0, 0.75, 0.75, v1);
  v.addPointCloud<PointXYZRGB>(cp, col_hdl, "segmented", v1);

  int v3(0);
  v.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
  v.setBackgroundColor(0, 0.75, 0.75, v3);
  v.addPointCloud<PointXYZRGB>(cp2nd, col_hdl_2nd, "segmented2nd", v3);

  int v4(0);
  v.createViewPort(0.5, 0.5, 1.0, 1.0, v4);
  v.setBackgroundColor(0, 0.75, 0.75, v4);
  v.addPointCloud<PointXYZRGB>(p, black_hdl, "normals", v4);
  v.addPointCloudNormals<PointXYZRGB, Normal>(p,n,4,0.04,"normals2", v4);
  //v.addPointCloudNormals<PointXYZ, Normal>(centroids1st,normals1st,1,0.1,"normals2", v4);


  int v2(0);
  v.createViewPort(0.5, 0.0, 1.0, 0.5, v2);
  v.setBackgroundColor(0, 0.75, 0.75, v2);
  v.addPointCloud<PointXYZRGB>(p, black_hdl, "normals4", v2);
  //v.addPointCloudNormals<PointXYZRGB, Normal>(p,n,4,0.04,"normals2", v4);
  v.addPointCloudNormals<PointXYZRGB, Normal>(p,n2,4,0.04,"normals3", v2);
  /*
  //v.addPointCloud<PointXYZRGB>(p, white_hdl, "cloud", v2);
  v.addPointCloud<PointXYZRGB>(tp, type_col_hdl, "types", v2);
  v.addPointCloud<PointXYZ>(centroids, blue_hdl, "clusters", v2);
  /*v.addPointCloudNormals<PointXYZ,Normal>(centroids, comp1, 1, 10.0, "comp1", v2);
  v.addPointCloudNormals<PointXYZ,Normal>(centroids, comp2, 1, 10.0, "comp2", v2);
  v.addPointCloudNormals<PointXYZ,Normal>(centroids, comp3, 1, 100.0, "comp3", v2);
  v.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "comp1", v2);
  v.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "comp2", v2);
  v.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "comp3", v2);/

  v.addPointCloud<PointXYZRGB>(ints_centroids, ints_hdl, "ints_centroid", v2);
  v.addPointCloudNormals<PointXYZRGB,Normal>(ints_centroids, ints_comp1, 1, 10.0, "ints_comp1", v2);
  v.addPointCloudNormals<PointXYZRGB,Normal>(ints_centroids, ints_comp2, 1, 10.0, "ints_comp2", v2);
  v.addPointCloudNormals<PointXYZRGB,Normal>(ints_centroids, ints_comp3, 1, 10.0, "ints_comp3", v2);
  v.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "ints_comp1", v2);
  v.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "ints_comp2", v2);
  v.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "ints_comp3", v2);
  v.addPointCloudNormals<PointXYZ,Normal>(centroids, connection, 1, 1.0, "connections", v2);
  */
  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }

  return 0;
}
