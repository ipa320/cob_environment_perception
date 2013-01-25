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
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: registration
 *
 * \author
 *  Author: goa-jh
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: Nov 28, 2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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


#include <pcl/features/normal_3d.h>
#include <cob_3d_mapping_features/edge_estimation_3d.h>
#include <cob_3d_mapping_features/edge_estimation_2d.h>
#include <cob_3d_mapping_features/edge_extraction.h>
#include <cob_3d_segmentation/segmentation.h>
#include <cob_3d_mapping_common/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/pca.h>
#ifdef PCL_VERSION_COMPARE
  #include <pcl/PointIndices.h>
#endif


template<typename Point>
void Keypoints_Segments<Point>::extractFeatures(const pcl::PointCloud<Point>& point_cloud, pcl::PointCloud<Point> &cloud_out2, pcl::PointCloud<PCAPoint> &mids) {

  /**************** 3D EDGES **************/
  ROS_INFO("3d edges");
  pcl::PointCloud<InterestPoint>::Ptr ip3d(new pcl::PointCloud<InterestPoint>);
  {
    pcl::PointCloud<Normal>::Ptr n(new pcl::PointCloud<Normal>);
    #ifdef PCL_VERSION_COMPARE
      boost::shared_ptr<pcl::search::KdTree<Point> > tree(new pcl::search::KdTree<Point>);
    #else
      boost::shared_ptr<pcl::KdTreeFLANN<Point> > tree(new pcl::KdTreeFLANN<Point>);
    #endif
    pcl::NormalEstimation<Point, Normal> ne;
    ne.setRadiusSearch(radius_);
    ne.setSearchMethod(tree);
    ne.setInputCloud(point_cloud.makeShared());
    ne.compute(*n);

    ROS_INFO("normals done");

#ifdef GICP_ENABLE
    boost::shared_ptr<pcl::search::OrganizedNeighbor<Point> > oTree (new pcl::search::OrganizedNeighbor<Point> );
#else
    #ifdef PCL_VERSION_COMPARE
      boost::shared_ptr<pcl::search::OrganizedNeighbor<Point> > oTree (new pcl::search::OrganizedNeighbor<Point> );
    #else
      boost::shared_ptr<pcl::OrganizedDataIndex<Point> > oTree (new pcl::OrganizedDataIndex<Point> );
    #endif
#endif
    cob_3d_mapping_features::EdgeEstimation3D<Point, Normal, InterestPoint> ee;
    ee.setRadiusSearch(radius_);
    ee.setSearchMethod(oTree);
    ee.setInputCloud(point_cloud.makeShared());
    ee.setInputNormals(n);
    ee.dist_threshold_ = dist_threshold_;
    ee.compute(*ip3d);
  }

  /**************** 2D EDGES **************/
  ROS_INFO("2d edges");
  pcl::PointCloud<InterestPoint>::Ptr ip2d(new pcl::PointCloud<InterestPoint>);
  {
    cob_3d_mapping_features::EdgeEstimation2D<Point, InterestPoint> ee;
    ee.setInputCloud(point_cloud.makeShared());
    ee.computeEdges(*ip2d);
  }

  ROS_INFO("segmentation");
  cob_3d_mapping_features::EdgeExtraction<InterestPoint,PointLabel> edge_ext;
  edge_ext.setInput3DEdges(ip3d);
  edge_ext.setInput2DEdges(ip2d);
  edge_ext.setThreshold(thr_);

  boost::shared_ptr<PointCloud<PointLabel> > edges(new PointCloud<PointLabel>);
  edge_ext.extractEdges(*edges);

  //segmentation
  cob_3d_segmentation::Segmentation seg;
  seg.propagateWavefront2(edges);

  std::vector<pcl::PointIndices> clusters;
  cv::Mat seg_img;
  seg.getClusterIndices(edges, clusters, seg_img);

  mids.clear();
  for(int i=0; i<clusters.size(); i++) {
    #ifdef PCL_VERSION_COMPARE
      if(clusters[i].indices.size()<point_cloud.size()*0.001)
    #else
      if(clusters[i].get_indices_size()<point_cloud.size()*0.001)
    #endif
      continue;

    pcl::PointCloud<Point> temp;
    pcl::ExtractIndices<Point> filter;
    filter.setInputCloud(point_cloud.makeShared());
    pcl::PointIndicesPtr pi(new pcl::PointIndices);
    *pi = clusters[i];
    filter.setIndices(pi);
    filter.filter(temp);

    pcl::PCA<Point> pca;
    pca.compute(temp);


    Eigen::Vector4f mid, m1, m2;// = pca.getMean();
    pcl::getMinMax3D(temp, m1, m2);
    mid=(m1+m2)*0.5;
    mid+=pca.getMean();
    mid*=0.5;

    /*std::cout<<"a1:\n"<<pca.getEigenVectors().row(0)<<"\n";
    std::cout<<"a2:\n"<<pca.getEigenVectors().row(1)<<"\n";
    std::cout<<"a3:\n"<<pca.getEigenVectors().row(2)<<"\n";*/

    PCAPoint pp;

    for(int k=0; k<3; k++) {

      for(int t=0; t<3; t++) pp.data_c[k*3+t]=pca.getEigenVectors().row(k)(t)*sqrtf(pca.getEigenValues()(k));
    }

    pp.x=mid(0);
    pp.y=mid(1);
    pp.z=mid(2);

    mids.points.push_back(pp);

    pp.x=m1(0);
    pp.y=m1(1);
    pp.z=m1(2);

    mids.points.push_back(pp);

    pp.x=m2(0);
    pp.y=m2(1);
    pp.z=m2(2);

    mids.points.push_back(pp);

    /*for(int k=0; k<3; k++)
    for(int t=0; t<20; t++) {
      Point p;
      p.x=mid(0)+t/20.*pca.getEigenVectors().row(k)(0)*sqrtf(pca.getEigenValues()(k));
      p.y=mid(1)+t/20.*pca.getEigenVectors().row(k)(1)*sqrtf(pca.getEigenValues()(k));
      p.z=mid(2)+t/20.*pca.getEigenVectors().row(k)(2)*sqrtf(pca.getEigenValues()(k));
      p.r=p.g=p.b=255;
      mids.points.push_back(p);
    }*/
  }
  mids.height=1;
  mids.width=mids.size();

  cloud_out2=point_cloud;

  ROS_INFO("%d",mids.size());

  /*for(int x=0;x<cloud_out2.width; x++) {
    for(int y=0;y<cloud_out2.height; y++) {
      cloud_out2(x,y).r=(*edges)(x,y).label*15;
      cloud_out2(x,y).g=((*edges)(x,y).label%20)*15;
      cloud_out2(x,y).b=((*edges)(x,y).label%4)*15;
    }
  }*/

}
