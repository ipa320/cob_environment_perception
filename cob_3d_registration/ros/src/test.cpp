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

// ROS includes
#include <ros/ros.h>
#include <pcl/point_types.h>
#define PCL_MINOR (PCL_VERSION[2] - '0')

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <cob_3d_registration/feature_container.h>
//#include <registration/registration_correspondence.h>
#include <pcl/visualization/cloud_viewer.h>

void
show (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  //... populate cloud
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }
}

Eigen::Matrix4f getTransf(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &ap, const Eigen::Vector3f &bp)
{
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> trans_est;
  pcl::PointCloud<pcl::PointXYZRGB> pc, pc2;

  pcl::PointXYZRGB p;

  p.x=a(0);
  p.y=a(1);
  p.z=a(2);
  pc.points.push_back(p);
  p.x=b(0);
  p.y=b(1);
  p.z=b(2);
  pc.points.push_back(p);

  p.x=ap(0);
  p.y=ap(1);
  p.z=ap(2);
  pc2.points.push_back(p);
  p.x=bp(0);
  p.y=bp(1);
  p.z=bp(2);
  pc2.points.push_back(p);

  Eigen::Matrix4f m=Eigen::Matrix4f::Identity();
  trans_est.estimateRigidTransformation (pc, pc2, m);

  return m;
}

int main(int argc, char **argv) {

  {
    pcl::PointCloud<pcl::PointXYZ> pc, pc2;
    pcl::PointXYZ p;

    p.x=0;
    p.y=0;
    p.z=0.1;
    pc.points.push_back(p);

    p.x=0;
    p.y=1;
    p.z=1;
    pc.points.push_back(p);

    Eigen::Vector3f a,b;
    a(0)=0;a(1)=0;a(2)=1;
    b(0)=0.1;a(1)=0;b(2)=1;

    Eigen::Quaternionf q;
    q.setFromTwoVectors(a,b);

    Eigen::Matrix4f m=Eigen::Matrix4f::Identity(), m2;

    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        m(i,j)=q.toRotationMatrix()(i,j);

    m(1,3)=0.5;

    std::cout<<m<<"\n\n";

    pcl::transformPointCloud(pc,pc2,m);

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
    trans_est.estimateRigidTransformation (pc, pc2, m2);

    std::cout<<m2<<"\n\n";
    Eigen::Vector4f t=p.getArray4fMap();
    std::cout<<(m*t)<<"\n\n";
    std::cout<<(m2*t)<<"\n\n";
    std::cout<<getTransf(pc[0].getVector3fMap(),pc[1].getVector3fMap(), pc2[0].getVector3fMap(),pc2[1].getVector3fMap())<<"\n\n";
  }

 /* if(argc<2)
    return 0;

  Keypoints_Segments<pcl::PointXYZRGB> seg;
  pcl::PointCloud<pcl::PointXYZRGB> pc,pc_out;
  pcl::PointCloud<PCAPoint> mids;
  pcl::io::loadPCDFile(argv[1],pc);

  //show(pc.makeShared());

  seg.extractFeatures(pc,pc_out,mids);

  pcl::io::savePCDFileBinary("a.pcd",pc_out);
  //pcl::io::savePCDFileBinary("b.pcd",mids);
  //show(pc_out.makeShared());
  //show(mids.makeShared());*/

  return 0;
}
