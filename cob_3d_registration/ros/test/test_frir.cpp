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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
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
/*
 * test_frir.cpp
 *
 *  Created on: 16.05.2012
 *      Author: josh
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/point_types.h>
#define PCL_MINOR (PCL_VERSION[2] - '0')

#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <pcl/io/pcd_io.h>
#include <cob_3d_registration/general_registration.h>
#include <cob_3d_registration/registration_info.h>

#include <cob_3d_registration/measurements/measure.h>
#include <boost/progress.hpp>

#include <gtest/gtest.h>

#define CYCLES 100
#define BLOCKSIZE 40

void generateRandomOrderedPC_Kinect(const int w, const int h, pcl::PointCloud<pcl::PointXYZ> &pc) {
  int x0 = w/2;
  int y0 = h/2;
  float f = 500.f;

  pc.clear();
  pc.width=w;
  pc.height=h;
  pc.resize(w*h);

  ROS_ASSERT(w%BLOCKSIZE==0);
  ROS_ASSERT(h%BLOCKSIZE==0);

  for(int xx=0; xx<w; xx+=BLOCKSIZE) {
    for(int yy=0; yy<h; yy+=BLOCKSIZE) {
      pcl::PointXYZ p;
      p.z = (rand()%1000)/300.f;
      for(int _x=0; _x<BLOCKSIZE; _x++)
        for(int _y=0; _y<BLOCKSIZE; _y++)
        {
          p.x = p.z*(xx+_x-x0)/f;
          p.y = p.z*(yy+_y-y0)/f;
          pc(xx+_x,yy+_y)=p;
        }
    }
  }
}

#define getInd(x, y) ((x)+(y)*pc.width)
void reproject(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZ> &pc_out, const Eigen::Matrix4f T)
{
  int x0 = pc.width/2;
  int y0 = pc.height/2;
  float f = 500.f;

  pc_out = pc;

  for(int xx=0; xx<pc.width; xx++)
    for(int yy=0; yy<pc.height; yy++)
      pc_out[getInd(xx,yy)].x = pc_out[getInd(xx,yy)].y = pc_out[getInd(xx,yy)].z = std::numeric_limits<float>::quiet_NaN();

  //raycast
  for(int xx=0; xx<pc.width; xx++) {
    for(int yy=0; yy<pc.height; yy++) {
      int ind = getInd(xx,yy);
      if(pcl_isfinite(pc[ind].z)) {
        Eigen::Vector4f v=T*pc[ind].getVector4fMap();

        int x=round(f*v(0)/v(2)+x0);
        int y=round(f*v(1)/v(2)+y0);

        if(x<0||x>=pc.width || y<0||y>=pc.height)
          continue;

        pc_out[getInd(x,y)].x = v(0);
        pc_out[getInd(x,y)].y = v(1);
        pc_out[getInd(x,y)].z = v(2);
      }
    }
  }

}


void generateNaNOrderedPC_Kinect(const int w, const int h, pcl::PointCloud<pcl::PointXYZ> &pc) {
  pc.clear();
  pc.width=w;
  pc.height=h;

  pcl::PointXYZ p;
  p.x=p.y=p.z=std::numeric_limits<float>::quiet_NaN();
  for(int xx=0; xx<w; xx++) {
    for(int yy=0; yy<h; yy++) {
      pc.push_back(p);
    }
  }
}

void test_null_ptrs() {
  time_t ti = time(NULL);
  ROS_INFO("init test_null_ptrs with %d",ti);
  srand(ti);

  cob_3d_registration::Registration_Infobased<pcl::PointXYZ> frir;
  frir.compute();
}

void test_nans() {
  ROS_INFO("test_nans");
  pcl::PointCloud<pcl::PointXYZ> pc1;
  generateNaNOrderedPC_Kinect(640,480, pc1);

  //register
  cob_3d_registration::Registration_Infobased<pcl::PointXYZ> frir;
  frir.setKinectParameters(500,320,240);
  frir.setInputOginalCloud(pc1.makeShared());
  frir.compute();
  frir.setInputOginalCloud(pc1.makeShared());
  frir.compute();
}

void test_many_random_pcs() {
  time_t ti = time(NULL);
  ROS_INFO("init test_many_random_pcs with %d",ti);
  srand(ti);

  boost::progress_display show_progress(CYCLES);
  int success=0, num=0;
  double dur=0.;
  for(int i=0; i<CYCLES; i++)
  {
    pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
    generateRandomOrderedPC_Kinect(640,480, pc1);
    generateRandomOrderedPC_Kinect(640,480, pc2);

    //register
    cob_3d_registration::Registration_Infobased<pcl::PointXYZ> frir;
    frir.setKinectParameters(500,320,240);
    frir.setInputOginalCloud(pc1.makeShared());
    frir.compute();
    frir.setInputOginalCloud(pc2.makeShared());
    measurement_tools::PrecisionStopWatchAll psw;
    if(frir.compute())
      ++success;
    dur+=psw.precisionStop();
    ++num;

    ++show_progress;
  }

  ROS_INFO("took %f",dur/(float)num);
  ROS_INFO("success rate %f",success/(float)num);
}

void test_many_tfs() {
  time_t ti = time(NULL);
  ROS_INFO("init test_many_tfs with %d",ti);
  srand(ti);

  boost::progress_display show_progress(CYCLES);
  int success=0, num=0;
  float dis=0.f;
  float disS=0.f;
  double dur=0.;
  int sucs[10]={};

  for(int i=0; i<CYCLES; i++)
  {
    pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
    generateRandomOrderedPC_Kinect(640,480, pc1);

    {
      Eigen::Matrix4f tf;
      tf=tf.Identity();
      tf.col(3)(2) = i/(float)(10*CYCLES);
      reproject(pc1,pc2,tf);

      //register
      cob_3d_registration::Registration_Infobased<pcl::PointXYZ> frir;
      frir.setKinectParameters(500,320,240);
      frir.setInputOginalCloud(pc1.makeShared());
      frir.compute();
      frir.setInputOginalCloud(pc2.makeShared());
      measurement_tools::PrecisionStopWatchAll psw;
      if(frir.compute()) {
        ++success;
        disS+=(frir.getTransformation()-tf).norm();
        sucs[i*10/CYCLES]++;
      }
      dur+=psw.precisionStop();
      ++num;

      dis+=(frir.getTransformation()-tf).norm();
    }

    {
      Eigen::Vector3f v;
      v.fill(0.f);v(0)=1.f;
      Eigen::AngleAxisf aa(i/(float)(10*CYCLES),v);
      Eigen::Matrix4f tf;
      tf=tf.Identity();
      tf.topLeftCorner<3,3>() = aa.toRotationMatrix();
      reproject(pc1,pc2,tf);

      //pcl::io::savePCDFile("a.pcd",pc1);
      //pcl::io::savePCDFile("b.pcd",pc2);

      //register
      cob_3d_registration::Registration_Infobased<pcl::PointXYZ> frir;
      frir.setKinectParameters(500,320,240);
      frir.setInputOginalCloud(pc1.makeShared());
      frir.compute();
      frir.setInputOginalCloud(pc2.makeShared());
      measurement_tools::PrecisionStopWatchAll psw;
      if(frir.compute()) {
        ++success;
        disS+=(frir.getTransformation()-tf).norm();
        sucs[i*10/CYCLES]++;
      }
      dur+=psw.precisionStop();
      ++num;

      dis+=(frir.getTransformation()-tf).norm();
    }

    ++show_progress;
  }

  ROS_INFO("took %f",dur/(float)num);
  ROS_INFO("success  rate %f",success/(float)num);
  ROS_INFO("distance rate %f",dis/(float)num);
  ROS_INFO("distance rate %f (on success)",disS/(float)num);

  for(int i=0; i<10; i++)
    std::cout<<sucs[i]<<"\t ";
  std::cout<<"\n";
}

TEST(Registration_FRIF, null_ptrs){
  test_null_ptrs();
}

TEST(Registration_FRIF, nans){
  test_nans();
}

TEST(Registration_FRIF, many_random_pcs){
  test_many_random_pcs();
}

TEST(Registration_FRIF, many_tfs){
  test_many_tfs();
}

int main(int argc, char **argv) {
//  ROS_INFO("EVALUATION_MODE_ is %d", EVALUATION_MODE_);
//  ROS_INFO("DEBUG_SWITCH_ is %d", DEBUG_SWITCH_);
//  ROS_INFO("USED_ODO_ is %d", USED_ODO_);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
