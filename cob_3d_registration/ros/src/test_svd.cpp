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
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_svd.h>
//#include <registration/registration_correspondence.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#define ADD_NOISE() \
    p.x+=var_nor();\
    p.y+=var_nor();\
    p.z+=var_nor();

pcl::PointCloud<pcl::PointXYZ> generateData(int data) {
  //normal distribution
  boost::mt19937 rng;
  rng.seed(clock());

  boost::normal_distribution<> nd(0, 0.1*data);

  boost::variate_generator<boost::mt19937&,
  boost::normal_distribution<> > var_nor(rng, nd);

  pcl::PointCloud<pcl::PointXYZ> pc;
  for(int i=0; i<100; i++) {
    pcl::PointXYZ p;

    p.x=(i-50)/50.;
    p.y=0;
    p.z=1;
    ADD_NOISE();
    pc.push_back(p);

    p.y=1;
    ADD_NOISE();
    pc.push_back(p);

    p.z=2;
    ADD_NOISE();
    pc.push_back(p);

    p.y=0;
    ADD_NOISE();
    pc.push_back(p);
  }
  for(int i=1; i<99; i++) {
    pcl::PointXYZ p;

    p.x=-1;
    p.y=0;
    p.z=1+i/100.;
    ADD_NOISE();
    pc.push_back(p);

    p.x=1;
    ADD_NOISE();
    pc.push_back(p);

    p.y=1;
    ADD_NOISE();
    pc.push_back(p);

    p.x=-1;
    ADD_NOISE();
    pc.push_back(p);
  }

  return pc;
}

void generateTF(int trans, Eigen::Quaternionf &R,  Eigen::Vector3f &t, Eigen::Matrix3f &mR)
{
  float strengthR=(trans%3)*0.05;
  trans/=3;
  float strengthT=(trans%3)*0.05;
  trans/=3;

  int temp=trans%8;
  Eigen::Vector3f axis;
  axis(0) = temp&1?1:0;
  axis(1) = temp&2?1:0;
  axis(2) = temp&4?1:0;
  trans/=8;

  Eigen::AngleAxisf aa(strengthR,axis);
  R = aa.toRotationMatrix();
  mR= R.toRotationMatrix();
  R = mR;

  t(0) = temp&1?1:0;
  if(temp&2) t(0)=-t(0);
  t(1) = temp&4?1:0;
  if(temp&8) t(1)=-t(1);
  t(2) = temp&16?1:0;
  if(temp&32) t(2)=-t(2);
  trans/=64;

  t*=strengthT;
}

void transform(pcl::PointCloud<pcl::PointXYZ> &pc, Eigen::Quaternionf &R,  Eigen::Vector3f &t)
{
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      m(i,j) = R.toRotationMatrix()(i,j);
  m.col(3).head<3>() = t;

  pcl::transformPointCloud(pc,pc,m);
}

void addNoise(pcl::PointCloud<pcl::PointXYZ> &pc, int noise)
{
  //normal distribution
  boost::mt19937 rng;
  rng.seed(clock());

  boost::normal_distribution<> nd(0, 0.02*noise);

  boost::variate_generator<boost::mt19937&,
  boost::normal_distribution<> > var_nor(rng, nd);

  for(int i=0; i<pc.size(); i++) {
    pcl::PointXYZ &p = pc[i];
    ADD_NOISE();
  }

}

FILE *fp = NULL;
void write(const char *str)
{
  ROS_ASSERT(fp);
  fputs(str,fp);
}
void write(const int n)
{
  char str[128];
  sprintf(str,"%d",n);
  write(str);
}
void write(const float n)
{
  char str[128];
  sprintf(str,"%f",n);
  write(str);
}

int main(int argc, char **argv) {
  if(argc<2) {
    printf("specify output path");
    return 1;
  }

  fp = fopen(argv[1],"w");

  write("<svd>");

  pcl::PointCloud<pcl::PointXYZ> pc1, pc2;

  std::cout<<"\n";
  for(int trans=0; trans<3*3*8*64; trans++) {
	  if( !(trans/(3*3*8))&1 && (trans/(3*3*8))&2 )
		  continue;
	  if( !(trans/(3*3*8))&4 && (trans/(3*3*8))&8 )
		  continue;
	  if( !(trans/(3*3*8))&16 && (trans/(3*3*8))&32 )
		  continue;
    std::cout<<(trans*100/(10*10*8*64))<<"%\r";
    Eigen::Quaternionf R;
    Eigen::Vector3f t;
	
    Eigen::Matrix3f mR_cmp;
    generateTF(trans, R,t,mR_cmp);

    for(int data=0; data<10; data++) {
      pc1=generateData(data);

      pc2=pc1;
      transform(pc2,R,t);

      for(int noise=0; noise<=10; noise++)
      {
        addNoise(pc2, noise);

        //get transformation
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
        Eigen::Matrix4f m=Eigen::Matrix4f::Identity();
        trans_est.estimateRigidTransformation (pc1, pc2, m);

        Eigen::Matrix3f m2;
        for(int i=0; i<3; i++)
          for(int j=0; j<3; j++)
            m2(i,j)=m(i,j);

        //compare to desired tf
        Eigen::Quaternionf Rn(m2);
        Eigen::Vector3f tn = m.col(3).head<3>();

        float error_r = Rn.angularDistance(R);
        float error_d = (tn-t).norm();
		
		/*if(error_r>0.01) {
  std::cout<<mR_cmp<<"\n";
  Rn=mR_cmp;
  std::cout<<Rn.toRotationMatrix()<<"\n";
  Rn.normalize();
  std::cout<<Rn.toRotationMatrix()<<"\n";
  std::cout<<m2<<"\n";
  std::cout<<"e: "<<error_r<<"\n";}*/

        //write
        write("<set>");

        write("<trans>");
        write((trans));
        write("</trans>");

        write("<data>");
        write((data));
        write("</data>");

        write("<noise>");
        write((noise));
        write("</noise>");

        write("<error_r>");
        write((error_r));
        write("</error_r>");

        write("<error_d>");
        write((error_d));
        write("</error_d>");

        write("</set>");
      }
    }
  }

  write("</svd>");

  fclose(fp);
  return 0;
}
