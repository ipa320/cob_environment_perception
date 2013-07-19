/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 06/2013
*
* \brief
* ?
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#include <gtest/gtest.h>
#include <cob_3d_mapping_common/cylinder.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>

using namespace cob_3d_mapping;

struct CylinderData
{
  CylinderData(unsigned int id,
              std::vector<float> color,
              Eigen::Vector3f sym_axis,
              Eigen::Vector3f x_axis,
              Eigen::Vector3f origin,
              double r,
              std::vector<std::vector<Eigen::Vector3f> > contours,
              std::vector<bool> holes)
  {
    id_ = id;
    color_ = color;
    sym_axis_ = sym_axis.normalized();
    origin_ = origin;
    r_ = r;
    if (sym_axis_[2] < 0 )
    {
      sym_axis_ = sym_axis_ * -1;
    }
    contours_3d_cyl_ = contours;
    holes_ = holes;
    Eigen::Affine3f t;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(sym_axis_, x_axis.cross(sym_axis_), origin_, t);
    std::cout << x_axis.cross(sym_axis_) << std::endl;
    pose_ = t.inverse();
    for(unsigned int i = 0; i < contours_3d_cyl_.size(); i++)
    {
      std::vector<Eigen::Vector3f> c_3d;
      for(unsigned int j = 0; j < contours_3d_cyl_[i].size(); j++)
      {
        Eigen::Vector3f p_3d = pose_ * contours_3d_cyl_[i][j];
        c_3d.push_back(p_3d);
      }
      contours_3d_.push_back(c_3d);
    }
  }

  unsigned int id_;
  std::vector<float> color_;
  Eigen::Vector3f sym_axis_;
  Eigen::Vector3f origin_;
  double r_;
  std::vector<std::vector<Eigen::Vector2f> > contours_2d_;
  std::vector<std::vector<Eigen::Vector3f> > contours_3d_cyl_;
  std::vector<std::vector<Eigen::Vector3f> > contours_3d_;
  std::vector<bool> holes_;
  Eigen::Affine3f pose_;
  std::vector<unsigned int> merge_candidates_;
};


class CylinderTest : public ::testing::Test
{
protected:
  virtual void
  SetUp ()
  {
    std::vector<std::vector<Eigen::Vector3f> > contours;
    Eigen::Vector3f p1(-1,-1,0), p2(0,-1,1), p3(1,-1,0), p4(1,1,0), p5(0,1,1), p6(-1,1,0);
    std::vector<Eigen::Vector3f> c;
    c.push_back(p1);
    c.push_back(p2);
    c.push_back(p3);
    c.push_back(p4);
    c.push_back(p5);
    c.push_back(p6);
    contours.push_back(c);
    std::vector<bool> holes;
    holes.push_back(false);
    CylinderData* cd1 = new CylinderData(0, std::vector<float>(4,1), Eigen::Vector3f(0,1,0), Eigen::Vector3f(1,0,0), Eigen::Vector3f(0,0,0), 1.0, contours, holes);
    cd_.push_back(cd1);

    contours.clear();
    c.clear();
    holes.clear();
    p1 = Eigen::Vector3f(-0.71,-1,0.71);
    //p2 = Eigen::Vector3f(1,-1,1);
    p3 = Eigen::Vector3f(0.71,-1,0.71);
    p4 = Eigen::Vector3f(0.71,1,0.71);
    //p5 = Eigen::Vector3f(1,1,1);
    p6 = Eigen::Vector3f(-0.71,1,0.71);
    c.push_back(p1);
    c.push_back(p2);
    c.push_back(p3);
    c.push_back(p4);
    c.push_back(p5);
    c.push_back(p6);
    contours.push_back(c);
    holes.push_back(false);
    CylinderData* cd2 = new CylinderData(0, std::vector<float>(4,1), Eigen::Vector3f(0,1,0), Eigen::Vector3f(1,0,0), Eigen::Vector3f(1,0,0), 1.0, contours, holes);
    cd_.push_back(cd2);
  }

  std::vector<CylinderData*> cd_;
};

/*TEST_F(CylinderTest, radiusFromCloud)
{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile("cylinder00000.pcd", *pc);
  std::vector<int> indices;
  for( unsigned int i = 0; i < pc->size(); i++)
    indices.push_back(i);
  Eigen::Vector3f origin(0, 0.5, 4);
  Eigen::Vector3f sym_axis(0, 1, 0);
  double r = radiusFromCloud(pc, indices, origin, sym_axis);
  EXPECT_NEAR(r, 0.25, 0.01);
}*/

TEST_F(CylinderTest, creationWorks)
{
  for(unsigned int i = 0; i < cd_.size(); i++)
  {
    cob_3d_mapping::Cylinder c(cd_[i]->id_, cd_[i]->origin_, cd_[i]->sym_axis_, cd_[i]->r_, cd_[i]->contours_3d_, cd_[i]->holes_, cd_[i]->color_);
    std::cout << "Pose " << c.pose_.matrix() << std::endl;
    std::cout << "Pose Inverse " << c.pose_.inverse().matrix() << std::endl;
    std::vector<std::vector<Eigen::Vector3f> > c_3d = c.getContours3D();
    std::cout << "Actual 3D contours" << std::endl;
    for(unsigned int j = 0; j < c_3d.size(); j++)
    {
      for(unsigned int k = 0; k < c_3d[j].size(); k++)
      {
        std::cout << cd_[i]->contours_3d_[j][k](0) << "," << cd_[i]->contours_3d_[j][k](1) << "," << cd_[i]->contours_3d_[j][k](2) << std::endl;
      }
    }
    std::cout << "Actual 3D contours in cyl coords" << std::endl;
    for(unsigned int j = 0; j < c_3d.size(); j++)
    {
      for(unsigned int k = 0; k < c_3d[j].size(); k++)
      {
        std::cout << cd_[i]->contours_3d_cyl_[j][k](0) << "," << cd_[i]->contours_3d_cyl_[j][k](1) << "," << cd_[i]->contours_3d_cyl_[j][k](2) << std::endl;
      }
    }
    std::cout << "Computed 3D contours" << std::endl;
    for(unsigned int j = 0; j < c_3d.size(); j++)
    {
      for(unsigned int k = 0; k < c_3d[j].size(); k++)
      {
        std::cout << c_3d[j][k](0) << "," << c_3d[j][k](1) << "," << c_3d[j][k](2) << std::endl;
      }
    }
    std::cout << "Computed 2D contours" << std::endl;
    for(unsigned int j = 0; j < c.contours_.size(); j++)
    {
      for(unsigned int k = 0; k < c.contours_[j].size(); k++)
      {
        std::cout << c.contours_[j][k](0) << "," << c.contours_[j][k](1) << std::endl;
      }
    }
    /*EXPECT_NEAR(pd_[i]->normal_.dot(p.normal_), 1.0, 0.001);
    EXPECT_NEAR(pd_[i]->d_, p.d_, 0.001);
    EXPECT_NEAR((p.pose_.rotation() * Eigen::Vector3f(0,0,1)).dot(pd_[i]->normal_), 1.0, 0.001);
    EXPECT_NEAR((p.pose_.translation() - pd_[i]->origin_).norm(), 0, 0.001);
    std::vector<std::vector<Eigen::Vector3f> > c_3d = p.getContours3D();
    //std::cout << "c of p " << i << ":" << std::endl;
    for(unsigned int j = 0; j < c_3d.size(); j++)
    {
      for(unsigned int k = 0; k < c_3d[j].size(); k++)
      {
        EXPECT_NEAR((pd_[i]->contours_3d_[j][k] - c_3d[j][k]).norm(), 0, 0.001);
        //std::cout << c_3d[j][k](0) << "," << c_3d[j][k](1) << "," << c_3d[j][k](2) << std::endl;
        //std::cout << pd_[i]->contours_3d_[j][k](0) << "," << pd_[i]->contours_3d_[j][k](1) << "," << pd_[i]->contours_3d_[j][k](2) << std::endl;
      }
    }*/
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

