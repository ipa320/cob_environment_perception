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
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_mapping_common
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 05/2013
 *
 * \brief
 * Description: Tests for polygon class
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

#include <gtest/gtest.h>
#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include <pcl/common/eigen.h>


struct PolygonData
{
  PolygonData(unsigned int id,
              std::vector<float> color,
              Eigen::Vector3f normal,
              Eigen::Vector3f x_axis,
              Eigen::Vector3f origin,
              std::vector<std::vector<Eigen::Vector2f> > contours,
              std::vector<bool> holes)
  {
    id_ = id;
    color_ = color;
    normal_ = normal.normalized();
    origin_ = origin;
    d_ = origin_.dot(normal_);
    if (d_ > 0) {
      normal_ = -normal_;
      d_ = -d_;
    }
    contours_2d_ = contours;
    holes_ = holes;
    Eigen::Affine3f t;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal_.cross(x_axis), normal_, origin_, t);
    pose_ = t.inverse();
    for(unsigned int i = 0; i < contours_2d_.size(); i++)
    {
      std::vector<Eigen::Vector3f> c_3d;
      for(unsigned int j = 0; j < contours_2d_[i].size(); j++)
      {
        Eigen::Vector3f p_3d = pose_ * Eigen::Vector3f(contours_2d_[i][j](0), contours_2d_[i][j](1), 0);
        c_3d.push_back(p_3d);
      }
      contours_3d_.push_back(c_3d);
    }
  }

  unsigned int id_;
  std::vector<float> color_;
  Eigen::Vector3f normal_;
  double d_;
  Eigen::Vector3f origin_;
  std::vector<std::vector<Eigen::Vector2f> > contours_2d_;
  std::vector<std::vector<Eigen::Vector3f> > contours_3d_;
  std::vector<bool> holes_;
  Eigen::Affine3f pose_;
  std::vector<unsigned int> merge_candidates_;
};

class PolygonTest : public ::testing::Test
{
protected:
  virtual void
  SetUp ()
  {
    // Init pd1
    std::vector<std::vector<Eigen::Vector2f> > contours;
    Eigen::Vector2f p1(0,0), p2(0,1), p3(1,1), p4(1,0);
    std::vector<Eigen::Vector2f> c;
    c.push_back(p1);
    c.push_back(p2);
    c.push_back(p3);
    c.push_back(p4);
    contours.push_back(c);
    std::vector<bool> holes;
    holes.push_back(false);
    PolygonData* pd1 = new PolygonData(0, std::vector<float>(4,1), Eigen::Vector3f(0,0,-1), Eigen::Vector3f(1,0,0), Eigen::Vector3f(-1,-1,1), contours, holes);
    pd_.push_back(pd1);

    // Init pd2
    contours.clear();
    Eigen::Vector2f p5(0.5,0.5);
    c.push_back(p5);
    contours.push_back(c);
    //holes.clear();
    //holes.push_back(false);
    Eigen::Vector3f normal(0,0.1,-1);
    normal.normalize();
    PolygonData* pd2 = new PolygonData(1, std::vector<float>(4,1), normal, Eigen::Vector3f(1,0,0), Eigen::Vector3f(-0.5,-1,1), contours, holes);
    pd_.push_back(pd2);

    // Init pd3
    Eigen::Vector2f p6(0.8, 0.8), p7(0.8, 0.6), p8(0.6, 0.8);
    std::vector<Eigen::Vector2f> c_hole;
    c_hole.push_back(p6);
    c_hole.push_back(p7);
    c_hole.push_back(p8);
    contours.push_back(c_hole);
    holes.push_back(true);
    PolygonData* pd3 = new PolygonData(2, std::vector<float>(4,1), normal, Eigen::Vector3f(1,0,0), Eigen::Vector3f(-0.5,-1,1), contours, holes);
    pd_.push_back(pd3);

    pd_[0]->merge_candidates_.push_back(0);
    pd_[0]->merge_candidates_.push_back(1);
    pd_[0]->merge_candidates_.push_back(2);
    pd_[1]->merge_candidates_.push_back(0);
    pd_[1]->merge_candidates_.push_back(1);
    pd_[1]->merge_candidates_.push_back(2);
    pd_[2]->merge_candidates_.push_back(0);
    pd_[2]->merge_candidates_.push_back(1);
    pd_[2]->merge_candidates_.push_back(2);
  }

  std::vector<PolygonData*> pd_;

};

TEST_F(PolygonTest, creationWorks)
{
  for(unsigned int i = 0; i < pd_.size(); i++)
  {
    cob_3d_mapping::Polygon p(pd_[i]->id_, pd_[i]->normal_, pd_[i]->d_, pd_[i]->contours_3d_, pd_[i]->holes_, pd_[i]->color_);
    EXPECT_NEAR(pd_[i]->normal_.dot(p.normal_), 1.0, 0.001);
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
    }
  }
}

TEST_F(PolygonTest, mergeUnion)
{
  cob_3d_mapping::Polygon::Ptr p0(new cob_3d_mapping::Polygon(pd_[0]->id_, pd_[0]->normal_, pd_[0]->d_, pd_[0]->contours_3d_, pd_[0]->holes_, pd_[0]->color_));
  std::vector<cob_3d_mapping::Polygon::Ptr> p_vec;
  cob_3d_mapping::Polygon::Ptr p1(new cob_3d_mapping::Polygon(pd_[1]->id_, pd_[1]->normal_, pd_[1]->d_, pd_[1]->contours_3d_, pd_[1]->holes_, pd_[1]->color_));
  p_vec.push_back(p1);

  p0->merge(p_vec);
  EXPECT_EQ(p0->merged_, 2);
  EXPECT_NEAR(p0->normal_.dot((0.5 * (pd_[0]->normal_ + pd_[1]->normal_)).normalized()), 1, 0.001);

  /*std::vector<std::vector<Eigen::Vector3f> > c_3d = p0->getContours3D();
  std::cout << p0->contours_.size() << std::endl;
  for(unsigned int i = 0; i < c_3d[0].size(); i++)
  {
    std::cout << c_3d[0][i](0) << "," << c_3d[0][i](1) << "," << c_3d[0][i](2) << std::endl;
  }*/
}

TEST_F(PolygonTest, mergingWorks)
{
  std::vector<cob_3d_mapping::Polygon::Ptr> p_vec;
  for(unsigned int i = 0; i < pd_.size(); i++)
  {
    cob_3d_mapping::Polygon::Ptr p(new cob_3d_mapping::Polygon(pd_[i]->id_, pd_[i]->normal_, pd_[i]->d_, pd_[i]->contours_3d_, pd_[i]->holes_, pd_[i]->color_));
    p_vec.push_back(p);
  }
  //std::cout << pd_[0]->normal_.dot(pd_[1]->normal_);
  //std::cout << p_vec.size() << std::endl;
  std::vector<int> intersections;
  p_vec[0]->getMergeCandidates(p_vec, intersections);
  //for(unsigned int i=0; i<intersections.size(); i++) std::cout << "\t" << intersections[i] << std::endl;
  EXPECT_EQ(intersections.size(), pd_[0]->merge_candidates_.size());
  std::vector<cob_3d_mapping::Polygon::Ptr> merge_candidates;
  double sum_area = 0;
  for(unsigned int i = 0; i < intersections.size(); i++)
  {
    merge_candidates.push_back(p_vec[i]);
    sum_area += p_vec[i]->computeArea3d();
  }
  p_vec[0]->merge(merge_candidates);
  EXPECT_EQ(p_vec[0]->merged_, pd_[0]->merge_candidates_.size()+1);
  EXPECT_LE(p_vec[0]->computeArea3d(), sum_area);

  /*intersections.clear();
  p_vec[2]->getMergeCandidates(p_vec, intersections);
  EXPECT_EQ(intersections.size(), pd_[2]->merge_candidates_.size());
  merge_candidates.clear();
  for(unsigned int i = 0; i < intersections.size(); i++)
  {
    merge_candidates.push_back(p_vec[intersections[i]]);
  }
  p_vec[2]->merge(merge_candidates);
  EXPECT_EQ(p_vec[2]->merged_, pd_[2]->merge_candidates_.size()+1);
  std::cout << p_vec[2]->contours_.size() << std::endl;*/
  /*std::cout << p_vec[1]->contours_.size() << std::endl;
  for(unsigned int i = 0; i < p_vec[1]->contours_[0].size(); i++)
  {
    std::cout << p_vec[1]->contours_[0][i](0) << p_vec[1]->contours_[0][i](1) << std::endl;
  }*/
}

TEST_F(PolygonTest, transformWorks)
{
  Eigen::Affine3f t(Eigen::Affine3f::Identity());
  t.translate(Eigen::Vector3f(-1,1,2));
  t.rotate(Eigen::AngleAxisf(0.5, Eigen::Vector3f(1,1,1)));
  for(unsigned int i = 0; i < pd_.size(); i++)
  {
    cob_3d_mapping::Polygon::Ptr p(new cob_3d_mapping::Polygon(pd_[i]->id_, pd_[i]->normal_, pd_[i]->d_, pd_[i]->contours_3d_, pd_[i]->holes_, pd_[i]->color_));
    Eigen::Affine3f p0 = p->pose_;
    p->transform(t);
    p->transform(t.inverse());
    EXPECT_NEAR(p->normal_.dot(pd_[i]->normal_), 1, 0.001);
    EXPECT_NEAR(p->d_, pd_[i]->d_, 0.001);
    Eigen::Affine3f dt = p->pose_*p0.inverse();
    /*std::cout << p->pose_.matrix() << std::endl;
    std::cout << p0.matrix() << std::endl;
    std::cout << p->pose_.isApprox(p0) << std::endl;*/
    EXPECT_TRUE(p->pose_.isApprox(p0));
    /*EXPECT_NEAR(dt(0,0), 1, 0.001);
    EXPECT_NEAR(dt(1,1), 1, 0.001);
    EXPECT_NEAR(dt(2,2), 1, 0.001);
    EXPECT_NEAR(dt(0,1), 0, 0.001);
    EXPECT_NEAR(dt(0,2), 0, 0.001);
    EXPECT_NEAR(dt(0,3), 0, 0.001);
    EXPECT_NEAR(dt(1,0), 0, 0.001);
    EXPECT_NEAR(dt(1,2), 0, 0.001);
    EXPECT_NEAR(dt(1,3), 0, 0.001);
    EXPECT_NEAR(dt(2,0), 0, 0.001);
    EXPECT_NEAR(dt(2,1), 0, 0.001);
    EXPECT_NEAR(dt(2,3), 0, 0.001);*/
  }
}

TEST_F(PolygonTest, msgConversionWorks)
{
  for(unsigned int i = 0; i < pd_.size(); i++)
  {
    cob_3d_mapping::Polygon::Ptr p(new cob_3d_mapping::Polygon(pd_[i]->id_, pd_[i]->normal_, pd_[i]->d_, pd_[i]->contours_3d_, pd_[i]->holes_, pd_[i]->color_));
    cob_3d_mapping_msgs::Shape s;
    cob_3d_mapping::toROSMsg(*p, s);
    cob_3d_mapping::Polygon p1;
    cob_3d_mapping::fromROSMsg(s, p1);

    EXPECT_EQ(p->id_, p1.id_);
    EXPECT_EQ(p->contours_.size(), p1.contours_.size());
    EXPECT_EQ(p->holes_.size(), p1.holes_.size());
    EXPECT_NEAR(p->d_, p1.d_, 0.00001);
    EXPECT_NEAR((p->normal_ - p1.normal_).norm(), 0, 0.001);
    EXPECT_TRUE(p->pose_.isApprox(p1.pose_));
    EXPECT_NEAR(p->merge_weight_, p1.merge_weight_, 0.00001);
    EXPECT_NEAR((p->color_[0] - p1.color_[0]), 0, 0.001);
    EXPECT_NEAR((p->color_[1] - p1.color_[1]), 0, 0.001);
    EXPECT_NEAR((p->color_[2] - p1.color_[2]), 0, 0.001);
    EXPECT_NEAR((p->color_[3] - p1.color_[3]), 0, 0.001);
    for(unsigned int j = 0; j < p->contours_.size(); j++)
    {
      for(unsigned int k = 0; k < p->contours_[j].size(); k++)
      {
        EXPECT_NEAR((p->contours_[j][k] - p1.contours_[j][k]).norm(), 0, 0.001);
      }
      EXPECT_EQ(p->holes_[j], p1.holes_[j]);
    }
  }
}




int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
