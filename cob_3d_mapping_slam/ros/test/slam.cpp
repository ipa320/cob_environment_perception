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
/*
 * slam.cpp
 *
 *  Created on: 27.05.2012
 *      Author: josh
 */



#define DEBUG_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>

#include <cob_3d_mapping_slam/curved_polygons/debug_interface.h>

#include <cob_3d_mapping_slam/slam/context.h>
#include <cob_3d_mapping_slam/slam/node.h>

#include <cob_3d_mapping_slam/dof/tflink.h>
#include <cob_3d_mapping_slam/dof/dof_uncertainty.h>
#include <cob_3d_mapping_slam/dof/dof_variance.h>

#include <cob_3d_mapping_slam/slam/dummy/objctxt.h>
#include <cob_3d_mapping_slam/slam/dummy/registration.h>
#include <cob_3d_mapping_slam/slam/dummy/key.h>
#include <cob_3d_mapping_slam/slam/dummy/robot.h>

#include <cob_3d_mapping_slam/curved_polygons/objctxt.h>
#include <cob_3d_mapping_slam/curved_polygons/key.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/CurvedPolygon_Array.h>
#include "gnuplot_i.hpp"
#include <gtest/gtest.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#define CYCLES 100
static const char *BAGFILES[][512]={
                                    //Test 1
                                    {//"test/cp_rgbd_dataset_freiburg2_desk_validation.bag",
                                     //"test/cp_rotate_real_cups4.bag",
                                     //"test/cp2_rot_real_cups1.bag",
                                     //"test/cp2_static_room.bag",
                                     //"test/cp3_rot_real_cups.bag",
                                     //"test/cp3_rgbd_dataset_freiburg2_dishes.bag",
                                     //"test/cp3_rgbd_dataset_freiburg2_rpy.bag",
                                     //"test/cp4_moving.bag",
                                     //"test/cp4_room.bag",
                                     "test/cp4_rgbd_dataset_freiburg2_rpy2.bag",
                                     //"test/cp3_rgbd_dataset_freiburg2_xyz.bag",
                                     //"test/cp3_rgbd_dataset_freiburg1_plant.bag",
                                     //"test/cp2_rgbd_dataset_freiburg2_dishes.bag",
                                     //"test/cp2_freiburg.bag",
                                     //"test/2012-06-14-09-04-44.bag",
                                     //"test/cp_rot_sim_kitchen.bag",
                                     //"test/cp_tr_real_cups.bag",
                                     //"test/cp_rotate_real_wall.bag",
                                     (const char*)0}
};

static const char *GROUNDTRUTH[][512]={
                                       {
                                        //"test/rgbd_dataset_freiburg2_dishes-groundtruth.txt",
                                        "test/rgbd_dataset_freiburg2_rpy-groundtruth.txt",
                                        //"test/rgbd_dataset_freiburg2_xyz-groundtruth.txt",
                                        //"test/rgbd_dataset_freiburg2_dishes-groundtruth.txt",
                                        0
                                       }
};


template<typename Matrix>
float MATRIX_DISTANCE(const Matrix &a, const Matrix &b, const float thr=0.05) {
  Matrix c=a-b;
  float d=c.norm();

  if(d>thr) {
    std::cout<<"A\n"<<a<<"\n";
    std::cout<<"B\n"<<b<<"\n";
  }

  EXPECT_NEAR(d,0,thr);
  return d;
}


TEST(Slam,merging)
{

  cob_3d_mapping_msgs::feature ft;
  cob_3d_mapping_msgs::polyline_point pt;
  cob_3d_mapping_msgs::CurvedPolygon s;

  {
    s.weight = 100;
    s.parameter[0] = 2;
    s.parameter[1] = 0;
    s.parameter[2] = -0.01;
    s.parameter[3] = 0;
    s.parameter[4] = -0.01;
    s.parameter[5] = 0;
    s.ID = 0;
    s.energy = "";
    s.features.clear();
    pt.edge_prob = 1;

    pt.x = -1;
    pt.y = -1;
    s.polyline.push_back(pt);
    pt.x = 1;
    pt.y = -1;
    s.polyline.push_back(pt);
    pt.x = 1;
    pt.y = 1;
    s.polyline.push_back(pt);
    pt.x = -1;
    pt.y = 1;
    s.polyline.push_back(pt);

    ft.ID = 1; //nearest point
    ft.x = 0;
    ft.y = 0;
    ft.z = 2;
    s.features.push_back(ft);

    ft.ID = 2; //form
    ft.x = 0;
    ft.y = 0;
    ft.z = 0;
    s.features.push_back(ft);

    ft.ID = 3; //nearest point
    ft.x = 0;
    ft.y = 0;
    ft.z = 2;
    s.features.push_back(ft);

    ft.ID = 4; //curv
    ft.x = 1;
    ft.y = 0;
    ft.z = 0;
    s.features.push_back(ft);

    ft.ID = 5; //curv
    ft.x = 1;
    ft.y = 0;
    ft.z = 0;
    s.features.push_back(ft);
  }
  Slam_CurvedPolygon::ex_curved_polygon xp1 = s;
  EXPECT_FALSE( xp1.invalid() );

  {
    s.parameter[0] = 2;
    s.parameter[1] = 0;
    s.parameter[2] = 0;
    s.parameter[3] = 0;
    s.parameter[4] = 0;
    s.parameter[5] = 0;

    s.ID = 1;
    s.features.clear();

    for(size_t i=0; i<s.polyline.size(); i++)
    {
      s.polyline[i].x += 0.5;
    }

    ft.ID = 1; //nearest point
    ft.x = 0.5;
    ft.y = 0;
    ft.z = 2;
    s.features.push_back(ft);

    ft.ID = 2; //form
    ft.x = 0;
    ft.y = 0;
    ft.z = 0;
    s.features.push_back(ft);

    ft.ID = 3; //nearest point
    ft.x = 0.5;
    ft.y = 0;
    ft.z = 2;
    s.features.push_back(ft);

    ft.ID = 4; //curv
    ft.x = 1;
    ft.y = 0;
    ft.z = 0;
    s.features.push_back(ft);

    ft.ID = 5; //curv
    ft.x = 1;
    ft.y = 0;
    ft.z = 0;
    s.features.push_back(ft);
  }

  Slam_CurvedPolygon::ex_curved_polygon xp2 = s;
  EXPECT_FALSE( xp2.invalid() );

  Eigen::Vector3f t;
  t(0) = 0;
  t(1) = 0;
  t(2) = 0;
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  //  R(0,0) = 0; R(0,2) = 1;
  //  R(2,2) = 0; R(2,0) = 1;
  xp2.transform(R, t, 0.1,0.1);

  std::cout<<xp1.getFeatures()[5].v_<<"\n\n";
  std::cout<<xp1.getFeatures()[6].v_<<"\n\n";
  std::cout<<xp1.getFeatures()[7].v_<<"\n\n";
  std::cout<<xp1.getFeatures()[8].v_<<"\n\n";
  std::cout<<"----------\n";
  std::cout<<xp2.getFeatures()[5].v_<<"\n\n";
  std::cout<<xp2.getFeatures()[6].v_<<"\n\n";
  std::cout<<xp2.getFeatures()[7].v_<<"\n\n";
  std::cout<<xp2.getFeatures()[8].v_<<"\n\n";
  std::cout<<"----------\n";

  std::cout<<"can "<<(!xp1.canMerge(xp2)?"not ":" ")<<"merge\n";

  xp1 += xp2;

  std::cout<<xp1.getFeatures()[5].v_<<"\n\n";
  std::cout<<xp1.getFeatures()[6].v_<<"\n\n";
  std::cout<<xp1.getFeatures()[7].v_<<"\n\n";
  std::cout<<xp1.getFeatures()[8].v_<<"\n\n";

  Eigen::Vector3f r;
  r(2) = 2;

  r(0) = -1.5;
  r(1) = 1;
  EXPECT_NEAR( (xp1.getFeatures()[5].v_-r).norm(), 0, 0.05 );
  r(0) = 1;
  r(1) = 1;
  EXPECT_NEAR( (xp1.getFeatures()[6].v_-r).norm(), 0, 0.05 );
  r(0) = 1;
  r(1) = -1;
  EXPECT_NEAR( (xp1.getFeatures()[7].v_-r).norm(), 0, 0.05 );
  r(0) = -1.5;
  r(1) = -1;
  EXPECT_NEAR( (xp1.getFeatures()[8].v_-r).norm(), 0, 0.05 );
}


struct SOdomotry_Data
{
  double timestamp;
  Eigen::Vector3f t;
  Eigen::Quaternionf q;
};

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/cloud_viewer.h>
TEST(Slam,bag_run)
//void t3()
{
  pcl::visualization::CloudViewer viewer("abc");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

  int bg_file=0;
  while(BAGFILES[0][bg_file]) {

    std::vector<SOdomotry_Data> odos;
    if(GROUNDTRUTH[0][bg_file]) {
      std::ifstream infile(GROUNDTRUTH[0][bg_file]);

      std::string line;
      while (std::getline(infile, line))
      {
        if(line.size()<1 || line[0]=='#') continue;

        std::istringstream iss(line);
        SOdomotry_Data odo;
        if (!(iss >> odo.timestamp >> odo.t(0) >> odo.t(1) >> odo.t(2) >> odo.q.x() >> odo.q.y() >> odo.q.z() >> odo.q.w())) {
          ROS_ERROR("parsing groundtruth");
          ROS_ASSERT(0);
          break;
        } // error

        odos.push_back(odo);
      }

    }

    rosbag::Bag bag, bag_out;
    try {
      //read file
      bag.    open(BAGFILES[0][bg_file],                         rosbag::bagmode::Read);
      bag_out.open(std::string(BAGFILES[0][bg_file])+".odo.bag", rosbag::bagmode::Write);

      //setup topics
      std::vector<std::string> topics;
      topics.push_back(std::string("curved_polygons"));
      topics.push_back(std::string("/curved_polygons"));
      topics.push_back(std::string("shapes_array"));
      topics.push_back(std::string("/shapes_array"));

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      visualization_msgs::Marker marker_text, marker_points, marker_planes, marker_cor1, marker_cor2, marker_del, marker_map;
      marker_text.header.frame_id = "/openni_rgb_frame";
      marker_text.pose.position.x = 0;
      marker_text.pose.position.y = 0;
      marker_text.pose.position.z = 0;
      marker_text.pose.orientation.x = 0.0;
      marker_text.pose.orientation.y = 0.0;
      marker_text.pose.orientation.z = 0.0;
      marker_text.pose.orientation.w = 1.0;
      marker_text.action = visualization_msgs::Marker::ADD;
      marker_text.type = visualization_msgs::Marker::LINE_LIST;
      marker_text.scale.x = marker_text.scale.y = marker_text.scale.z = 0.01;
      marker_text.color.r = marker_text.color.g = marker_text.color.b =  marker_text.color.a = 1;

      marker_map = marker_del = marker_cor1 =  marker_cor2 = marker_points = marker_planes = marker_text;
      marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker_text.scale.x = marker_text.scale.y = marker_text.scale.z = 0.35;
      marker_points.color.g = marker_points.color.b =  0;
      marker_planes.color.g = marker_planes.color.r =  0;
      marker_cor2.color.b = marker_cor2.color.r =  0;
      marker_del.action = visualization_msgs::Marker::DELETE;
      marker_map.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker_map.scale.x = marker_map.scale.y = marker_map.scale.z = 1;

      marker_text.id = 0;
      marker_points.id = 1;
      marker_planes.id = 2;
      marker_cor1.id = 3;
      marker_cor2.id = 4;
      marker_map.id = 5;

      //setup slam
      typedef DOF6::DOF6_Source<DOF6::TFLinkvf,DOF6::DOF6_Uncertainty<Dummy::RobotParameters,float> > DOF6;
      typedef Slam::Node<Slam_CurvedPolygon::OBJCTXT<DOF6> > Node;

      Slam::Context<Slam_CurvedPolygon::KEY<DOF6>, Node> ctxt(.60,.30);

      Eigen::Vector3f last_tr = Eigen::Vector3f::Zero();
      Eigen::Matrix3f last_rot= Eigen::Matrix3f::Identity();

      Eigen::Vector3f last_odo_tr = Eigen::Vector3f::Zero();
      Eigen::Vector3f first_odo_tr = Eigen::Vector3f::Zero();
      Eigen::Matrix3f first_odo_rot= Eigen::Matrix3f::Identity();

      //load data
      int ctr=0;
      ros::Time last, start;
      bool first=true;
      std::vector<cob_3d_mapping_msgs::ShapeArray> memory_sa;
      BOOST_FOREACH(rosbag::MessageInstance const m, view)
      {

        cob_3d_mapping_msgs::ShapeArray::ConstPtr sa = m.instantiate<cob_3d_mapping_msgs::ShapeArray>();
        if (sa != NULL)
        {
          if(sa->header.stamp.toSec()<1311867724.916822)
            continue;
          memory_sa.push_back(*sa);
          continue;
        }

        geometry_msgs::Point line_p, line_p0;
        line_p0.x = line_p0.y = line_p0.z = 0;

        cob_3d_mapping_msgs::CurvedPolygon_Array::ConstPtr s = m.instantiate<cob_3d_mapping_msgs::CurvedPolygon_Array>();
        if (s != NULL)
        {
          if(s->header.stamp.toSec()<1311867724.916822)
            continue;

          //          if(!(std::abs((s->stamp-start).toSec()-0)<0.001 || first || std::abs((s->stamp-start).toSec()-3.15)<0.001 || std::abs((s->stamp-start).toSec()-3.7)<0.1))
          //            continue;

          if(last!=s->header.stamp) {
            marker_cor1.header.stamp = marker_cor2.header.stamp = marker_points.header.stamp = marker_planes.header.stamp = marker_text.header.stamp = s->header.stamp;

            ROS_INFO("new frame");

            last=s->header.stamp;
            Debug::Interface::get().setTime((last-start).toSec());
            if(first) {
              first=false;
              start = s->header.stamp;
            }
            else
              ctxt.finishFrame();

            ROS_INFO("timestamp %f (%d)", (last-start).toSec(), ctr);
            ctr++;

            pcl::PointXYZRGB p;
            Eigen::Vector3f from,to,temp;
            while(Debug::Interface::get().getArrow(from,to,p.r,p.g,p.b))
            {
              line_p.x = from(0);
              line_p.y = from(1);
              line_p.z = from(2);
              p.r>100?marker_cor1.points.push_back(line_p):marker_cor2.points.push_back(line_p);
              line_p.x = to(0);
              line_p.y = to(1);
              line_p.z = to(2);
              p.r>100?marker_cor1.points.push_back(line_p):marker_cor2.points.push_back(line_p);
              temp = to-from;
              //              from -= temp*0.1;
              //              temp *= 1.2;
              for(float ms=0; ms<=1; ms+=0.01)
              {
                p.x=ms*temp(0)+from(0);
                p.y=ms*temp(1)+from(1);
                p.z=ms*temp(2)+from(2);
                if(p.getVector3fMap().squaredNorm()<100)
                  rgb->push_back(p);
              }
            }

            //OUTPUT--------------

            //check path
            Eigen::Matrix3f tmp_rot = Eigen::Matrix3f::Identity();
            Eigen::Matrix3f tmp_rot2 = Eigen::Matrix3f::Identity();
            Eigen::Vector3f tmp_tr  = Eigen::Vector3f::Zero();
            const Slam::SWAY<Node> *n = &ctxt.getPath().getLocal();
            while(n)
            {

              tmp_tr = tmp_rot2*tmp_tr + n->link_.getTranslation();
              tmp_rot = ((Eigen::Matrix3f)n->link_.getRotation())*tmp_rot;
              tmp_rot2= ((Eigen::Matrix3f)n->link_.getRotation());

              std::cout<<"con\n";
              n = n->node_->getConnections().size()?&n->node_->getConnections()[0]:NULL;
            }
            std::cout<<"ROT1\n"<<(::DOF6::EulerAnglesf)tmp_rot<<"\n";
            std::cout<<"TR1\n"<<tmp_tr<<"\n";

            Eigen::Quaternionf quat((Eigen::Matrix3f)tmp_rot.inverse());

            nav_msgs::Odometry odo;
            odo.header.frame_id = "/openni_rgb_frame";
            odo.header.stamp = last;
            odo.pose.pose.position.x = -tmp_tr(0);
            odo.pose.pose.position.y = -tmp_tr(1);
            odo.pose.pose.position.z = -tmp_tr(2);
            odo.pose.pose.orientation.x = quat.x();
            odo.pose.pose.orientation.y = quat.y();
            odo.pose.pose.orientation.z = quat.z();
            odo.pose.pose.orientation.w = quat.w();

            bag_out.write("odometry", last, odo);

            while(odos.size()>0 && odos.front().timestamp<last.toSec())
              odos.erase(odos.begin());
            if(odos.size()>0)
            {
              if(start == s->header.stamp)
              {
                first_odo_tr  = odos.front().t;
                first_odo_rot = odos.front().q;
              }

              odos.front().t -= first_odo_tr;
              odos.front().t = first_odo_rot.inverse()*odos.front().t;
              odos.front().q = first_odo_rot.inverse()*odos.front().q;

              std::cout<<"ROT2\n"<<(::DOF6::EulerAnglesf)tmp_rot<<"\n";
              std::cout<<"TR2\n"<<tmp_tr<<"\n";

              std::cout<<"ODO ROT2\n"<<(::DOF6::EulerAnglesf)odos.front().q.toRotationMatrix().inverse()<<"\n";
              std::cout<<"ODO TR2\n"<<-odos.front().t<<"\n";

              odo.header.stamp = ros::Time(odos.front().timestamp);
              odo.pose.pose.position.x = odos.front().t(0);
              odo.pose.pose.position.y = odos.front().t(1);
              odo.pose.pose.position.z = odos.front().t(2);

              ROS_INFO("odometry tr jump with length %f", (odos.front().t-last_odo_tr).norm());
              last_odo_tr = odos.front().t;
              odo.pose.pose.orientation.x = odos.front().q.x();
              odo.pose.pose.orientation.y = odos.front().q.y();
              odo.pose.pose.orientation.z = odos.front().q.z();
              odo.pose.pose.orientation.w = odos.front().q.w();

              bag_out.write("groundtruth", ros::Time(odos.front().timestamp), odo);
            }

            tmp_rot = Eigen::Matrix3f::Identity();
            tmp_rot2 = Eigen::Matrix3f::Identity();
            tmp_tr  = Eigen::Vector3f::Zero();
            n = &ctxt.getPath().getLocal();
            while(n)
            {

              tmp_tr = tmp_rot2*tmp_tr + n->link_.getTranslation();
              tmp_rot = ((Eigen::Matrix3f)n->link_.getRotation())*tmp_rot;
              tmp_rot2= ((Eigen::Matrix3f)n->link_.getRotation());

              for(size_t i=0; i<n->node_->getContext().getObjs().size(); i++)
              {
                //                if( n->node_->getContext().getObjs()[i]->getUsedCounter()<7)
                //                  continue;

                std::vector<Eigen::Vector3f> tris;
                n->node_->getContext().getObjs()[i]->getData().getTriangles(tris);
                ROS_ASSERT(tris.size()%3==0);

                ::std_msgs::ColorRGBA col;
                unsigned int rnd=(((i+1)*11)<<12)+(i+3)*i*7;//rand();
                ROS_ASSERT(n->node_->getContext().getObjs()[i]->getUsedCounter() <= n->node_->getContext().getObjs()[i]->getCreationCounter());
                col.a = std::log(n->node_->getContext().getObjs()[i]->getUsedCounter())/(std::log(n->node_->getContext().getObjs()[i]->getCreationCounter())+0.1f);
                if(!pcl_isfinite(col.a))
                {
                  col.a = 1.f;
                  ROS_ERROR("color not finite %d %d",n->node_->getContext().getObjs()[i]->getUsedCounter(),n->node_->getContext().getObjs()[i]->getCreationCounter());
                }
                col.r = ((rnd>>0)&0xff)/255.f;
                col.g = ((rnd>>8)&0xff)/255.f;
                col.b = ((rnd>>16)%0xff)/255.f;
                for(size_t j=0; j<tris.size(); j++)
                {
                  //Eigen::Vector3f v=((Eigen::Matrix3f)n->link_.getRotation())*tris[j]+n->link_.getTranslation();
                  //Eigen::Vector3f v=tmp_rot.inverse()*(tris[j]-tmp_tr);
                  Eigen::Vector3f v=tmp_rot*tris[j]+tmp_tr;
                  line_p.x = v(0);
                  line_p.y = v(1);
                  line_p.z = v(2);
                  marker_map.points.push_back(line_p);
                  marker_map.colors.push_back(col);
                }
              }

              n = n->node_->getConnections().size()?&n->node_->getConnections()[0]:NULL;
              //break;
            }

            while(memory_sa.size()>0 && memory_sa.front().header.stamp<=last)
            {
              bag_out.write("shapes_array", last, memory_sa.front());
              ROS_INFO("shapes_array %d",memory_sa.front().shapes.size());
              memory_sa.erase(memory_sa.begin());
            }
            for(int i=0; i<6; i++)
            {
              marker_del.id = i;
              if(i<5) bag_out.write("/markers", last, marker_del);
              else bag_out.write("/map", last, marker_del);
            }
            bag_out.write("/markers", last, marker_points);
            bag_out.write("/markers", last, marker_planes);
            bag_out.write("/markers", last, marker_cor1);
            bag_out.write("/markers", last, marker_cor2);
            bag_out.write("/map", last, marker_map);
            marker_points.points.clear();
            marker_planes.points.clear();
            marker_cor1.points.clear();
            marker_cor2.points.clear();
            marker_map.points.clear();
            marker_map.colors.clear();
            //OUTPUT-------------------


            if((last_tr-tmp_tr).norm()>0.2 || ((::DOF6::EulerAnglesf)(last_rot.inverse()*tmp_rot)).getVector().squaredNorm()>0.15f*0.15f) {
              ROS_INFO("BIG JUMP");
              ROS_ERROR("BIG JUMP");

              marker_text.text = "BIG JUMP";
              bag_out.write("/markers", last, marker_text);
              //ASSERT_TRUE(false);
              //while(1) getchar();
            }
            last_tr=tmp_tr;
            last_rot=tmp_rot;

            rgb->width=1;
            rgb->height=rgb->size();
            if(rgb->size())
            {
              //              Eigen::Quaternionf q((Eigen::Matrix3f)tmp_rot.inverse());
              //              pcl::transformPointCloud(*rgb,*rgb, -tmp_tr, q);
              viewer.showCloud(rgb);
              std::cerr<<"press any key\n";
              //              if((last-start).toSec()>31.8)
              if(getchar()=='q')
              {
                bag_out.close();
                return;
              }
              rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            }

            ctxt.startFrame(s->header.stamp.toSec());
          }

          for(size_t k=0; k<s->polygons.size(); k++)
          {

            if(s->polygons[k].weight<50) continue;

            Slam_CurvedPolygon::ex_curved_polygon xp = s->polygons[k];

            if(xp.invalid())
              continue;

            //        if(    std::abs(xp.getNearestPoint().norm()-1.2)>0.2
            //            || std::abs(xp.getNearestPoint()(0))>0.4
            //            || std::abs(xp.getNearestPoint()(1))>0.4
            //        ) continue;

            //                        if(s->polyline.size()<10)
            //                          continue;
            //                if(xp.getEnergy()<5)
            //                  continue;
            //        if(!xp.isPlane())
            //          continue;
            //        if(xp.getNearestPoint().norm()>4)
            //          continue;

            ctxt += s->polygons[k];

            line_p.x = xp.getNearestPoint()(0);
            line_p.y = xp.getNearestPoint()(1);
            line_p.z = xp.getNearestPoint()(2);
            if(xp.isPlane())
            {
              marker_planes.points.push_back(line_p0);
              marker_planes.points.push_back(line_p);
            }
            else
            {
              marker_points.points.push_back(line_p0);
              marker_points.points.push_back(line_p);
            }

            pcl::PointXYZRGB p;
            std::cerr<<"param: "
                <<s->polygons[k].parameter[0]<<" "
                <<s->polygons[k].parameter[1]<<" "
                <<s->polygons[k].parameter[2]<<" "
                <<s->polygons[k].parameter[3]<<" "
                <<s->polygons[k].parameter[4]<<" "
                <<s->polygons[k].parameter[5]<<"\n";
            for(float ms=0; ms<=1; ms+=0.01)
            {
              p.r=p.g=p.b=0;
              if(!xp.isPlane())
                p.r=255;
              else
                p.b=255;
              p.x=ms*xp.getNearestPoint()(0);
              p.y=ms*xp.getNearestPoint()(1);
              p.z=ms*xp.getNearestPoint()(2);
              if(p.getVector3fMap().squaredNorm()<100)
                rgb->push_back(p);
            }

            Eigen::Vector3f temp;
            temp=xp.getFeatures()[1].v_;
            temp.normalize();
            temp*=0.1;
            for(float ms=0; ms<=1; ms+=0.01)
            {
              p.r=p.g=p.b=0;
              p.g=255;
              p.b=255;
              p.x=ms*temp(0)+xp.getNearestPoint()(0);
              p.y=ms*temp(1)+xp.getNearestPoint()(1);
              p.z=ms*temp(2)+xp.getNearestPoint()(2);
              if(p.getVector3fMap().squaredNorm()<100)
                rgb->push_back(p);
            }
            temp=xp.getFeatures()[3].v_;
            temp.normalize();
            temp*=0.1;
            for(float ms=0; ms<=1; ms+=0.01)
            {
              p.r=p.g=p.b=0;
              p.r=255;
              p.b=255;
              p.x=ms*temp(0)+xp.getNearestPoint()(0);
              p.y=ms*temp(1)+xp.getNearestPoint()(1);
              p.z=ms*temp(2)+xp.getNearestPoint()(2);
              if(p.getVector3fMap().squaredNorm()<100)
                rgb->push_back(p);
            }
            temp=xp.getFeatures()[4].v_;
            temp.normalize();
            temp*=0.1;
            for(float ms=0; ms<=1; ms+=0.01)
            {
              p.r=p.g=p.b=0;
              p.r=255;
              p.g=255;
              p.x=ms*temp(0)+xp.getNearestPoint()(0);
              p.y=ms*temp(1)+xp.getNearestPoint()(1);
              p.z=ms*temp(2)+xp.getNearestPoint()(2);
              if(p.getVector3fMap().squaredNorm()<100)
                rgb->push_back(p);
            }
          }
        }

      }

      ctxt.finishFrame();

      bag.close();
    }
    catch(...) {
      bag_out.close();
      std::cout<<"failed to load: "<<BAGFILES[0][bg_file]<<"\n";
      ASSERT_TRUE(false);
    }

    bg_file++;
  }
}

int main(int argc, char **argv){
  ros::Time::init();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
