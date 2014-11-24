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
 *  ROS package name: cob_3d_mapping_tools
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 10/2012
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

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/conversions.h>

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_features/organized_normal_estimation_omp.h"
#include "cob_3d_segmentation/depth_segmentation.h"

#include "cob_3d_mapping_tools/gui/impl/core.hpp"

class MainApp : public wxApp
{
  typedef pcl::PointXYZRGB PT;
  typedef pcl::PointCloud<PT> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> NormalCloud;
  typedef pcl::PointCloud<PointLabel> LabelCloud;
  typedef cob_3d_segmentation::PredefinedSegmentationTypes ST;

  typedef Gui::ResourceTypes::OrganizedPointCloud<PT> PC;
  typedef Gui::ViewTypes::Color Col;

  // gui stuff
  Gui::Core* c_;
  PointCloud::Ptr pc_;
  PointCloud::Ptr segmented_;
  NormalCloud::Ptr normals_;
  LabelCloud::Ptr labels_;

  Gui::Resource<PC>* r_rgb_;
  Gui::Resource<PC>* r_seg_;
  Gui::View<PC,Col>* v_rgb_;
  Gui::View<PC,Col>* v_seg_;

  // bag file stuff
  rosbag::Bag *p_bag;
  rosbag::View *p_view;
  ros::NodeHandle* nh;
  std::list<rosbag::View::iterator> timeline;
  std::list<rosbag::View::iterator>::iterator tl_it;

  cob_3d_features::OrganizedNormalEstimationOMP<PT, pcl::Normal, PointLabel> one_;
  cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> seg_;
  ST::Graph::Ptr graph_;

public:
  MainApp()
    : pc_(new PointCloud)
    , segmented_(new PointCloud)
    , normals_(new NormalCloud)
    , labels_(new LabelCloud)
    , graph_(new ST::Graph)
  {
    std::cout << "Create MainApp" << std::endl;
    c_ = Gui::Core::Get();
  }

  void compute()
  {
    one_.setInputCloud(pc_);
    one_.compute(*normals_);
    *segmented_ = *pc_;
    seg_.setInputCloud(pc_);
    seg_.performInitialSegmentation();
    graph_->clusters()->mapClusterColor(segmented_);

    ST::CH::ClusterPtr c_it;
    for (c_it = graph_->clusters()->begin(); c_it != graph_->clusters()->end(); ++c_it)
    {
      if (c_it->size() < 20) continue;
      if ( c_it->type == I_EDGE || c_it->type == I_NAN || c_it->type == I_BORDER) continue;
      graph_->clusters()->computeClusterComponents(c_it);
      //if ( (c_it->pca_point_values(1) / c_it->pca_point_values(2)) < 0.01f )
      if ( c_it->pca_point_values(1) < 0.00005f && c_it->pca_point_values(2) > 0.001)
      {
        std::cout << c_it->pca_point_values(1) << " / " << c_it->pca_point_values(2) << std::endl;
        for (ST::CH::ClusterType::iterator idx = c_it->begin(); idx != c_it->end(); ++idx)
        {
          pc_->points[*idx].rgba = 0xff0000;
        }
      }
    }
  }

  void mouseShowNext(wxMouseEvent& event, Gui::Resource<PC>* r)
  {
    if (event.LeftDClick() || event.RightDClick())
    {
      if(event.LeftDClick()) ++tl_it;
      else  --tl_it;

      if(tl_it == timeline.end()) { exit(0); }

      sensor_msgs::PointCloud2::ConstPtr last_msg = (*tl_it)->instantiate<sensor_msgs::PointCloud2>();
      std::cout << last_msg->header.stamp << std::endl;
      pcl::PCLPointCloud2 pc2;
      pcl_conversions::toPCL(*last_msg, pc2);
      pcl::fromPCLPointCloud2(pc2, *pc_);
      //pcl::fromROSMsg<PT>(*last_msg, *pc_);
      compute();
      r_rgb_->resourceChanged();
      r_seg_->resourceChanged();
    }
  }

  void keyShowNext(wxKeyEvent& event, Gui::Resource<PC>* r)
  {
    // 70 -> "f", 66 -> "b"
    //std::cout << event.GetKeyCode() << std::endl;
    if (event.GetKeyCode() == 70 || event.GetKeyCode() == 66)
    {
      if(event.GetKeyCode() == 70) ++tl_it;
      else  --tl_it;

      if(tl_it == timeline.end()) { exit(0); }

      sensor_msgs::PointCloud2::ConstPtr last_msg = (*tl_it)->instantiate<sensor_msgs::PointCloud2>();
      std::cout << last_msg->header.stamp << std::endl;
      pcl::fromROSMsg<PT>(*last_msg, *pc_);
      compute();
      r_rgb_->resourceChanged();
      r_seg_->resourceChanged();
    }
  }

  bool OnInit()
  {
    std::cout << "Init" << std::endl;
    if (this->argc != 3) { std::cout << " please provide path to bag file and a topic name!" << std::endl; exit(0); }
    std::string bag_file(wxString(this->argv[1]).mb_str());
    std::string topic(wxString(this->argv[2]).mb_str());

    char const* av = "bag_gui";
    char** av_ptr = (char**)&av;
    ros::init(argc, av_ptr, "bag_gui");
    nh = new ros::NodeHandle();
    one_.setOutputLabels(labels_);
    one_.setPixelSearchRadius(8,2,2);
    one_.setSkipDistantPointThreshold(8);

    seg_.setNormalCloudIn(normals_);
    seg_.setLabelCloudInOut(labels_);
    seg_.setClusterGraphOut(graph_);


    p_bag = new rosbag::Bag();
    p_view = new rosbag::View();
    try { p_bag->open(bag_file, rosbag::bagmode::Read); }
    catch (rosbag::BagException) { std::cerr << "Error opening file " << bag_file << std::endl; return false; }
    p_view->addQuery(*p_bag, rosbag::TopicQuery(topic));

    rosbag::View::iterator vit = p_view->begin();
    while(vit!=p_view->end()) { timeline.push_back(vit); ++vit; }
    tl_it = timeline.begin();

    sensor_msgs::PointCloud2::ConstPtr last_msg = (*tl_it)->instantiate<sensor_msgs::PointCloud2>();
    pcl::fromROSMsg<PT>(*last_msg, *pc_);
    compute();

    r_rgb_ = Gui::Core::rMan()->create<PC>("r_rgb", pc_);
    r_seg_ = Gui::Core::rMan()->create<PC>("r_seg", segmented_);
    v_rgb_ = r_rgb_->createView<Col>("v_rgb");
    v_seg_ = r_seg_->createView<Col>("v_seg");

    boost::function<void (wxMouseEvent&, Gui::Resource<PC>*)> f = boost::bind(&MainApp::mouseShowNext, this, _1, _2);
    v_seg_ ->registerMouseCallback(f);
    boost::function<void (wxKeyEvent&, Gui::Resource<PC>*)> k = boost::bind(&MainApp::keyShowNext, this, _1, _2);
    v_seg_ ->registerKeyCallback(k);

    v_seg_->show();
    v_rgb_->show();

    std::cout<<"Done Init"<<std::endl;
    return true;
  }

  int OnExit()
  {
    std::cout << "I shut myself down!" << std::endl;
    if(p_bag) { p_bag->close(); delete p_bag; }
    delete p_view;
    return 0;
  }
};

IMPLEMENT_APP(MainApp)
