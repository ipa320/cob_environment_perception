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
 * ROS package name: cob_3d_mapping_tools
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2012
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

template<typename PointT>
void cob_3d_mapping_tools::Gui::GuiTool<PointT>::init()
{
  getMinMaxZ<PointT>(cloud_, z_min_, z_max_);
  std::cout << "Depth range from " << z_min_ << "m to " << z_max_ <<"m" << std::endl;
  z_diff_ = z_max_ - z_min_;
  z_focus_ = (0.5f * z_diff_) * 1000.0;
  z_range_ = 100;

  cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Color", CV_WINDOW_AUTOSIZE);

  if(enable_z_min_max_)
  {
    char FocusBarName[50];
    char RangeBarName[50];
    sprintf(FocusBarName, "Focus in mm %d", (int)(z_diff_ * 1000.0));
    sprintf(RangeBarName, "Range in prec. %d", 100);
    //void (GuiTool::*cb_focus_trackbar)(int,void*) = &this->cb_focus_trackbar
    cv::createTrackbar(FocusBarName, "Depth", &this->z_focus_, (int)(z_diff_ * 1000.0), GuiTool::cb_depth_trackbar, this);
    cv::createTrackbar(RangeBarName, "Depth", &this->z_range_, 100, GuiTool::cb_depth_trackbar, this);
  }

  cv::setMouseCallback("Depth", GuiTool::cb_mouse, this);

  updateRGB();
  updateDepth();
}

template<typename PointT>
void cob_3d_mapping_tools::Gui::GuiTool<PointT>::setPointCloud(const PointCloudConstPtr& cloud)
{
  cloud_ = cloud;
}

template<typename PointT>
void cob_3d_mapping_tools::Gui::GuiTool<PointT>::updateRGB()
{
  boost::function<cv::Vec3b (int)> convRGB = PointConverterRGB<PointT>(cloud_);
  convertPointCloud2Cv<PointT>(cloud_, cvColor_, convRGB);
  cv::imshow("Color", this->cvColor_);
}

template<typename PointT>
void cob_3d_mapping_tools::Gui::GuiTool<PointT>::updateDepth()
{
  float from = std::max(z_min_, 0.001f * z_focus_ + z_min_ - 0.01f * z_range_ * z_diff_);
  float to = std::min(z_max_, 0.001f * z_focus_ + z_min_ + 0.01f * z_range_ * z_diff_);
  boost::function<cv::Vec3b (int)> convDepth = PointConverterDepth<PointT>(cloud_, from, to);
  convertPointCloud2Cv<PointT>(cloud_, cvDepth_, convDepth);
  cv::imshow("Depth", this->cvDepth_);
}

template<typename PointT>
void cob_3d_mapping_tools::Gui::GuiTool<PointT>::spin()
{
  this->init();
  cv::waitKey(0);
}

template<typename PointT>
void cob_3d_mapping_tools::Gui::GuiTool<PointT>::cb_depth_trackbar(int value, void* userdata)
{
  static_cast<GuiTool*>(userdata)->updateDepth();
}

template<typename PointT>
void cob_3d_mapping_tools::Gui::GuiTool<PointT>::cb_mouse(int event, int x, int y, int flags, void* userdata)
{
  switch (event)
  {
  case CV_EVENT_LBUTTONUP:
  {
    cv::Mat_<cv::Vec3b>& pix = static_cast<cv::Mat_<cv::Vec3b>&>(static_cast<GuiTool*>(userdata)->cvDepth_);
    std::cout << "you just hit pixel " << x << "," << y << " which has a color value of "
              << (int)pix(y,x)[2] <<","<< (int)pix(y,x)[1] <<","<< (int)pix(y,x)[0]
              << " and a depth value of " << static_cast<GuiTool*>(userdata)->cloud_->at(x,y).z << "m" << std::endl;
    break;
  }
  default:
    break;
  }
}

