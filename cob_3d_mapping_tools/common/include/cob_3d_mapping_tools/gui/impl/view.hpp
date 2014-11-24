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
 * \date Date of creation: 08/2012
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

#ifndef COB_3D_MAPPING_TOOLS_GUIE_VIEW_HPP_
#define COB_3D_MAPPING_TOOLS_GUIE_VIEW_HPP_

#include "cob_3d_mapping_tools/gui/view.h"

  /* -----------------....-------*/
 /* --------- ViewBase ---------*/
/* ----------------------------*/
Gui::ViewBase::~ViewBase()
{ r_base_ptr->releaseView(name_); std::cout << "Base View destroyed" << std::endl; }


  /* ------------------------*/
 /* --------- View ---------*/
/* ------------------------*/
template<typename RT, typename VT>
boost::signals2::connection Gui::View<RT,VT>::registerMouseCallback(boost::function<void (wxMouseEvent&, Resource<RT>*)> f)
{ return mouse_sig_.connect(f); }

template<typename RT, typename VT>
boost::signals2::connection Gui::View<RT,VT>::registerKeyCallback(boost::function<void (wxKeyEvent&, Resource<RT>*)> f)
{ return key_sig_.connect(f); }

template<typename RT, typename VT>
void Gui::View<RT,VT>::show(Gui::ViewTypes::View2D)
{ Core::wMan()->create(static_cast<Gui::ImageView<RT,VT>*>(this),std::string(RT::STR + VT::STR)); }

template<typename RT, typename VT>
void Gui::View<RT,VT>::refresh(Gui::ViewTypes::View2D)
{ static_cast<Gui::ImageView<RT,VT>*>(this)->render(); }


 /* --------- Res: Image -|- View: Color ---------*/
/* ----------------------------------------------*/
template<typename RT, typename VT>
void Gui::View<RT,VT>::reloadData(ResourceTypes::Image, ViewTypes::Color)
{
  typedef Gui::ImageView<Gui::ResourceTypes::Image, Gui::ViewTypes::Color> IV;

  int w = this->r_ptr->getData()->cols;
  int h = this->r_ptr->getData()->rows;
  static_cast<IV*>(this)->bmp_.reset(new wxBitmap(w,h));
  wxNativePixelData data(*(static_cast<IV*>(this)->bmp_));
  wxNativePixelData::Iterator pwx(data);
  for(int row = 0; row < h; ++row)
  {
    for(int col = 0; col < w; ++col, ++pwx)
    {
      cv::Vec3b& pcv = (*this->r_ptr->getData())(row,col);
      pwx.Blue() = pcv[0];
      pwx.Green() = pcv[1];
      pwx.Red() = pcv[2];
    }
  }
}


 /* --------- Res: Organized PC -|- View: Color ---------*/
/* -----------------------------------------------------*/
template<typename RT, typename VT>
template<typename PT>
void Gui::View<RT,VT>::reloadData(ResourceTypes::OrganizedPointCloud<PT>, ViewTypes::Color)
{
  typedef Gui::ImageView<Gui::ResourceTypes::OrganizedPointCloud<PT>, Gui::ViewTypes::Color> IV;

  int w = this->r_ptr->getData()->width;
  int h = this->r_ptr->getData()->height;
  static_cast<IV*>(this)->bmp_.reset(new wxBitmap(w,h));
  wxNativePixelData data(*(static_cast<IV*>(this)->bmp_));
  wxNativePixelData::Iterator pwx(data);
  for(int row = 0; row < h; ++row)
  {
    for(int col = 0; col < w; ++col, ++pwx)
    {
      PT* ppc = &(*this->r_ptr->getData())[col + row * w];
      pwx.Red() = ppc->r;
      pwx.Green() = ppc->g;
      pwx.Blue() = ppc->b;
    }
  }
}


/* --------- Res: Organized PC -|- View: Depth ---------*/
/* -----------------------------------------------------*/
template<typename RT, typename VT>
template<typename PT>
void Gui::View<RT,VT>::reloadData(ResourceTypes::OrganizedPointCloud<PT>, ViewTypes::Depth_Z)
{
  typedef Gui::ImageView<Gui::ResourceTypes::OrganizedPointCloud<PT>, Gui::ViewTypes::Depth_Z> IV;
  int w = this->r_ptr->getData()->width;
  int h = this->r_ptr->getData()->height;
  float z_min, z_max;
  Tools::getMinMaxZ<PT>(this->r_ptr->getData(), z_min, z_max);

  static_cast<IV*>(this)->bmp_.reset(new wxBitmap(w,h));
  wxNativePixelData data(*(static_cast<IV*>(this)->bmp_));
  wxNativePixelData::Iterator pwx(data);
  cv::Vec3b bgr;
  for(int row = 0; row < h; ++row)
  {
    for(int col = 0; col < w; ++col, ++pwx)
    {
      float z = (*this->r_ptr->getData())[col + row * w].z;
      if(z != z)
      {
        pwx.Red() = 0;
        pwx.Green() = 0;
        pwx.Blue() = 0;
      }
      else
      {
        Tools::getGradientColor(z, z_min, z_max, bgr);
        pwx.Red() = bgr[2];
        pwx.Green() = bgr[1];
        pwx.Blue() = bgr[0];
      }
    }
  }
}


/* --------- Res: Organized PC -|- View: Normal ---------*/
/* -----------------------------------------------------*/
template<typename RT, typename VT>
template<typename PT, size_t Channel>
void Gui::View<RT,VT>::reloadData(ResourceTypes::OrganizedPointCloud<PT>, ViewTypes::Normal<Channel>)
{
  typedef Gui::ImageView<Gui::ResourceTypes::OrganizedPointCloud<PT>, Gui::ViewTypes::Normal<Channel> > IV;

  int w = this->r_ptr->getData()->width;
  int h = this->r_ptr->getData()->height;
  static_cast<IV*>(this)->bmp_.reset(new wxBitmap(w,h));
  wxNativePixelData data(*(static_cast<IV*>(this)->bmp_));
  wxNativePixelData::Iterator pwx(data);
  cv::Vec3b bgr;
  for(int row = 0; row < h; ++row)
  {
    for(int col = 0; col < w; ++col, ++pwx)
    {
      PT* ppc = &(*this->r_ptr->getData())[col + row * w];
      if(ppc->normal[Channel] != ppc->normal[Channel])
      {
        pwx.Red() = 0;
        pwx.Green() = 255.0f;
        pwx.Blue() = 0;
      }
      else
      {
        int n_n = (ppc->normal[Channel] + 1.0f) * 127.5f;
        pwx.Red() = n_n;
        pwx.Green() = n_n;
        pwx.Blue() = n_n;
      }
    }
  }
}


  /* -----------------------------*/
 /* --------- ImageView ---------*/
/* -----------------------------*/
template<typename RT, typename VT>
void Gui::ImageView<RT,VT>::render()
{
  wxPaintDC dc(this);
  render(dc);
}

template<typename RT, typename VT>
void Gui::ImageView<RT,VT>::render(wxDC& dc)
{
  dc.DrawBitmap(*bmp_, 0, 0, false);
}


template<typename RT, typename VT>
void Gui::ImageView<RT,VT>::paintEvent(wxPaintEvent& event)
{
  wxPaintDC dc(this);
  render(dc);
}

template<typename RT, typename VT>
void Gui::ImageView<RT,VT>::mouseEvent(wxMouseEvent& event)
{
  if(event.LeftDClick())
  {
    wxPoint p1 = event.GetPosition();
    //std::cout << "mouse click thing on " << this->name_ << " at " << p1.x << "," << p1.y << std::endl;
    this->mouse_sig_(event, this->r_ptr);
  }
  else if(event.RightDClick())
  {
    this->mouse_sig_(event, this->r_ptr);
  }
}

template<typename RT, typename VT>
void Gui::ImageView<RT,VT>::keyEvent(wxKeyEvent& event)
{
  this->key_sig_(event, this->r_ptr);
}

// Needed to fix these wxWidget macros (v2.8) to be able to use class templates in event table!
BEGIN_EVENT_TABLE_TEMPLATE2(Gui::ImageView, wxPanel, RT, VT)
wx__DECLARE_EVT0(
  wxEVT_PAINT,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxPaintEventFunction>(&Gui::ImageView<RT,VT>::paintEvent))
wx__DECLARE_EVT0(
  wxEVT_LEFT_UP,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_MIDDLE_DOWN,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_MIDDLE_UP,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_RIGHT_DOWN,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_RIGHT_UP,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_MOTION,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_LEFT_DCLICK,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_MIDDLE_DCLICK,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_RIGHT_DCLICK,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_LEAVE_WINDOW,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_ENTER_WINDOW,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))
wx__DECLARE_EVT0(
  wxEVT_MOUSEWHEEL,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxMouseEventFunction>(&Gui::ImageView<RT,VT>::mouseEvent))

wx__DECLARE_EVT0(
  wxEVT_KEY_DOWN,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxCharEventFunction>(&Gui::ImageView<RT,VT>::keyEvent))
wx__DECLARE_EVT0(
  wxEVT_KEY_UP,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxCharEventFunction>(&Gui::ImageView<RT,VT>::keyEvent))
wx__DECLARE_EVT0(
  wxEVT_CHAR,
  (wxObjectEventFunction)(wxEventFunction) static_cast<wxCharEventFunction>(&Gui::ImageView<RT,VT>::keyEvent))

END_EVENT_TABLE()

#endif
