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

#ifndef COB_3D_MAPPING_TOOLS_GUI_CORE_HPP_
#define COB_3D_MAPPING_TOOLS_GUI_CORE_HPP_

#include "cob_3d_mapping_tools/gui/core.h"
#include "cob_3d_mapping_tools/gui/impl/view.hpp"
#include "cob_3d_mapping_tools/gui/impl/resource.hpp"
#include "cob_3d_mapping_tools/gui/impl/tools.hpp"


  /* ---------------------------------*/
 /* --------- WindowManager ---------*/
/* ---------------------------------*/
template<typename RT, typename VT>
void Gui::WindowManager::create(ImageView<RT,VT>* object, const std::string& status_msg)
{
  Window* w = new Window(object->name_);
  wxStatusBar* sb = new wxStatusBar(w);
  sb->SetStatusText(wxString(status_msg.c_str(), wxConvUTF8));
  object->Create(w, -1, wxDefaultPosition, wxSize(object->bmp_->GetWidth(), object->bmp_->GetHeight()));
  wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(object, 0, wxEXPAND);
  sizer->Add(sb, 0, 0, 0);
  sizer->SetSizeHints(w);
  w->SetSizer(sizer);
  w->SetAutoLayout(true);
  w->Show(true);
}

template<typename RT, typename VT>
void Gui::WindowManager::moveWindow(View<RT,VT>* window_ptr, int x, int y)
{
  moveWindow(window_ptr,wxPoint(x,y),VT());
}

template<typename RT, typename VT>
void Gui::WindowManager::moveWindow(View<RT,VT>* w, const wxPoint& p, ViewTypes::View2D)
{
  wxWindow* wxW = static_cast<ImageView<RT,VT>*>(w);
  while (!wxW->IsTopLevel())
    wxW = wxW->GetParent();
  wxW->SetPosition(p);
}


  /* -----------------------------------*/
 /* --------- ResourceManager ---------*/
/* -----------------------------------*/
template<typename RT>
std::map<std::string, Gui::Resource<RT>* >* Gui::ResourceManager::get()
{
  static std::map<std::string, Resource<RT>* > map; return &map;
}


template<typename RT>
Gui::Resource<RT>* Gui::ResourceManager::create(const std::string& name, const typename RT::DataTypePtr& data)
{
  Resource<RT>* r = new Resource<RT>(name,data);
  get<RT>()->insert(std::pair<std::string,Resource<RT>* >(name, r));
  return r;
}

template<typename RT>
Gui::Resource<RT>* Gui::ResourceManager::create(const std::string& name, const std::string& file, ResourceTypes::BaseCloud)
{
  typename RT::DataTypePtr tmp_data(new typename RT::DataTypeRaw);
  if (pcl::io::loadPCDFile<RT::PointType>(file, *tmp_data) < 0) return NULL;
  return create<RT>(name,tmp_data);
}

template<typename RT>
Gui::Resource<RT>* Gui::ResourceManager::create(const std::string& name, const std::string& file, ResourceTypes::Image)
{
  typename RT::DataTypePtr tmp_data(new typename RT::DataTypeRaw);
  *tmp_data = cv::imread(file);
  if (tmp_data->data==NULL) return NULL;
  return create<RT>(name,tmp_data);
}


template<typename RT>
void Gui::ResourceManager::destroy(const std::string& name)
{
  typedef typename std::map<std::string,Resource<RT>*>::iterator iterator;
  iterator it = get<RT>()->find(name);
  delete it->second;
  get<RT>()->erase(it);
}

#endif
