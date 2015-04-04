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

#ifndef COB_3D_MAPPING_TOOLS_GUI_VIEW_H_
#define COB_3D_MAPPING_TOOLS_GUI_VIEW_H_

#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/signals2.hpp>

#include <wx/wx.h>
#include <wx/rawbmp.h>

#include "cob_3d_mapping_tools/gui/tools.h"

namespace Gui
{

  // forward declaration:
  class ResourceBase;
  template<typename RT> class Resource;

  class ViewBase
  {
  public:
    virtual void show()=0;
    virtual void onDataChanged()=0;

    ResourceBase* r_base_ptr;

  protected:
    ViewBase(const std::string& name) : name_(name) { }
    ~ViewBase();

    std::string name_;
    friend class WindowManager;
  };



  template<typename RT, typename VT>
  class View : public ViewBase
  {
  public:
    View(const std::string& name, Resource<RT>* r) : ViewBase(name), r_ptr(r) { this->r_base_ptr = r; }

    void show() { reloadData(RT(),VT()); show(VT()); }
    void onDataChanged() { reloadData(RT(),VT()); refresh(VT()); }
    boost::signals2::connection registerMouseCallback(boost::function<void (wxMouseEvent&, Resource<RT>*)>);
    boost::signals2::connection registerKeyCallback(boost::function<void (wxKeyEvent&, Resource<RT>*)>);

  protected:
    ~View() { std::cout << "View destroyed" << std::endl; }

    void show(ViewTypes::View2D);
    void refresh(ViewTypes::View2D);

    void reloadData(ResourceTypes::Image, ViewTypes::Color);
    template<typename PT> void reloadData(ResourceTypes::OrganizedPointCloud<PT>, ViewTypes::Color);
    template<typename PT> void reloadData(ResourceTypes::OrganizedPointCloud<PT>, ViewTypes::Depth_Z);
    template<typename PT, size_t Channel> void reloadData(ResourceTypes::OrganizedPointCloud<PT>, ViewTypes::Normal<Channel>);

    Resource<RT>* r_ptr;
    boost::signals2::signal<void (wxMouseEvent&, Resource<RT>*)> mouse_sig_;
    boost::signals2::signal<void (wxKeyEvent&, Resource<RT>*)> key_sig_;

    friend class WindowManager;
  };



  template<typename RT, typename VT>
  class ImageView : public View<RT,VT> , public wxPanel
  {
  public:
    ImageView(const std::string& name, Resource<RT>* r) : View<RT,VT>(name,r) { }
    ~ImageView() { }
    void render();

    wxBitmapPtr bmp_;

  private:
    void render(wxDC& dc);
    void paintEvent(wxPaintEvent& event);
    void mouseEvent(wxMouseEvent& event);
    void keyEvent(wxKeyEvent& event);

    friend class WindowManager;
    DECLARE_EVENT_TABLE()
  };



  template<typename RT, typename VT>
  class TextView : public View<RT,VT>
  {
  public:
    TextView(const std::string& name, Resource<RT>* r) : View<RT,VT>(name,r) { }
    ~TextView() { }

    friend class WindowManager;
  };

}

#endif
