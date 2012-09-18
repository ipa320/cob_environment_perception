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

#ifndef COB_3D_MAPPING_TOOLS_GUI_CORE_H_
#define COB_3D_MAPPING_TOOLS_GUI_CORE_H_

#include <wx/wx.h>
#include <wx/minifram.h>
#include "cob_3d_mapping_tools/gui/resource.h"

namespace Gui
{
  class Window : public wxFrame
  {
  public:
    Window(const std::string& title) : wxFrame(NULL, -1, wxString(title.c_str(), wxConvUTF8)), id(title)
    { }

    std::string id;
  };

  /* ---------------------------------*/
 /* --------- WindowManager ---------*/
/* ---------------------------------*/

  class WindowManager
  {
  public:
    ~WindowManager() { std::cout << "wm destroyed" << std::endl; }
    template<typename RT, typename VT> void create(ImageView<RT,VT>* object, const std::string& status_msg="");
    template<typename RT, typename VT> void moveWindow(View<RT,VT>* window_ptr, int x, int y);
  private:
    WindowManager() { }

    template<typename RT, typename VT> void moveWindow(View<RT,VT>* w, const wxPoint& p, ViewTypes::View2D);
    friend class Core;
  };

  /* -----------------------------------*/
 /* --------- ResourceManager ---------*/
/* -----------------------------------*/
  class ResourceManager
  {
  public:
    ~ResourceManager() { std::cout << "res manager destroyed" << std::endl; }

    // --- delegated construction ---
    template<typename RT> Resource<RT>* create(const std::string& name, const std::string& file) {return create<RT>(name,file,RT());}
    template<typename RT> Resource<RT>* create(const std::string& name, const typename RT::DataTypePtr& data);

    template<typename RT> void destroy(const std::string& name);

  private:
    ResourceManager() { }

    template<typename RT> std::map<std::string, Resource<RT>* >* get();

    // --- specializations ---
    template<typename RT> Resource<RT>* create(const std::string& name, const std::string& file, ResourceTypes::BaseCloud);
    template<typename RT> Resource<RT>* create(const std::string& name, const std::string& file, ResourceTypes::Image);

    friend class Core;
  };

  /* ------------------------*/
 /* --------- CORE ---------*/
/* ------------------------*/
  class Core
  {
  public:
    ~Core() { std::cout << "Core Destroyed" << std::endl; }
    static Core* Get() { static Core* c_ = new Core(); return c_; }
    static WindowManager* wMan() { static WindowManager* wm_ = new WindowManager(); return wm_; }
    static ResourceManager* rMan() { static ResourceManager* rm_ = new ResourceManager(); return rm_; }
  private:
    Core() { }
  };
}

#endif
