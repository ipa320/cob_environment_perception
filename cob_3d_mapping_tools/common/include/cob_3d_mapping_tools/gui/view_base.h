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

#ifndef COB_3D_MAPPING_TOOLS_GUI_VIEW_BASE_H_
#define COB_3D_MAPPING_TOOLS_GUI_VIEW_BASE_H_

#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>

namespace Gui
{
  class ViewBase
  {
    public:
    typedef boost::shared_ptr<ViewBase> Ptr;

    protected:
    ViewBase(const std::string& name) : name_(name) { }
    ~ViewBase() { } // prevent user to call "delete ViewBase::Ptr::get()". No need to declare virtual.

    virtual void onDataChanged()=0;
    std::string name_;
  };
}









/*
namespace cob_3d_mapping_tools
{
  namespace Gui
  {
    template<typename DataT>
    class BaseView
    {
    public:
      BaseView(DataT* data) : data_ptr(data) {}
      virtual ~BaseView() {}

      void create();
      virtual void createChild()=0;
      virtual void onDataChanged()=0;
      virtual void show()=0;
      virtual void hide()=0;

    protected:
      DataT* data_ptr;
    };
  }
}
*/
#endif
