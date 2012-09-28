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

#ifndef COB_3D_MAPPING_TOOLS_GUI_RESOURCE_H_
#define COB_3D_MAPPING_TOOLS_GUI_RESOURCE_H_

#include <map>

#include "cob_3d_mapping_tools/gui/view.h"

namespace Gui
{
  /* ------------------------*/
 /* --------- Base ---------*/
/* ------------------------*/
  class ResourceBase
  {
  public:
    virtual void releaseView(const std::string&)=0;
    virtual void resourceChanged()=0;
  };


  /* ----------------------------*/
 /* --------- Resource ---------*/
/* ----------------------------*/
  template<typename RT>
  class Resource : public ResourceBase
  {
  public:
    Resource(const std::string& name, const typename RT::DataTypePtr& data) : name_(name), data_(data) { }
    ~Resource() { std::cout << "resource destroyed" << std::endl; }

    // --- delegated construction ---
    template<typename VT> View<RT, VT>* createView(const std::string& name) {return createView<VT>(name,VT());}

    inline typename RT::DataTypePtr& getData() { return data_; }
    void releaseView(const std::string& name) { views_.erase(name); }
    void resourceChanged();

  private:
    // --- specializations ---
    template<typename VT> View<RT, VT>* createView(const std::string& name,ViewTypes::View2D);
    template<typename VT> View<RT, VT>* createView(const std::string& name,ViewTypes::ViewText);

    std::string name_;
    typename RT::DataTypePtr data_;
    std::map<std::string, ViewBase*> views_;
  };
}

#endif
