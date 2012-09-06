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
  namespace ResourceTypes
  {
    struct PointCloud {};
    struct OrganizedPointCloud : PointCloud {};
    struct Image {};
  }

  // forward declarations:
  //template<typename RT, typename VT> class View;

  template<typename RT>
  class Resource
  {
  public:
    typedef boost::shared_ptr<Resource<RT> > Ptr;

    Resource(const std::string& name, RT data) : name_(name), data_(data) { }
    ~Resource() { }

    template<typename VT> typename View<RT, VT>::Ptr createView(const std::string& name, int options);

    inline RT& getData() { return data_; }

  private:
    std::string name_;
    RT data_;

    std::map<std::string, ViewBase::Ptr> views_;
  };

  class ResourceManager
  {
  public:
    ~ResourceManager() { }

  private:
    ResourceManager() { }

    template<typename RT> std::map<std::string, Resource<RT> >* get();
    template<typename RT> typename Resource<RT>::Ptr create(const std::string& name, const RT& data);

    friend class Core;
  };
}

#endif
