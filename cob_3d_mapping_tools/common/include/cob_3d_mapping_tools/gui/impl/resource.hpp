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

#ifndef COB_3D_MAPPING_TOOLS_GUI_RESOURCE_HPP_
#define COB_3D_MAPPING_TOOLS_GUI_RESOURCE_HPP_

#include "boost/make_shared.hpp"
#include "cob_3d_mapping_tools/gui/resource.h"

template<typename RT>
template<typename VT>
typename Gui::View<RT,VT>::Ptr Gui::Resource<RT>::createView(const std::string& name, int options)
{
  ViewBase::Ptr ptr(new View<RT, VT>(name, this));
  views_.insert(std::pair<std::string, ViewBase::Ptr>(name, ptr));
  return boost::static_pointer_cast<View<RT, VT> >(ptr);
}

template<typename RT>
std::map<std::string, Gui::Resource<RT> >* Gui::ResourceManager::get()
{
  static std::map<std::string, Resource<RT> > map; return &map;
}

template<typename RT>
typename Gui::Resource<RT>::Ptr Gui::ResourceManager::create(const std::string& name, const RT& data)
{
  return boost::make_shared<Resource<RT> >(
        get<RT>()->insert(std::pair<std::string,Resource<RT> >(name, Resource<RT>(name,data))).first->second);
}

#endif
