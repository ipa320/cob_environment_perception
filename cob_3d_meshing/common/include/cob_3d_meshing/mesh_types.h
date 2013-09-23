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
 *  ROS package name: cob_3d_meshing
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 09/2013
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

#ifndef COB_MESH_TYPES_H
#define COB_MESH_TYPES_H

#include "cob_3d_meshing/slots.hpp"
#include "cob_3d_meshing/mesh_property_types.h"

namespace cob_3d_meshing
{
  /*** policy classes to access plugin functionalities ***
   *
   * "inline static void T::runPlugin(const PluginT&, ...)
   * is called by slots.run<T>() for all defined plugins.
   * it should just redirect to the desired function within the plugin.
   *
   */
  template<typename MeshTraits = DefaultMeshTraits>
  class SetVertex
  {
  public:
    typedef typename MeshTraits::VertexHandle VertexHandle;

    struct PluginArgs { VertexHandle vh; int idx; };

    template<typename PluginT>
    inline static void runPlugin(PluginT& plugin, const PluginArgs& args)
    {
      plugin.setVertex(args.vh, args.idx);
    }
  };

  template<typename MeshTraits = DefaultMeshTraits>
  class Init
  {
  public:
    typedef typename MeshTraits::MeshT MeshT;

    template<typename PluginT>
    inline static void runPlugin(PluginT& plugin, MeshT* mesh)
    {
      plugin.init(mesh);
    }
  };


  template<typename PT1 = NullT,
           typename PT2 = NullT,
           typename PT3 = NullT,
           typename PT4 = NullT,
           typename PT5 = NullT,
           typename MeshTraits = DefaultMeshTraits >
  class Mesh
  {
  public:
    typedef typename MeshTraits::MeshT MeshT;
    typedef typename MeshTraits::VertexHandle VertexHandle;
    typedef typename MeshTraits::FaceHandle FaceHandle;

    // tie all properties together in one handler:
    typedef Slots<PT1,PT2,PT3,PT4,PT5> Properties;

    template<typename U> friend class MeshDecimation;

    Mesh(const PT1& prop1 = PT1(),
         const PT2& prop2 = PT2(),
         const PT3& prop3 = PT3(),
         const PT4& prop4 = PT4(),
         const PT5& prop5 = PT5())
      : mesh_()
      , prop_(prop1,prop2,prop3,prop4,prop5)
    {
      // init all properites
      runPolicy<Init<MeshTraits> >(prop_, &mesh_);
    }

    template<typename PointT>
    VertexHandle addVertex(int idx, const PointT& p)
    {
      typename SetVertex<MeshTraits>::PluginArgs args;
      args.vh = mesh_.add_vertex(typename MeshT::Point(p.x,p.y,p.z));
      args.idx = idx;
      runPolicy<SetVertex<MeshTraits> >(prop_,args);
      return args.vh;
    }

    FaceHandle addFace(const VertexHandle& v1,
                       const VertexHandle& v2,
                       const VertexHandle& v3)
    {
      return mesh_.add_face(v1,v2,v3);
    }

    bool savePLY(const std::string& file)
    {
      return OpenMesh::IO::write_mesh(mesh_, file);
    }

  private:
    MeshT mesh_;
    Properties prop_;

  };

}

#endif
