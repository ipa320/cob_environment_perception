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

#include "cob_3d_meshing/mesh_property_types.h"

namespace cob_3d_meshing
{
  /* TODO:
   * Empty Base Class Optimization(EBCO) to remove storage overhead
   * for empty properties */
  template<typename PT1 = MeshProperties::NullT<>,
           typename PT2 = MeshProperties::NullT<>,
           typename PT3 = MeshProperties::NullT<>,
           typename PT4 = MeshProperties::NullT<>,
           typename PT5 = MeshProperties::NullT<>,
           typename PT6 = MeshProperties::NullT<>,
           typename MeshTraits = DefaultMeshTraits >
  class Mesh
  {
  public:
    typedef typename MeshTraits::MeshT MeshT;
    typedef typename MeshTraits::VertexHandle VertexHandle;
    typedef typename MeshTraits::FaceHandle FaceHandle;

    template<typename U> friend class MeshDecimation;

    Mesh(const PT1& prop1 = PT1(),
         const PT2& prop2 = PT2(),
         const PT3& prop3 = PT3(),
         const PT4& prop4 = PT4(),
         const PT5& prop5 = PT5(),
         const PT6& prop6 = PT6())
      : mesh_()
      , prop1_(prop1)
      , prop2_(prop2)
      , prop3_(prop3)
      , prop4_(prop4)
      , prop5_(prop5)
      , prop6_(prop6)
    {
      prop1_.init(&mesh_);
      prop2_.init(&mesh_);
      prop3_.init(&mesh_);
      prop4_.init(&mesh_);
      prop5_.init(&mesh_);
      prop6_.init(&mesh_);
    }

    template<typename PointT>
    VertexHandle addVertex(int idx, const PointT& p)
    {
      VertexHandle vh = mesh_.add_vertex(typename MeshT::Point(p.x,p.y,p.z));
      prop1_.setVertex(vh,idx);
      prop2_.setVertex(vh,idx);
      prop3_.setVertex(vh,idx);
      prop4_.setVertex(vh,idx);
      prop5_.setVertex(vh,idx);
      prop6_.setVertex(vh,idx);
      return vh;
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
    PT1 prop1_;
    PT2 prop2_;
    PT3 prop3_;
    PT4 prop4_;
    PT5 prop5_;
    PT6 prop6_;
  };

}

#endif
