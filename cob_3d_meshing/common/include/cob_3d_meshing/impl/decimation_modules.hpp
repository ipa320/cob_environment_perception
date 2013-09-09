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

using namespace OpenMesh;
using namespace Decimator;

template<typename MeshT>
void ModNormalQuadricT<MeshT>::initialize()
{
  /*
  using OpenMesh::Geometry::Quadricd;
  if (!quadrics_.is_valid())
    Base::mesh().add_property( quadrics_ );

  typename MeshT::VertexIter v_it  = Base::mesh().vertices_begin();
  typename MeshT::VertexIter v_end = Base::mesh().vertices_end();

  for (; v_it!=v_end; ++v_it)
  {
    Vec4f p = Base::mesh().property(normals_, v_it);
    //std::cout << p << std::endl;
    Base::mesh().property(quadrics_, v_it)
      = Quadricd(p[0], p[1], p[2], p[3]);
  }
  */

  using OpenMesh::Geometry::Quadricd;
  if (!quadrics_.is_valid())
    Base::mesh().add_property( quadrics_ );

  typename Mesh::VertexIter v_it  = Base::mesh().vertices_begin();
  typename Mesh::VertexIter v_end = Base::mesh().vertices_end();

  for (; v_it!=v_end; ++v_it)
    Base::mesh().property(quadrics_, v_it).clear();

  typename Mesh::FaceIter f_it  = Base::mesh().faces_begin();
  typename Mesh::FaceIter f_end = Base::mesh().faces_end();
  typename Mesh::FaceVertexIter fv_it;
  typename Mesh::VertexHandle vh0, vh1, vh2;
  Vec4f n;

  for (; f_it!=f_end; ++f_it)
  {
    fv_it = Base::mesh().fv_iter(f_it.handle());
    for (int i=0; i<3; ++i, ++fv_it)
    {
      vh0 = fv_it.handle();
      n = Base::mesh().property(normals_, vh0);
      Base::mesh().property(quadrics_, vh0) += Quadricd(n);
    }

    /*
    vh0 = fv_it.handle(); ++fv_it;
    vh1 = fv_it.handle(); ++fv_it;
    vh2 = fv_it.handle();

    n1 = Base::mesh().property(normals_, vh0);
    n2 = Base::mesh().property(normals_, vh1);
    n3 = Base::mesh().property(normals_, vh2);

    Quadricd q1(n1);
    Quadricd q2(n2);
    Quadricd q3(n3);

    Base::mesh().property(quadrics_, vh0) += q1;
    Base::mesh().property(quadrics_, vh1) += q2;
    Base::mesh().property(quadrics_, vh2) += q3;
    */
  }
}

template<typename MeshT>
void ModNormalQuadricT<MeshT>::set_error_tolerance_factor(double _factor)
{
  if (this->is_binary()) {
    if (_factor >= 0.0 && _factor <= 1.0) {
      // the smaller the factor, the smaller max_err_ gets
      // thus creating a stricter constraint
      // division by error_tolerance_factor_ is for normalization
      double max_err = max_err_ * _factor / this->error_tolerance_factor_;
      set_max_err(max_err);
      this->error_tolerance_factor_ = _factor;

      initialize();
    }
  }
}

}
}

