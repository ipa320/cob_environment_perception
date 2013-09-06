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

#ifndef COB_DECIMATION_MODULES_H
#define COB_DECIMATION_MODULES_H

#include <OpenMesh/Tools/Decimater/ModBaseT.hh>

namespace OpenMesh
{
  namespace Decimater
  {
    template<typename MeshT>
      class ModNormalQuadricT : public OpenMesh::Decimater::ModBaseT<MeshT>
    {
    public:
      // Defines the types Self, Handle, Base, Mesh, and CollapseInfo
      // and the memberfunction name()
      DECIMATING_MODULE( ModNormalQuadricT, MeshT, NormalQuadric );

    public:
    ModNormalQuadricT( MeshT &_mesh )
      : Base(_mesh, false)
      {
        unset_max_err();
        Base::mesh().add_property( quadrics_ );
      }

      virtual ~ModNormalQuadricT()
      {
        Base::mesh().remove_property(quadrics_);
      }

    public: // inherited

      virtual void initialize(void);

      virtual float collapse_priority(const CollapseInfo& _ci)
      {
        typedef Geometry::QuadricT<double> Q;

        Q q = Base::mesh().property(quadrics_, _ci.v0);
        q += Base::mesh().property(quadrics_, _ci.v1);
        /*Vec3f v = (_ci.p1 + _ci.p0);
          v = v * 0.5;
          double err = q(v);*/
        double err = q(_ci.p1);
        //std::cout << v << " Err: " << err << std::endl;

        return float( (err < max_err_) ? err : Base::ILLEGAL_COLLAPSE );
      }

      virtual void postprocess_collapse(const CollapseInfo& _ci)
      {
        Base::mesh().property(quadrics_, _ci.v1) +=
          Base::mesh().property(quadrics_, _ci.v0);

        //Base::mesh().point(_ci.v1) += Base::mesh().point(_ci.v0);
        //Base::mesh().point(_ci.v1) *= 0.5;
      }

      void set_error_tolerance_factor(double _factor);

    public: // specific methods

      void set_max_err(double _err, bool _binary=true)
      {
        max_err_ = _err;
        Base::set_binary(_binary);
      }

      void set_normal_property(const VPropHandleT<Vec4f>& normals)
      {
        normals_ = normals;
      }

      void set_label_property(const VPropHandleT<int>& labels)
      {
        labels_ = labels;
      }

      void unset_max_err(void) { max_err_ = DBL_MAX; Base::set_binary(false); }
      double max_err() const { return max_err_; }

    private:
      double max_err_;

      VPropHandleT< Geometry::QuadricT<double> >  quadrics_;
      VPropHandleT< Vec4f > normals_;
      VPropHandleT< int > labels_;
    };


//=============================================================================
  } // END_NS_DECIMATER
} // END_NS_OPENMESH
//=============================================================================

#endif
