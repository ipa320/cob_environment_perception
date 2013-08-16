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
 *  ROS package name: cob_3d_segmentation
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2013
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

#ifndef COB_MESH_DECIMATION_H
#define COB_MESH_DECIMATION_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModBaseT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>

#include <pcl/point_types.h>


namespace OpenMesh {
namespace Decimater {

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
    Vec3f v = (_ci.p1 + _ci.p0);
    v = v * 0.5;
    double err = q(v);
    //std::cout << v << " Err: " << err << std::endl;

    return float( (err < max_err_) ? err : float( Base::ILLEGAL_COLLAPSE ) );
  }

  virtual void postprocess_collapse(const CollapseInfo& _ci)
  {
    Base::mesh().property(quadrics_, _ci.v1) +=
      Base::mesh().property(quadrics_, _ci.v0);

    Base::mesh().point(_ci.v1) += Base::mesh().point(_ci.v0);
    Base::mesh().point(_ci.v1) *= 0.5;
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


// based on: Surface Simplification Using Quadric Error Metrics,
// M Garland, P S Heckbert
template<typename PointT, typename NormalT, typename LabelT>
class MeshSimplification
{
public:
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  typedef typename pcl::PointCloud<NormalT>::ConstPtr NormalCloudConstPtr;
  typedef typename pcl::PointCloud<LabelT>::ConstPtr LabelCloudConstPtr;

  typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh; // Triangle Mesh
  typedef boost::shared_ptr<Mesh> MeshPtr;
  typedef OpenMesh::Decimater::DecimaterT<Mesh> Decimater;
  typedef boost::shared_ptr<Decimater> DecimaterPtr;
  typedef OpenMesh::Decimater::ModNormalQuadricT<Mesh> ModT;
  typedef typename ModT::Handle ModHandle;
  typedef OpenMesh::Vec4f PropNormalT;
  typedef OpenMesh::VPropHandleT<PropNormalT> PropNormalHandle;
  typedef OpenMesh::VPropHandleT< int > PropLabelHandle;

public:
  MeshSimplification()
    : mesh_(new Mesh())
    , dec_(new Decimater(*mesh_))
    , mod_()
    , p_normals_()
    , p_labels_()
  {
    mesh_->add_property(p_normals_, "Normals");
    mesh_->add_property(p_labels_, "Labels");
    dec_->add(mod_);
    dec_->module(mod_).set_binary(false);
    dec_->module(mod_).set_normal_property(p_normals_);
    dec_->module(mod_).set_label_property(p_labels_);
  }

  ~MeshSimplification() { }

  void initializeMesh(const PointCloudConstPtr& input,
                      const LabelCloudConstPtr& labels,
                      const NormalCloudConstPtr& normals,
                      const std::map<int,Eigen::Vector4f>& params);
  void decimate();

  inline bool isNeighbor(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
  {
    //Eigen::Vector3f ab = b - a;
    //return ( fabs(a.dot(ab) / (a.norm() * ab.norm())) < 0.9762f);
    return cob_3d_mapping::PrimeSense::areNeighbors(a,b);
  }

  inline Mesh& getMesh() { return dec_->mesh(); }

private:

  MeshPtr mesh_;
  DecimaterPtr dec_;
  ModHandle mod_;
  PropNormalHandle p_normals_;
  PropLabelHandle p_labels_;
};

#endif
