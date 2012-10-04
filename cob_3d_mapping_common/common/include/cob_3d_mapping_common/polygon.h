/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 03/2012
*
* \brief
* Class representing polygon shapes
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#ifndef POLYGON_H_
#define POLYGON_H_

#include "cob_3d_mapping_common/shape.h"
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
extern "C" {
#include "cob_3d_mapping_common/gpc.h"
}
#include <fstream>

//#include "cob_3d_mapping_common/stop_watch.h"

namespace cob_3d_mapping
{
  struct merge_config
  {
    double angle_thresh;
    float  d_thresh;
    std::string weighting_method;
  };


  //Polygon Class, derived from Shape class
  class Polygon : public Shape
  {
  public:
    typedef boost::shared_ptr<Polygon> Ptr;/**< Polygon pointer. Boost shared pointer to polygon. */

    typedef boost::shared_ptr<const Polygon> ConstPtr;/**< Const. Polygon pointer. Const boost shared pointer to polygon. */

  public:
  /**
  * \brief Constructor for polygon object
  */
    Polygon()
      : normal(Eigen::Vector3f::Zero())
      , d(0.0)
      , transform_from_world_to_plane(Eigen::Affine3f::Identity())
      , merge_weight_(1.0)
    { }

    inline size_t outerContourIndex() const
    {
      for(size_t i=0;i<contours.size();++i) { if(!holes[i]) { return i; } };
      return 0;
    }

    //##########methods for instantiation##############
    virtual void computeAttributes(const Eigen::Vector3f &new_normal, const Eigen::Vector4f & new_centroid);
    virtual void transform2tf(const Eigen::Affine3f& trafo);
    void smoothPolygon();

    //###########methods for merging##################
    virtual void getMergeCandidates(const std::vector<Polygon::Ptr>& poly_vec, std::vector<int>& intersections) const;
    virtual bool isIntersectedWith(const Polygon::Ptr& poly) const;
    void getIntersection(const Polygon::Ptr& poly, gpc_polygon* gpc_intersection) const;
    bool getContourOverlap(const Polygon::Ptr& poly, float& rel_overlap, int& abs_overlap) const;
    float computeSimilarity(const Polygon::Ptr& poly) const;

    virtual void merge(std::vector<Polygon::Ptr>& poly_vec);
    void merge_union(std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr&  p_average);
    void assignWeight();
    void assignID(const std::vector<Polygon::Ptr>& poly_vec);
    virtual void applyWeighting(const std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr& p_average);
    void getGpcStructure(const Eigen::Affine3f& external_trafo, gpc_polygon* gpc_p) const;
    void applyGpcStructure(const Eigen::Affine3f& external_trafo, const gpc_polygon* gpc_p);

    /**
    * \brief Check for similar polygon parameters.
    *
    * Check is performed based on merge settings of polygons.
    */
    inline bool hasSimilarParametersWith(const Polygon::Ptr& poly) const
    {
      Eigen::Vector3f d = (this->centroid - poly->centroid).head(3);
      return ( fabs(poly->normal.dot(this->normal)) > this->merge_settings_.angle_thresh &&
               fabs( d.dot(this->normal) ) < this->merge_settings_.d_thresh &&
               fabs( d.dot(poly->normal) ) < this->merge_settings_.d_thresh );
    }

    //#######methods for calculation#####################

    void computeCentroid();

    double computeArea() const;  // http://paulbourke.net/geometry/polyarea/
    double computeArea3d() const;
    void getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,const Eigen::Vector3f &origin,
                                           Eigen::Affine3f &transformation) const;
    void getTransformationFromPlaneToWorld(const Eigen::Vector3f z_axis,const Eigen::Vector3f &normal,
                                           const Eigen::Vector3f &origin, Eigen::Affine3f &transformation);
    void getTransformedContours(const Eigen::Affine3f& trafo, std::vector<std::vector<Eigen::Vector3f> >& new_contours ) const;
    void TransformContours(const Eigen::Affine3f& trafo);
    void computePoseAndBoundingBox(Eigen::Affine3f& pose, Eigen::Vector4f& min_pt, Eigen::Vector4f& max_pt);

    //#############debugging methods#######################
    void debug_output(std::string name);


    //########## member variables#################
    //needed for 32-bit systems: see http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      std::vector<std::vector<Eigen::Vector3f> > contours;/**< Contours of polygon sturcure. */
    Eigen::Vector3f normal;/**< Normal of polygons structure. */
    double d;
    Eigen::Affine3f transform_from_world_to_plane;/**< Transformation from world coordinate system to cvoordinate system on polygon. */
    std::vector<bool> holes;/**< Bool vector indicating holes in polygon structure*/
    double merge_weight_;/**< Merge weight of polygon - regulating its influence in merge processes.*/
    merge_config merge_settings_;/** Merge settings - configuring assignment of weight and merge processes*/

  };
}
#endif /* POLYGON_H_ */
