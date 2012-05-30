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
 * ROS stack name: cob_environment_perception
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
 * ToDo:
 *
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

#ifndef POLYGON_H_
#define POLYGON_H_

#include "cob_3d_mapping_common/shape.h"
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
extern "C" {
#include "cob_3d_mapping_common/gpc.h"
}

namespace cob_3d_mapping
{

struct merge_config {
  double angle_thresh ;
  float  d_thresh;
} ;


//Polygon Class, derived from Shape class


  class Polygon : public Shape
  {
  public:

    void
    computeCentroid();

    // http://paulbourke.net/geometry/polyarea/
    // works for all polygons except self-intersecting polygons
    double
    computeArea();





//    Compute vector containing indices(intersections) of merge candidates for polygon in poly_vec
    void isMergeCandidate(std::vector< boost::shared_ptr<Polygon> >& poly_vec,merge_config& limits, std::vector<int>& intersections);
    bool isMergeCandidate_intersect(Polygon& p_map);

//    Merge polygon with polygons in poly_vec
    void merge(std::vector< boost::shared_ptr<Polygon> >& poly_vec);

//    Calculate members of polygon
 	void assignMembers(Eigen::Vector3f &new_normal,double &new_d);
 	void assignMembers();

 	//    Use general polygon clipper to create polygon structures
 	    void
 	    GpcStructureUsingMap(Eigen::Affine3f& external_trafo,gpc_polygon* gpc_p);

 	    void
 	    GpcStructure( gpc_polygon* gpc_p);

 	void debug_output(std::string name);


    //needed for 32-bit systems: see http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<std::vector<Eigen::Vector3f> > contours;
    Eigen::Vector3f normal;
    double d;
    Eigen::Affine3f transform_from_world_to_plane;
    std::vector<bool> holes;


  private:




  };

  typedef boost::shared_ptr<Polygon> PolygonPtr;
}

#endif /* POLYGON_H_ */
