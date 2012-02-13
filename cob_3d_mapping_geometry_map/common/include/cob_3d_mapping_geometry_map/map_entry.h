/*
 * MapEntry.h
 *
 *  Created on: 02.11.2011
 *      Author: goa-hh
 */

#ifndef MAPENTRY_H_
#define MAPENTRY_H_

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

  struct MapEntry
  {
    //needed for 32-bit systems: see http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    unsigned int id;
    std::vector<std::vector<Eigen::Vector3f> > polygon_world;
    //cob_env_model::PolygonArray polygon_world;
    //gpc_polygon polygon_plane;
    Eigen::Vector3f normal;
    double d;
    Eigen::Affine3f transform_from_world_to_plane;
    unsigned int merged;
  };

  typedef boost::shared_ptr<MapEntry> MapEntryPtr;



#endif /* MAPENTRY_H_ */
