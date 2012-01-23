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
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description: Feature Map for storing and handling geometric features
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2011
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

#ifndef __GEOMETRY_MAP_H__
#define __GEOMETRY_MAP_H__

//##################
//#### includes ####

// external includes
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

// internal includes
extern "C" {
#include "cob_3d_mapping_geometry_map/gpc.h"
}
//#ifndef __GEOMETRY_MAP_VISUALISATION_H__
#include "cob_3d_mapping_geometry_map/vis/geometry_map_visualisation.h"
#include "cob_3d_mapping_geometry_map/map_entry.h"



//#endif
//#include "cob_3d_mapping_geometry_map/vis/TestPlanes.h"



class GeometryMap
{
public:
//  struct MapEntry
//  {
//    //needed for 32-bit systems: see http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    unsigned int id;
//    std::vector<std::vector<Eigen::Vector3f> > polygon_world;
//    //cob_env_model::PolygonArray polygon_world;
//    //gpc_polygon polygon_plane;
//    Eigen::Vector3f normal;
//    double d;
//    Eigen::Affine3f transform_from_world_to_plane;
//    unsigned int merged;
//  };

  /*inline std::ostream& operator << (std::ostream& os, const MapEntry& m)
  {
    os << "(" << m.d << "," << m.normal << "," << ")";
    return (os);
  }*/

	//  std::ofstream outputFile;
	//  int counter_output;

//  typedef boost::shared_ptr<MapEntry> MapEntryPtr;

  // Constructor
  GeometryMap()
  :new_id_(0),
 //  counter_output(0),
   file_path_("./"),
   save_to_file_(false)

  {
	//  outputFile.open("/home/goa-hh/test.txt");

  }

  // Destructor
  ~GeometryMap()
  {
	  //outputFile.close();
  }

  void
  addMapEntry(MapEntryPtr p);

  void
  searchIntersection(MapEntry p , std::vector<int>& intersections);

  void
  mergeWithMap(MapEntryPtr p_ptr , std::vector<int> intersections);

  void
  removeMapEntry(int id);

  void
  getGpcStructure(MapEntry& p, gpc_polygon* gpc_p);

  void
  getGpcStructureUsingMap(MapEntry& p,
                          Eigen::Affine3f& transform_from_world_to_plane,
                          gpc_polygon* gpc_p);

  void
  printMapEntry(MapEntry& p);

  void
  printMap();


  void
  printGpcStructure(gpc_polygon* p);

  void
  saveMapEntry(std::string path, int ctr, MapEntry& p);

  void
  saveMap(std::string path);

  void
  clearMap();

  void
  getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
                             Eigen::Vector3f &u,
                             Eigen::Vector3f &v);

  void
  getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,
                                    const Eigen::Vector3f &origin,
                                    Eigen::Affine3f &transformation);

  void
  getPointOnPlane(const Eigen::Vector3f &normal,double d,Eigen::Vector3f &point);

  float
  rounding(float x);

  boost::shared_ptr<std::vector<MapEntryPtr> >
  getMap()
  {
    return boost::make_shared<std::vector<MapEntryPtr> >(map_);
  }

  void
  setFilePath(std::string file_path)
  {
    file_path_ = file_path;
  }

  void
  setSaveToFile(bool save_to_file)
  {
    save_to_file_ = save_to_file;
  }

protected:
  std::vector<MapEntryPtr> map_;
  unsigned int new_id_;
  std::string file_path_;
  bool save_to_file_;

};

#endif //__GEOMETRY_MAP_H__
