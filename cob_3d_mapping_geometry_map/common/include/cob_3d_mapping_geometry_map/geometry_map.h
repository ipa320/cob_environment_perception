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

#include "cob_3d_mapping_geometry_map/vis/geometry_map_visualisation.h"
#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_common/cylinder.h"
#include "cob_3d_mapping_common/shape_cluster.h"


class GeometryMap
{
public:
  typedef std::vector<cob_3d_mapping::Polygon::Ptr>::iterator polygon_iterator;
  typedef std::vector<cob_3d_mapping::Polygon::Ptr>::iterator cylinder_iterator;

    // Constructor
  /**
  * \brief Constructor for geometry map object.
  */
  GeometryMap()
    : new_id_(0)
    , frame_counter_(0)
    , file_path_("./")
    , save_to_file_(false)
    , cos_angle_(0.97)
    , d_(0.01)
    , last_tf_err_(Eigen::Affine3f::Identity())
  {
  }

  /**
  * \brief Destructor for geometry map object.
  */
  ~GeometryMap()
  {
  }

  void addMapEntry(cob_3d_mapping::Polygon::Ptr& p_ptr);
  void addMapEntry(cob_3d_mapping::Cylinder::Ptr& c_ptr);
  void addMapEntry(cob_3d_mapping::ShapeCluster::Ptr& sc_ptr);

  bool
  computeTfError(const std::vector<cob_3d_mapping::Polygon::Ptr>& list_polygon, const Eigen::Affine3f& tf_old, Eigen::Affine3f& adjust_tf);


  /**
  * \brief Increment frame counter.
  */
  inline void
  incrFrame() { ++frame_counter_; };

  void
  cleanUp();




  void
  saveMapEntry(std::string path, int ctr, cob_3d_mapping::Polygon& p);

  void
  saveMap(std::string path);

  void
  clearMap();


  float
  rounding(float x);

  void
  colorizeMap();


  /**
  * \brief Return polygon map.
  */  
  inline std::vector<cob_3d_mapping::Polygon::Ptr>* getMap_polygon() { return &(map_polygon_); }
  /**
  * \brief Return polygon map.
  */  
  inline std::vector<cob_3d_mapping::Cylinder::Ptr>* getMap_cylinder() { return &(map_cylinder_); }
  /**
  * \brief Return polygon map.
  */  
  inline std::vector<cob_3d_mapping::ShapeCluster::Ptr>* getMap_shape_cluster() { return &(map_shape_cluster_); }


  /**
  * \brief Fiele path is set.
  * \param[in] file_path  Filepath
  */
  void
  setFilePath(std::string file_path)
  {
    file_path_ = file_path;
  }

  /**
  * \brief Set option if map is saved to file.
  */
  void
  setSaveToFile(bool save_to_file)
  {
    save_to_file_ = save_to_file;
  }

  /**
  * \brief Set merging thresholds.
  * \param[in] cos_angle Angular limit.
  * \param[in] d Distance limit.
  */
  void
  setMergeThresholds(double cos_angle, double d)
  {
    cos_angle_ = cos_angle;
    d_ = d;
  }

  /**
  * \brief Last transformation error is returned.
  */
  inline const Eigen::Affine3f& getLastError() { return last_tf_err_; }

protected:
  std::vector<cob_3d_mapping::Polygon::Ptr> map_polygon_; /**< Array containing all polygon structures. */
  std::vector<cob_3d_mapping::Cylinder::Ptr> map_cylinder_; /**< Array containing all cylinder structures. */
  std::vector<cob_3d_mapping::ShapeCluster::Ptr> map_shape_cluster_; /**< Array containing all shape clusters. */
  unsigned int new_id_; /**< Counter for shape IDs*/
  //int counter_output;
  int frame_counter_; /**< Counter for frame_stamp of shapes. */
  std::string file_path_; /**< Path for file output. */
  bool save_to_file_; /**< Boolean for file output. */
  double cos_angle_; /**< Angle limit, used during merging. */
  double  d_; /**< Distance threshold, used during merging. */
  Eigen::Affine3f last_tf_err_; /**< Transformation error. */
};

#endif //__GEOMETRY_MAP_H__
