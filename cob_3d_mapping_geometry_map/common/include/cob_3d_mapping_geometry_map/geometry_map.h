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
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: cob_env_model
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2011
 *
 * \brief
 * Description: Feature Map for storing and handling geometric features
 *
 * ToDo:
 *
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

#ifndef __GEOMETRY_MAP_H__
#define __GEOMETRY_MAP_H__

// external includes

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

//cob includes
//#include "cob_3d_mapping_geometry_map/vis/geometry_map_visualisation.h"
#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_common/cylinder.h"
#include "cob_3d_mapping_common/shape_cluster.h"

/**
* \brief Class for GeometryMap
* \details The class GeometryMap handles storage of Shape objects of types
* Polygon,Cylinder,ShapeCluster.
*/
class GeometryMap
{
public:
  /**
  * \brief Polygon iterator.
  */
  typedef std::vector<cob_3d_mapping::Polygon::Ptr>::iterator polygon_iterator;


  /**
  * \brief Cylinder iterator.
  */
  typedef std::vector<cob_3d_mapping::Polygon::Ptr>::iterator cylinder_iterator;

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


  /**
  * \brief Add polygon to map.
  *
  * \details Method adds new polygon to map or initiates merge process
  * with existing polygons in map. Weighting and merging configuration are set.
  *
  * \param[in] p_ptr Polygon, that is added to map.
  */
  void addMapEntry(cob_3d_mapping::Polygon::Ptr& p_ptr);


  /**
  * \brief Add cylinder to map.
  *
  * \details Method adds new cylinder to map or initiates merge process
  * with existing cylinders in map. Weighting and merging configuration are set.
  * \param[in] c_ptr Cylinder, that is added to map.
  */
  void addMapEntry(cob_3d_mapping::Cylinder::Ptr& c_ptr);

  /**
  * \brief Add shape cluster to map.
  *
  * \details Method adds new shape cluster to map or initiates merge process
  * with existing shape clusters in map. Weighting and merging configuration are set.
  * \param[in] sc_ptr Shape Cluster, that is added to map.
  */
  //void addMapEntry(cob_3d_mapping::ShapeCluster::Ptr& sc_ptr);

  /**
  * \brief Transformation error is calculated.
  *
  * \details The error Transformation between two polygons from different
  * input frames is calculated.
  * param[in] list_polygon List of Polygons to be checked for similarity.
  * param[in] tf_old Original transformation
  * param[out] adjust_tf The correctional transformation, calculated by this function.
  */
  //bool
  //computeTfError(const std::vector<cob_3d_mapping::Polygon::Ptr>& list_polygon, const Eigen::Affine3f& tf_old, Eigen::Affine3f& adjust_tf);


  /**
  * \brief Increment frame counter.
  */
  inline void
  incrFrame() { ++frame_counter_; };



/**
* \brief Remove clutter from map.
*
* \details Geometry map is cleaned using criterias like:
* Minimal size of shape, plausible parameters, repeated detection.
*/
  void
  cleanUp();


/**
* \brief Debug output of polygon map to file.
* \param[in] path path Name of output file.
* \param[in] p Polygon that is saved.
*/
  void
  saveMapEntry(std::string path, int ctr, cob_3d_mapping::Polygon& p);

/**
* \brief Debug output of whole polygon map.
*
* \param[in] path Name of output file.
*/
  void
  saveMap(std::string path);



/**
* \brief Remove all shapes from map.
*/
  void
  clearMap();


/**
* \brief Customized rounding operation.
*/
  float
  rounding(float x);

/**
* \brief Colorize shapes in map.
*/
  void
  colorizeMap();


  /**
  * \brief Return Polygon map.
  * \return Polygon Pointer to array with Polygons contained in map.
  */
  inline std::vector<cob_3d_mapping::Polygon::Ptr>* getMapPolygon() { return &(map_polygon_); }
  /**
  * \brief Return Cylinder map.
  * \return Cylinder Pointer to array with Cylinders contained in map.
  */
  inline std::vector<cob_3d_mapping::Polygon::Ptr>* getMapCylinder() { return &(map_cylinder_); }
  /**
  * \brief Return ShapeCluster map.
  * \return ShapeCluster Pointer to array with ShapeClusters contained in map.
  */
  //inline std::vector<cob_3d_mapping::ShapeCluster::Ptr>* getMap_shape_cluster() { return &(map_shape_cluster_); }


  /**
  * \brief File path is set.
  * \param[in] file_path  Filepath
  */
  void
  setFilePath(std::string file_path)
  {
    file_path_ = file_path;
  }


  /**
  * \brief Set option if map is saved to file.
  * \param[in] save_to_file Set to true, to enable file output.
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
  * \return Latest error transformation matrix.
  */
  inline const Eigen::Affine3f& getLastError() { return last_tf_err_; }

protected:
  std::vector<cob_3d_mapping::Polygon::Ptr> map_polygon_; /**< Array containing all polygon structures. */
  std::vector<cob_3d_mapping::Polygon::Ptr> map_cylinder_; /**< Array containing all cylinder structures. */
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
