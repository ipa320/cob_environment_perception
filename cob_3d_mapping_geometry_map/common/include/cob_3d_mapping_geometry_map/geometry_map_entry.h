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
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_mapping_geometry_map
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

#pragma once

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
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>

namespace cob_3d_mapping {
	
/**
 * \brief Class for GeometryMapEntry
 * \details The class is an absctract representation of a Shape object of types
 * Polygon,Cylinder.
 */
class GeometryMapEntry
{
public:
	typedef boost::shared_ptr<GeometryMapEntry> Ptr;
	enum E_TYPE {TYPE_POLYGON, TYPE_CYLINDER};
	
	virtual E_TYPE getType() const = 0;
	virtual unsigned int getId() const = 0;
	
	virtual bool isMergeCandidate(const GeometryMapEntry::Ptr &o) const = 0;
	virtual bool needsCleaning(const int frame_counter, const bool persistent=true) const = 0;
	
	virtual bool merge(const GeometryMapEntry::Ptr &o) = 0;
	virtual void setMergeSettings(const cob_3d_mapping::MergeConfig &limits) = 0;
	virtual void setHeader(const unsigned int id, const int frame_count) = 0;
	virtual void colorize() = 0;
	
	virtual void save(const std::string &path) const {}
	virtual bool checkVisibility(const Eigen::Affine3f &T, const Eigen::Vector3f &camera_params, const Eigen::Vector3f &Z) {return false;}
	
	virtual operator cob_3d_mapping_msgs::Shape() const = 0;
};

/**
 * \brief Class for GeometryMapEntry_Polygon
 * \details The class is an representation of a polygon.
 */
class GeometryMapEntry_Polygon : public GeometryMapEntry
{
	Polygon::Ptr polygon_;
public:
	GeometryMapEntry_Polygon(const Polygon::Ptr &p) : polygon_(p) {}

	virtual unsigned int getId() const {return polygon_->id_;}
	virtual E_TYPE getType() const {return TYPE_POLYGON;}
	virtual bool isMergeCandidate(const GeometryMapEntry::Ptr &o) const;
	virtual bool needsCleaning(const int frame_counter, const bool persistent) const;
	virtual bool merge(const GeometryMapEntry::Ptr &o);
	virtual void setMergeSettings(const cob_3d_mapping::MergeConfig &limits);
	virtual void colorize();
	virtual void setHeader(const unsigned int id, const int frame_count) {
	  polygon_->id_ = id;
	  polygon_->frame_stamp_ = frame_count;
	}
	
	/**
	* \brief Debug output of polygon map to file.
	* \param[in] path path Name of output file.
	* \param[in] p Polygon that is saved.
	*/
	virtual void save(const std::string &path) const;
	virtual bool checkVisibility(const Eigen::Affine3f &T, const Eigen::Vector3f &camera_params, const Eigen::Vector3f &Z);
	
	virtual operator cob_3d_mapping_msgs::Shape() const {
		cob_3d_mapping_msgs::Shape s;
		toROSMsg(*polygon_,s);
		return s;
	}
};

/**
 * \brief Class for GeometryMapEntry_Cylinder
 * \details The class is an representation of a cylinder.
 */
class GeometryMapEntry_Cylinder : public GeometryMapEntry
{
	Cylinder::Ptr cylinder_;
public:
	GeometryMapEntry_Cylinder(const Cylinder::Ptr &p) : cylinder_(p) {}

	virtual unsigned int getId() const {return cylinder_->id_;}
	virtual E_TYPE getType() const {return TYPE_CYLINDER;}
	virtual bool isMergeCandidate(const GeometryMapEntry::Ptr &o) const;
	virtual bool needsCleaning(const int frame_counter, const bool persistent) const {return false;}
	virtual bool merge(const GeometryMapEntry::Ptr &o);
	virtual void setMergeSettings(const cob_3d_mapping::MergeConfig &limits);
	virtual void colorize();
	virtual void setHeader(const unsigned int id, const int frame_count) {
	  cylinder_->id_ = id;
	  cylinder_->frame_stamp_ = frame_count;
	}
	
	virtual operator cob_3d_mapping_msgs::Shape() const {
		cob_3d_mapping_msgs::Shape s;
		toROSMsg(*cylinder_,s);
		return s;
	}
};

}
