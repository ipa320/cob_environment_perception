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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_tools
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
 * ToDo:
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

#include <cob_3d_mapping_tools/point_generator.h>
#include <cob_3d_mapping_common/label_defines.h>

#include <Eigen/Geometry>

using namespace std;
using namespace cob_3d_mapping_tools;


template<typename PointT> 
PointGenerator<PointT>::PointGenerator() :
  noise_(0.0f),
  n_dist_(0.0f, 1.0f),
  n_rng_(rng_, n_dist_)
{ }

template<typename PointT> 
PointGenerator<PointT>::~PointGenerator() 
{ }

template<typename PointT> void 
PointGenerator<PointT>::generateLine(const float & step_size,
				     const Eigen::Vector3f & origin,
				     const Eigen::Vector3f & direction,
				     const uint32_t & color)
{
  if(!cloud_)
  {
    PCL_WARN("[PointGenerator<PointT>] No output cloud given!\n");
    return;
  }

  float direction_norm = direction.norm();
  
  Eigen::Vector3f step_vector;
  Eigen::Vector3f curr_point = Eigen::Vector3f::Zero();
  pcl::PointCloud<PointT> new_cloud;
  int direction_steps = round(direction_norm / step_size);
  step_vector = (direction_norm / (float)direction_steps) * direction.normalized();
  for (int step = 0; step <= direction_steps; ++step)
  {
    PointT p;
    curr_point = step * step_vector;
    p.x = curr_point.x() + origin.x() + random();
    p.y = curr_point.y() + origin.y() + random();
    p.z = curr_point.z() + origin.z() + random();
    if (rgba_index_ >= 0)
    {
      memcpy (((char *)&p) + rgba_index_, &color, sizeof(uint32_t));
    }
    new_cloud.push_back(p);

  }
  *cloud_ += new_cloud;
}

template<typename PointT> void
PointGenerator<PointT>::generateCircle(const float & step_size,
				       const Eigen::Vector3f & origin,
				       const Eigen::Vector3f & rotation_axis,
				       const Eigen::Vector3f & radius,
				       const float & angle,
				       const uint32_t & color)
{
  using namespace Eigen;

  if(!cloud_)
  {
    PCL_WARN("[PointGenerator<PointT>] No output cloud given!\n");
    return;
  }
  if( (angle > (2.0f * (float)M_PI)) || (angle < 0.0f) )
  {
    PCL_WARN("[PointGenerator<PointT>::generateCircle] invalid angle(%f)\n",angle);
    return;
  }

  pcl::PointCloud<PointT> new_cloud;
  float radius_norm = radius.norm();

  int rotation_steps = round(angle * radius_norm / step_size);

  if (rotation_steps <= 1)
  {
    PCL_WARN("[PointGenerator<PointT>::generateCircle] step_size(%f) >= angle(%f) * radius(%f)\n",
	     step_size, angle, radius_norm);
    PCL_WARN("only a single point at center was created\n");

    // single point at center:
    addSinglePoint(origin, color);
    return;
  }

  float step_angle = angle / (float)rotation_steps;
  // make sure radius vector is orthogonal to rotation_axis, but keep radius vector length:
  Vector3f axis = rotation_axis.normalized();
  Vector3f radius_save = (Matrix3f::Identity() - axis * axis.transpose()) * radius;
  radius_save = radius_save.normalized() * radius_norm;
  Quaternion<float> q;
  Vector3f curr_point = radius; // set first point
  if (angle < 2.0f * (float) M_PI) // make last step if not full circle
    rotation_steps++;
  new_cloud.reserve((size_t)rotation_steps);
  for (int step = 1; step <= rotation_steps; ++step)
  {
    PointT p;
    p.x = curr_point.x() + origin.x() + random();
    p.y = curr_point.y() + origin.y() + random();
    p.z = curr_point.z() + origin.z() + random();
    if (rgba_index_ >= 0)
    {
      memcpy (((char *)&p) + rgba_index_, &color, sizeof(uint32_t));
    }
    new_cloud.push_back(p);

    // rotate current point vector
    q = AngleAxis<float>(step * step_angle, axis);
    curr_point = q * radius;
  }
  *cloud_ += new_cloud;
}

template<typename PointT> void
PointGenerator<PointT>::generateCirclePlane(const float & step_size,
					    const Eigen::Vector3f & origin,
					    const Eigen::Vector3f & rotation_axis,
					    const float & radius)
{
  generateCirclePlane(step_size, origin, rotation_axis, 
		      getArbitraryPerpendicularNormalized(rotation_axis) * radius);
}

template<typename PointT> void
PointGenerator<PointT>::generateCirclePlane(const float & step_size,
					    const Eigen::Vector3f & origin,
					    const Eigen::Vector3f & rotation_axis,
					    const Eigen::Vector3f & radius,
					    const float & angle)
{
  float radius_norm = radius.norm();
  Eigen::Vector3f radius_step = (step_size / radius_norm) * radius;
  Eigen::Vector3f curr_radius = Eigen::Vector3f::Zero();
  while (curr_radius.norm() <= radius_norm)
  {
    generateCircle(step_size, origin, rotation_axis, curr_radius, angle, LBL_PLANE);
    curr_radius += radius_step;
  }
}

template<typename PointT> void 
PointGenerator<PointT>::generatePlane(const float & step_size, 
				      const Eigen::Vector3f & origin,
				      const Eigen::Vector3f &direction_1,
				      const Eigen::Vector3f &direction_2)
{
  Eigen::Vector3f curr_direction;
  Eigen::Vector3f direction_normalized = direction_2.normalized();
  int direction_steps = round(direction_2.norm() / step_size);
  float step_size_adjusted = direction_2.norm() / (float)direction_steps;

  for (int step = 1; step <= direction_steps; ++step)
  {
    curr_direction = step * step_size_adjusted * direction_normalized;
    generateLine(step_size, origin + curr_direction, direction_1, LBL_PLANE);
  }
}

template<typename PointT> void
PointGenerator<PointT>::generateEdge(const float & step_size, 
				     const Eigen::Vector3f & origin,
				     const Eigen::Vector3f & direction_edge,
				     const Eigen::Vector3f & direction_depth)
{
  Eigen::Vector3f direction_2;
  direction_2 = direction_edge.cross(direction_depth);
  direction_2 = direction_2.normalized() * direction_depth.norm();
  generateEdge( step_size, origin, direction_edge, direction_depth, direction_2);
}

template<typename PointT> void
PointGenerator<PointT>::generateEdge(const float & step_size,
				     const Eigen::Vector3f & origin,
				     const Eigen::Vector3f & direction_edge,
				     const Eigen::Vector3f & direction_1,
				     const Eigen::Vector3f & direction_2)
{
  Eigen::Vector3f curr_direction;
  Eigen::Vector3f direction_normalized = direction_edge.normalized();
  int direction_steps = round(direction_edge.norm() / step_size);
  float step_size_adjusted = direction_edge.norm() / (float)direction_steps;

  for (int step = 1; step <= direction_steps; ++step)
  {
    curr_direction = step * step_size_adjusted * direction_normalized;
    generateLine(step_size, origin + curr_direction + direction_1, -direction_1, LBL_EDGE);
    generateLine(step_size, origin + curr_direction + direction_2, -direction_2, LBL_EDGE);
    addSinglePoint(origin + curr_direction, LBL_EDGE);
  }
}

template<typename PointT> void
PointGenerator<PointT>::generateCorner(const float & step_size,
				       const Eigen::Vector3f & origin,
				       const Eigen::Vector3f & direction_1,
				       const Eigen::Vector3f & direction_2,
				       const Eigen::Vector3f & direction_3,
				       const float & corner_size)
{
  Eigen::Vector3f curr_direction_1;
  Eigen::Vector3f curr_direction_2;
  Eigen::Vector3f curr_direction_3;
  Eigen::Vector3f direction_1_normalized = direction_1.normalized();
  Eigen::Vector3f direction_2_normalized = direction_2.normalized();
  Eigen::Vector3f direction_3_normalized = direction_3.normalized();
  int steps;
  float step_size_adjusted, size;
  if (corner_size != 0.0f) 
    size = corner_size;
  else 
    size = direction_1.norm();
  steps = round(size / step_size);
  step_size_adjusted = size / (float)steps;

  for (int step = 1; step <= steps; ++step)
  {
    curr_direction_1 = step * step_size_adjusted * direction_1_normalized;
    curr_direction_2 = step * step_size_adjusted * direction_2_normalized;
    curr_direction_3 = step * step_size_adjusted * direction_3_normalized;
    generateLine(step_size, origin + curr_direction_1, direction_3_normalized * size, LBL_EDGE);
    generateLine(step_size, origin + curr_direction_2, direction_1_normalized * size, LBL_EDGE);
    generateLine(step_size, origin + curr_direction_3, direction_2_normalized * size, LBL_EDGE);
  }
  addSinglePoint(origin, LBL_EDGE);
}

template<typename PointT> void
PointGenerator<PointT>::generateBox(const float & step_size,
				    const Eigen::Vector3f & origin,
				    const Eigen::Vector3f & direction_1,
				    const Eigen::Vector3f & direction_2,
				    const float & corner_size,
				    const float & height)
{
  Eigen::Vector3f direction_3 = direction_1.cross(direction_2).normalized() * height;
  generateBox(step_size, origin, direction_1, direction_2, direction_3, corner_size);
}

template<typename PointT> void
PointGenerator<PointT>::generateBox(const float & step_size,
				    const Eigen::Vector3f & origin,
				    const Eigen::Vector3f & direction_1,
				    const Eigen::Vector3f & direction_2,
				    const Eigen::Vector3f & direction_3,
				    const float & corner_size)
{
  Eigen::Vector3f edge_direction_1;
  Eigen::Vector3f edge_direction_2;
  Eigen::Vector3f edge_direction_3;
  Eigen::Vector3f corner_direction_1;
  Eigen::Vector3f corner_direction_2;
  Eigen::Vector3f corner_direction_3;
  float edge_length;
  bool enable_edge_1 = true;
  bool enable_edge_2 = true;
  bool enable_edge_3 = true;
  if ((edge_length = direction_1.norm() - 2 * corner_size) > 0)
  {
    edge_direction_1 = direction_1.normalized() * (edge_length);
    corner_direction_1  = direction_1.normalized() * corner_size;
  }
  else
  {
    enable_edge_1 = false;
    corner_direction_1  = direction_1.normalized() * (direction_1.norm() / 2);
  }

  if ((edge_length = direction_2.norm() - 2 * corner_size) > 0)
  {
    edge_direction_2 = direction_2.normalized() * (edge_length);
    corner_direction_2  = direction_2.normalized() * corner_size;
  }
  else
  {
    enable_edge_2 = false;
    corner_direction_2  = direction_2.normalized() * (direction_2.norm() / 2);
  }

  if ((edge_length = direction_3.norm() - 2 * corner_size) > 0)
  {
    edge_direction_3 = direction_3.normalized() * (edge_length);
    corner_direction_3  = direction_3.normalized() * corner_size;
  }
  else
  {
    enable_edge_3 = false;
    corner_direction_3  = direction_3.normalized() * (direction_3.norm() / 2);
  }

  // Generate edges:
  if (enable_edge_1)
  {
    generateEdge(step_size, origin + corner_direction_1, 
		 edge_direction_1, corner_direction_2, corner_direction_3);
    generateEdge(step_size, origin + direction_2 + corner_direction_1, 
		 edge_direction_1, corner_direction_3, -corner_direction_2);
    generateEdge(step_size, origin + direction_3 + corner_direction_1, 
		 edge_direction_1, -corner_direction_3, corner_direction_2);
    generateEdge(step_size, origin + direction_2 + direction_3 + corner_direction_1,
		 edge_direction_1, -corner_direction_2, -corner_direction_3);
  }
  if (enable_edge_2)
  {
    generateEdge(step_size, origin + corner_direction_2, 
		 edge_direction_2, corner_direction_3, corner_direction_1);
    generateEdge(step_size, origin + direction_1 + corner_direction_2, 
		 edge_direction_2, -corner_direction_1, corner_direction_3);
    generateEdge(step_size, origin + direction_3 + corner_direction_2, 
		 edge_direction_2, corner_direction_1, -corner_direction_3);
    generateEdge(step_size, origin + direction_3 + direction_1 + corner_direction_2, 
		 edge_direction_2, -corner_direction_3, -corner_direction_1);
  }
  if (enable_edge_3)
  {
    generateEdge(step_size, origin + corner_direction_3, 
		 edge_direction_3, corner_direction_1, corner_direction_2);
    generateEdge(step_size, origin + direction_1 + corner_direction_3, 
		 edge_direction_3, corner_direction_2, -corner_direction_1);
    generateEdge(step_size, origin + direction_2 + corner_direction_3, 
		 edge_direction_3, -corner_direction_2, corner_direction_1);
    generateEdge(step_size, origin + direction_1 + direction_2 + corner_direction_3, 
		 edge_direction_3, -corner_direction_1, -corner_direction_2);
  }

  // Generate corners:
  generateCorner(step_size, origin, 
		 corner_direction_1, corner_direction_2, corner_direction_3);

  generateCorner(step_size, origin + direction_1, 
		 -corner_direction_1, corner_direction_2, corner_direction_3);
  generateCorner(step_size, origin + direction_2, 
		 corner_direction_1, -corner_direction_2, corner_direction_3);
  generateCorner(step_size, origin + direction_3, 
		 corner_direction_1, corner_direction_2, -corner_direction_3);

  generateCorner(step_size, origin + direction_1 + direction_2,
		 -corner_direction_1, -corner_direction_2, corner_direction_3);
  generateCorner(step_size, origin + direction_2 + direction_3,
		 corner_direction_1, -corner_direction_2, -corner_direction_3);
  generateCorner(step_size, origin + direction_1 + direction_3,
		 -corner_direction_1, corner_direction_2, -corner_direction_3);

  generateCorner(step_size, origin + direction_1 + direction_2 + direction_3,
		 -corner_direction_1, -corner_direction_2, -corner_direction_3);

  // Generate planes:
  if (enable_edge_1 && enable_edge_2)
  {
    generatePlane(step_size, origin + corner_direction_1 + corner_direction_2, 
		  edge_direction_1, edge_direction_2);
    generatePlane(step_size, origin + direction_3 + corner_direction_1 + corner_direction_2, 
		  edge_direction_1, edge_direction_2);
  }
  if (enable_edge_2 && enable_edge_3)
  {
    generatePlane(step_size,origin + corner_direction_2 + corner_direction_3,
		  edge_direction_2, edge_direction_3);
    generatePlane(step_size, origin + direction_1 + corner_direction_2 + corner_direction_3,
		  edge_direction_2, edge_direction_3);
  }
  if (enable_edge_3 && enable_edge_1)
  {
    generatePlane(step_size, origin + corner_direction_3 + corner_direction_1,
		  edge_direction_3, edge_direction_1);
    generatePlane(step_size, origin + direction_2 + corner_direction_3 + corner_direction_1,
		  edge_direction_3, edge_direction_1);
  }
}

template<typename PointT> void
PointGenerator<PointT>::generateCylinder(const float & step_size,
					 const Eigen::Vector3f & origin,
					 const Eigen::Vector3f & direction,
					 const float & radius)
{
  Eigen::Vector3f radius_vector = getArbitraryPerpendicularNormalized(direction) * radius;
  generateCylinder(step_size, origin, direction, radius_vector, 2*M_PI);
}

template<typename PointT> void
PointGenerator<PointT>::generateCylinder(const float & step_size,
					 const Eigen::Vector3f & origin,
					 const Eigen::Vector3f & direction,
					 const Eigen::Vector3f & radius, 
					 const float & angle)
{
  using namespace Eigen;

  int rotation_steps = round(angle * radius.norm() / step_size);
  float step_angle = angle / (float)rotation_steps;
  Vector3f curr_point;
  Vector3f direction_normalized = direction.normalized();

  curr_point = radius;  
  Quaternion<float> q;
  for(int step = 1; step <= rotation_steps; ++step)
  {
    generateLine(step_size, origin + curr_point, direction, LBL_CYL);
    q = AngleAxis<float>(step * step_angle, direction_normalized);
    curr_point = q * radius;
  }
}

template<typename PointT> void
PointGenerator<PointT>::generateCone(const float & step_size,
				     const Eigen::Vector3f & origin,
				     const Eigen::Vector3f & direction,
				     const float & radius_base,
				     const float & radius_peek)
{
  Eigen::Vector3f direction_step;
  Eigen::Vector3f radius_step;
  Eigen::Vector3f curr_direction = Eigen::Vector3f::Zero(); // direction vector iterator
  Eigen::Vector3f curr_radius; // radius vector iterator, start at base radius

  float direction_norm = direction.norm();
  float side_length = sqrt (pow (radius_base - radius_peek, 2) + pow (direction_norm, 2));
  int steps = round(side_length / step_size);
  direction_step = direction_norm / (float)steps * direction.normalized();
  curr_radius = getArbitraryPerpendicularNormalized(direction);
  radius_step = ((radius_base - radius_peek) / (float)steps) * curr_radius;
  curr_radius = curr_radius * radius_base;

  for (int step = 0; step <=  steps; ++step)
  {
    generateCircle(step_size, origin + curr_direction, direction, curr_radius, 2 * M_PI, LBL_CYL);
    curr_direction += direction_step;
    curr_radius -= radius_step;
  }
}

template<typename PointT> void
PointGenerator<PointT>::generateSphere(const float & step_size,
				       const Eigen::Vector3f & center,
				       const Eigen::Vector3f & rotation_axis,
				       const float & radius,
				       const float & angle)
{
  if( (angle > ((float)M_PI)) || (angle < 0.0f) )
  {
    PCL_WARN("[PointGenerator<PointT>::generateSphere] invalid angle(%f)\n",angle);
    PCL_WARN("0.0 < angle <= PI\n");
    return;
  }
  int radius_steps = round(angle * radius / step_size);
  float step_angle = angle / (float)radius_steps;
  Eigen::Vector3f curr_radius;
  Eigen::Vector3f curr_point;
  Eigen::Vector3f rotation_axis_vector = rotation_axis.normalized() * radius;
  Eigen::Vector3f radius_vector = getArbitraryPerpendicularNormalized(rotation_axis) * radius;
  
  for (int step = 0; step <= radius_steps; ++step)
  {
    curr_radius = radius_vector * sin((float)step * step_angle);
    curr_point = rotation_axis_vector * cos((float)step * step_angle);
    generateCircle(step_size, center + curr_point, rotation_axis_vector, curr_radius,
		   2 * M_PI, LBL_SPH);
  } 
}

template<typename PointT> void
PointGenerator<PointT>::generateSphere(const float & step_size,
				       const Eigen::Vector3f & center,
				       const float & radius)
{
  generateSphere(step_size, center, Eigen::Vector3f::UnitY(), radius);
}

template<typename PointT> void
PointGenerator<PointT>::generateHandle(const float & step_size,
				       const Eigen::Vector3f & origin,
				       const Eigen::Vector3f & rotation_axis,
				       const Eigen::Vector3f & radius_curvature,
				       const float & radius_handle,
				       const float & angle)
{
  int radius_steps = round(2 * M_PI * radius_handle / step_size);
  float step_angle = 2 * M_PI / (float)radius_steps;
  Eigen::Vector3f curr_radius;
  Eigen::Vector3f vector_radius_handle = radius_curvature.normalized() * radius_handle;
  Eigen::Vector3f rotation_axis_radius_handle = radius_curvature.cross(rotation_axis);
  rotation_axis_radius_handle = rotation_axis_radius_handle.normalized();

  Eigen::Quaternion<float> q;
  for (int step = 0; step <= radius_steps; ++step)
  {
    q = Eigen::AngleAxis<float>((float)step * step_angle, rotation_axis_radius_handle);
    curr_radius = q * vector_radius_handle;
    generateCircle(step_size, origin, rotation_axis, radius_curvature + curr_radius, angle, LBL_EDGE);
  }
}

template<typename PointT> void
PointGenerator<PointT>::addSinglePoint(const Eigen::Vector3f & point, const uint32_t & color)
{
  PointT p;
  p.x = point.x() + random();
  p.y = point.y() + random();
  p.z = point.z() + random();
  if (rgba_index_ >= 0)
  {
    memcpy (((char *)&p) + rgba_index_, &color, sizeof(uint32_t));
  }
  pcl::PointCloud<PointT> pc;
  pc.push_back(p);
  *cloud_ += pc;
}

template class PointGenerator<pcl::PointXYZ>;
template class PointGenerator<pcl::PointXYZRGBA>;
