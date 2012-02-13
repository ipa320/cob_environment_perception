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

#ifndef COB_3D_MAPPING_TOOLS_POINT_GENERATOR_H_
#define COB_3D_MAPPING_TOOLS_POINT_GENERATOR_H_

#include <math.h>
#include <Eigen/StdVector>
#include <boost/random.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>


namespace cob_3d_mapping_tools
{
  /*! @brief Point cloud generator for simple shapes 
   *
   * Provides a set of functions to add synthetic shapes to a existing point cloud. 
   * Combining some functions allows to create whole artificial scenes.
   */
  template<typename PointT> class PointGenerator
  {
  public:
    /*! Empty constructor */
    PointGenerator();
    /*! Empty destructor */
    ~PointGenerator();

    // public typedefs
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef boost::shared_ptr<PointCloud> PointCloudPtr;
    typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

    /*! @brief Set the pointer to the output point cloud
     *
     * @param[out] cloud pointer to the output point cloud
     */
    inline void setOutputCloud(PointCloudPtr & cloud)
    {
      std::vector<sensor_msgs::PointField> fields;
      rgba_index_ = -1;
      rgba_index_ = pcl::getFieldIndex(*cloud, "rgba", fields);
      if (rgba_index_ >= 0)
	rgba_index_ = fields[rgba_index_].offset;

      cloud_ = cloud;
    }

    /*! @brief Define a Gaussian noise on the shapes to be generated
     *
     * @param[in] sigma strength of noise in meters
     */
    inline void setGaussianNoise(const float & sigma)
    {
      noise_ = sigma;
    }

    /*! @brief Generates points in a line
     *
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the starting point of the line
     * @param[in] direction the direction relative from the origin pointing to the end of the line
     * @param[in] color the color assigned to the generated points (default: white)
     */
    void generateLine(const float & step_size,
		      const Eigen::Vector3f & origin, 
		      const Eigen::Vector3f & direction,
		      const uint32_t & color = 0xffffff);

    /*! @brief Generates points in a circle
     *
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the center of the circle
     * @param[in] rotation_axis a vector defining the axis the circle is rotating around
     * @param[in] radius a vector perpendicular to @a rotation_axis. Its length defines 
     *    the size of the circle.
     * @param[in] angle define a angle (in rad) to generate an arc starting at the radius vector.
     * @param[in] color the color assigned to the generated points (default: white)
     */
    void generateCircle(const float & step_size,
			const Eigen::Vector3f & origin,
			const Eigen::Vector3f & rotation_axis,
			const Eigen::Vector3f & radius,
			const float & angle = 2 * M_PI,
			const uint32_t & color = 0xffffff);

    /*! @brief Generates a circle shaped plane
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the chenter of the circle plane
     * @param[in] rotation_axis a vector defining the axis the cicle is rotating around
     * @param[in] radius defining the size of the circle in meters
     */
    void generateCirclePlane(const float & step_size,
			     const Eigen::Vector3f & origin,
			     const Eigen::Vector3f & rotation_axis,
			     const float & radius);

    /*! @brief Generates a filled arc 
     *
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the center of the arc
     * @param[in] rotation_axis a vector defining the axis the circle is rotating around
     * @param[in] radius a vector perpendicular to @a rotation_axis. Its length defines 
     *    the size of the circle.
     * @param[in] angle define a angle (in rad) to generate an arc starting at the radius vector.
     */
    void generateCirclePlane(const float & step_size,
			     const Eigen::Vector3f & origin,
			     const Eigen::Vector3f & rotation_axis,
			     const Eigen::Vector3f & radius,
			     const float & angle = 2 * M_PI);

    /*! @brief Generates quadrangle plane
     * 
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the starting edge of the plane
     * @param[in] direction_1 the first direction vector relative to the @a origin
     * @param[in] direction_2 the second direction vector relative to the @a origin
     */
    void generatePlane(const float & step_size,
		       const Eigen::Vector3f & origin,
		       const Eigen::Vector3f & direction_1,
		       const Eigen::Vector3f & direction_2);

    /*! @brief Generates an edge shaped from two perpendicular planes
     *
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the starting point of the edge
     * @param[in] direction_edge the direction vector pointing to the end point of the edge
     * @param[in] direction_depth the vector defining the direction and length of the first of 
     *     the planes. The other plane's direction is determined from the cross product of both.
     */
    void generateEdge(const float & step_size,
		      const Eigen::Vector3f & origin,
		      const Eigen::Vector3f & direction_edge,
		      const Eigen::Vector3f & direction_depth);

    /*! @brief Generates an edge shaped by two intersecting planes 
     *
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the starting point of the edge
     * @param[in] direction_edge the direction vector pointing to the end point of the edge
     * @param[in] direction_1 ...
     * @param[in] direction_2 ...
     */
    void generateEdge(const float & step_size,
		      const Eigen::Vector3f & origin,
		      const Eigen::Vector3f & direction_edge,
		      const Eigen::Vector3f & direction_1,
		      const Eigen::Vector3f & direction_2);

    /*! @brief Generates a corner 
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the starting point of the corner
     * @param[in] direction_1 the first direction vector relative to the origin
     * @param[in] direction_2 the second direction vector relative to the origin
     * @param[in] direction_3 the third direction vector relative to the origin
     * @param[in] corner_size if not set, the length of @a direction_1 is used as the size
     */
    void generateCorner(const float & step_size,
			const Eigen::Vector3f & origin,
			const Eigen::Vector3f & direction_1,
			const Eigen::Vector3f & direction_2,
			const Eigen::Vector3f & direction_3,
			const float & corner_size = 0.0f);

    /*! @brief Generates a box
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the starting edge of the box
     * @param[in] direction_1 the first direction vector 
     * @param[in] direction_2 the second direction vector
     * @param[in] corner_size the size how big the corners should be labeled
     * @param[in] height sets the size of the third direction, defined by the cross product 
     *     of the first two vectors
     */
    void generateBox(const float & step_size,
		     const Eigen::Vector3f & origin,
		     const Eigen::Vector3f & direction_1,
		     const Eigen::Vector3f & direction_2,
		     const float & corner_size,
		     const float & height);

    /*! @brief Generates a box
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the starting edge of the box
     * @param[in] direction_1 the first direction vector
     * @param[in] direction_2 the second direction vector
     * @param[in] direction_3 the third direction vector
     * @param[in] corner_size the size how big the corners should be labeled
     */
    void generateBox(const float & step_size,
		     const Eigen::Vector3f & origin,
		     const Eigen::Vector3f & direction_1,
		     const Eigen::Vector3f & direction_2,
		     const Eigen::Vector3f & direction_3,
		     const float & corner_size);

    /*! @brief Generates a full cylinder
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the center of bottom
     * @param[in] direction the rotation axis of the cylinder, the length determines its height
     * @param[in] radius the radius of the cylinder
     */
    void generateCylinder(const float & step_size,
			  const Eigen::Vector3f & origin,
			  const Eigen::Vector3f & direction,
			  const float & radius);

    /*! @brief Generates a cylinder
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the center of bottom
     * @param[in] direction the rotation axis of the cylinder, the length determines its height
     * @param[in] radius the radius of the cylinder
     * @param[in] angle the angle starting from the radius vector
     */
    void generateCylinder(const float & step_size,
			  const Eigen::Vector3f & origin,
			  const Eigen::Vector3f & direction,
			  const Eigen::Vector3f & radius, 
			  const float & angle);

    /*! @brief Generates a full cone
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the center of bottom
     * @param[in] radius_base radius at the bottom
     * @param[in] radius_peak radius at the top
     */
    void generateCone(const float & step_size,
		      const Eigen::Vector3f & origin,
		      const Eigen::Vector3f & direction,
		      const float & radius_base,
		      const float & radius_peak = 0.0f);

    /*! @brief Generates a hemisphere
     * @param[in] step_size the distance between each generated point
     * @param[in] center of the sphere
     * @param[in] rotation_axis the rotation axis vector
     * @param[in] radius the size of the hemisphere 
     * @param[in] angle the height of the hemisphere
     */    
    void generateSphere(const float & step_size,
			const Eigen::Vector3f & center,
			const Eigen::Vector3f & rotation_axis,
			const float & radius,
			const float & angle = M_PI);

    /*! @brief Generates a full sphere
     * @param[in] step_size the distance between each generated point
     * @param[in] center the center of the sphere
     * @param[in] radius the radius of the sphere in meters
     */
    void generateSphere(const float & step_size,
			const Eigen::Vector3f & center,
			const float & radius);

    /*! @brief Generates a handle (or a torus)
     * @param[in] step_size the distance between each generated point
     * @param[in] origin the center of the torus
     * @param[in] rotation_axis the rotation axis of the torus
     * @param[in] radius_curvature the size of the torus
     * @param[in] radius_handle the thickness of the torus
     * @param[in] angle the rotation angle (in rad) starting at @a radius_curvature vector
     */
    void generateHandle(const float & step_size,
			const Eigen::Vector3f & origin,
			const Eigen::Vector3f & rotation_axis,
			const Eigen::Vector3f & radius_curvature,
			const float & radius_handle,
			const float & angle);

    inline float random()
    {
      return (noise_ * n_rng_());
    }
			

  protected:
    inline int round(float f)
    {
      return f<0?f-.5:f+.5;
    }

    inline Eigen::Vector3f randomVector()
    {
      return (Eigen::Vector3f(random(), random(), random()));
    }

    /*! @brief produces a random normalized arbitrary prependicular vector
     *
     * @param[in] vector input vector
     * @return a random vector perpendicular to @a vector
     */
    inline Eigen::Vector3f getArbitraryPerpendicularNormalized(const Eigen::Vector3f & vector)
    {
      Eigen::Vector3f ret = vector.cross(Eigen::Vector3f::UnitZ());
      if (ret == Eigen::Vector3f::Zero())
	ret = vector.cross(Eigen::Vector3f::UnitY());
      return ret.normalized();
    }

    void addSinglePoint(const Eigen::Vector3f & point, const uint32_t & color = 0xffffff);

    PointCloudPtr cloud_;
    
    int rgba_index_;
    float noise_;

    // create general pseudo random number series with Mersenne Twister Algorithm
    boost::mt19937 rng_;
    // normal distribution (initialized in constructor)
    boost::normal_distribution<float> n_dist_;
    // connect distribution with random series
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > n_rng_;
  };
}

#endif // COB_3D_MAPPING_TOOLS_POINT_GENERATOR_H_
