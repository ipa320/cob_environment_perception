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
  template<typename PointT> class PointGenerator
  {
  public:
    PointGenerator();

    ~PointGenerator();

    // public typedefs
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef boost::shared_ptr<PointCloud> PointCloudPtr;
    typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

    inline void setOutputCloud(PointCloudPtr & cloud)
    {
      std::vector<sensor_msgs::PointField> fields;
      rgba_index_ = -1;
      rgba_index_ = pcl::getFieldIndex(*cloud, "rgba", fields);
      if (rgba_index_ >= 0)
	rgba_index_ = fields[rgba_index_].offset;

      cloud_ = cloud;
    }

    inline void setGaussianNoise(const float & sigma)
    {
      noise_ = sigma;
    }

    void generateLine(const float & step_size,
		      const Eigen::Vector3f & origin, 
		      const Eigen::Vector3f & direction,
		      const uint32_t & color = 0xffffff);
    void generateCircle(const float & step_size,
			const Eigen::Vector3f & origin,
			const Eigen::Vector3f & rotation_axis,
			const Eigen::Vector3f & radius,
			const float & angle = 2 * M_PI,
			const uint32_t & color = 0xffffff);
    void generateCirclePlane(const float & step_size,
			     const Eigen::Vector3f & origin,
			     const Eigen::Vector3f & rotation_axis,
			     const float & radius);
    void generateCirclePlane(const float & step_size,
			     const Eigen::Vector3f & origin,
			     const Eigen::Vector3f & rotation_axis,
			     const Eigen::Vector3f & radius,
			     const float & angle = 2 * M_PI);
    void generatePlane(const float & step_size,
		       const Eigen::Vector3f & origin,
		       const Eigen::Vector3f & direction_1,
		       const Eigen::Vector3f & direction_2);
    void generateEdge(const float & step_size,
		      const Eigen::Vector3f & origin,
		      const Eigen::Vector3f & direction_edge,
		      const Eigen::Vector3f & direction_depth);
    void generateEdge(const float & step_size,
		      const Eigen::Vector3f & origin,
		      const Eigen::Vector3f & direction_edge,
		      const Eigen::Vector3f & direction_1,
		      const Eigen::Vector3f & direction_2);
    void generateCorner(const float & step_size,
			const Eigen::Vector3f & origin,
			const Eigen::Vector3f & direction_1,
			const Eigen::Vector3f & direction_2,
			const Eigen::Vector3f & direction_3,
			const float & corner_size = 0.0f);
    void generateBox(const float & step_size,
		     const Eigen::Vector3f & origin,
		     const Eigen::Vector3f & direction_1,
		     const Eigen::Vector3f & direction_2,
		     const float & corner_size,
		     const float & height);
    void generateBox(const float & step_size,
		     const Eigen::Vector3f & origin,
		     const Eigen::Vector3f & direction_1,
		     const Eigen::Vector3f & direction_2,
		     const Eigen::Vector3f & direction_3,
		     const float & corner_size);
    void generateCylinder(const float & step_size,
			  const Eigen::Vector3f & origin,
			  const Eigen::Vector3f & direction,
			  const float & radius);
    void generateCylinder(const float & step_size,
			  const Eigen::Vector3f & origin,
			  const Eigen::Vector3f & direction,
			  const Eigen::Vector3f & radius, 
			  const float & angle);
    void generateCone(const float & step_size,
		      const Eigen::Vector3f & origin,
		      const Eigen::Vector3f & direction,
		      const float & radius_base,
		      const float & radius_peak = 0.0f);
    void generateSphere(const float & step_size,
			const Eigen::Vector3f & center,
			const Eigen::Vector3f & rotation_axis,
			const float & radius,
			const float & angle = M_PI);
    void generateSphere(const float & step_size,
			const Eigen::Vector3f & center,
			const float & radius);
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
