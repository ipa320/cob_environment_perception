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
 * Author: goa-tz
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2012
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

#ifndef CYLINDER_H_
#define CYLINDER_H_

#include "cob_3d_mapping_common/shape.h"
#include "cob_3d_mapping_common/polygon.h"

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
extern "C" {
#include "cob_3d_mapping_common/gpc.h"
}
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#ifdef PCL_VERSION_COMPARE
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/exceptions.h>

//#include <pcl/common/transformation_from_correspondences.h>




namespace cob_3d_mapping{

class Cylinder: public Polygon

/*
 * Cylinder:
 *
 * Members:		axes_		........................... 3x3 ........ Axes of the cylinder coordinate system (ccs)
 * 				axes_[1]	........................... 3x1 ........ Symmetry axis of the cylinder (ccs - y)
 * 				axes_[2]	........................... 3x1 ........ axis from origin to centroid  (ccs - z)
 * 				axis_[0]	........................... 3x1 ........ Axis completes the triad (ccs - x)
 * 				normal		........................... 3x1 ........ equivalent to axes_[2], normal of unrolled polygon
 *
 */



{

public:

	void ContoursFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud);
	void ContoursFromList( std::vector<std::vector<Eigen::Vector3f> >& in_list);

	void ParamsFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud, std::vector<int>& indices);
	void ParamsFromShapeMsg();




	void getCyl3D(std::vector<std::vector<Eigen::Vector3f> >& contours3D);
	void allocate();

	void weightAttributes(std::vector<boost::shared_ptr<Cylinder> >& c_array,Cylinder& average_c);
	void applyWeightingCylinder(std::vector<boost::shared_ptr<Cylinder> >& merge_candidates);
	void isMergeCandidate(const std::vector<boost::shared_ptr<Cylinder> >& cylinder_array,const merge_config& limits,std::vector<int>& intersections);
	void mergeCylinder(std::vector<boost::shared_ptr<Cylinder> >& c_array);






	void dbg_out(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,std::string& name);

	double r_;
	std::vector<Eigen::Vector3f> axes_;
	Eigen::Vector3f origin_;
//	Polygon unrolled_;
	bool debug_;

private:
	void getTrafo2d(const Eigen::Vector3f& vec3d, float& Tx, float& alpha);
	void getShiftedCylinder(Cylinder& c,Cylinder & shifted_Cylinder);
};



typedef boost::shared_ptr<Cylinder> CylinderPtr;

}

#endif
