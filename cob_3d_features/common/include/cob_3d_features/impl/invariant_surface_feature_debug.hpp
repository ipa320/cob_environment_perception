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
 * ROS package name: cob_3d_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Joshua Hampp, email:joshua.hampp@ipa.fraunhofer.de
 *
 * Date of creation: 03/2014
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

#pragma once

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
void cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::dbg_mesh_of_subsamp(const TVector &at, const Scalar radius, std::vector<TVector> &pts, std::vector<int> &inds) const {
      std::vector<Triangle> submap;
      subsample(at, radius, submap);
	  
	  for(size_t i=0; i<submap.size(); i++) {
		for(int j=0; j<3; j++) {
			inds.push_back((int)inds.size());
			pts.push_back(submap[i].p3_[j]);
		}
	  }
}

template<typename TSurface, typename Scalar, typename Real, typename TAffine>
pcl::PolygonMesh::Ptr cob_3d_features::InvariantSurfaceFeature<TSurface,Scalar,Real,TAffine>::dbg_triangles2mesh(const std::vector<Triangle> &res) const {
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	pcl::PointCloud<pcl::PointXYZ> points;
	for(size_t i=0; i<res.size(); i++) {
		mesh->polygons.push_back(pcl::Vertices());
		for(int j=0; j<3; j++) {
			mesh->polygons.back().vertices.push_back(points.size());
			pcl::PointXYZ pt;
			pt.x=res[i][j](0);pt.y=res[i][j](1);pt.z=res[i][j](2);
			points.push_back(pt);
		}
	}
	pcl::toROSMsg(points, mesh->cloud);
	return mesh;
}
