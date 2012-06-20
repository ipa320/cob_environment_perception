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
#include "cob_3d_mapping_common/cylinder.h"

namespace cob_3d_mapping {




void Cylinder::roll() {

	//Transform to local coordinate system
	Polygon poly_plane;

	for (size_t j = 0; j < unrolled_.contours.size(); j++) {

		poly_plane.holes.resize(unrolled_.contours.size());
		poly_plane.contours.resize(unrolled_.contours.size());

		for (size_t k = 0; k < unrolled_.contours[j].size(); k++) {
			poly_plane.contours[j].resize(unrolled_.contours[j].size());
			//				std::cout<<"DEBUG: point before trafo\n"<<unrolled_.contours[j][k]<<std::endl;

			//		      Eigen::Vector3f point_trans = transformation_from_world_to_cylinder_*unrolled_.contours[j][k];
			Eigen::Vector3f point_trans =
					unrolled_.transform_from_world_to_plane
					* unrolled_.contours[j][k];

			poly_plane.contours[j][k] = point_trans;

		}
	}

	// transform into cylinder shape via polar coordinates
	for (size_t j = 0; j < poly_plane.contours.size(); j++) {

		contours.resize(poly_plane.contours.size());
		holes.resize(poly_plane.contours.size());

		for (size_t k = 0; k < poly_plane.contours[j].size(); k++) {

			contours[j].resize(poly_plane.contours[j].size());
			float alpha;
			Eigen::Vector3f point_temp;
			//		      alpha= B/r
			//			alpha = unrolled_.contours[j][k][0] / r_;
			alpha = poly_plane.contours[j][k][0] / r_;


			//				 use polar coordinates to create cylinder points
			point_temp << r_ * sin(alpha), poly_plane.contours[j][k][1], r_	* cos(alpha);


			//	      transform back in world system
			//			point_temp=transformation_from_world_to_cylinder_.inverse()* point_temp;
			point_temp = unrolled_.transform_from_world_to_plane.inverse()
							* point_temp;

			//			std::cout<<"DEBUG: point after trafo\n"<<point_temp<<std::endl;

			contours[j][k] = point_temp;
			//		  		    std::cout<<std::endl;


		}
	}

}

void Cylinder::unroll() {

	//calculate transformation from world to cylinder

	this->unrolled_.assignMembers(axes_[1], axes_[2], origin_);

	//	configure unroilled polygon
	unrolled_.contours.resize(contours.size());
	//		unrolled_.transform_from_world_to_plane=transformation_from_world_to_cylinder_;
	for (size_t j = 0; j < contours.size(); j++) {

		unrolled_.contours[j].resize(contours[j].size());
		unrolled_.holes.resize(contours[j].size());

		for (size_t k = 0; k < contours[j].size(); k++) {

			//		  Transform  Points in Cylinder Coordinate System
			Eigen::Vector3f point_trans =
					unrolled_.transform_from_world_to_plane * contours[j][k];

			float Tx, alpha;
			//	      flatten polygon
			getTrafo2d(point_trans, Tx, alpha);
			// New coordinates( p_y = 0 because now on a plane)
			point_trans[0] = Tx;
			point_trans[2] = 0;
			//	      	std::cout<<"point _trans\n"<< point_trans<<std::endl;
			//	      transform back in world system
			Eigen::Vector3f point_global =
					unrolled_.transform_from_world_to_plane.inverse()
					* point_trans;
			//	      	std::cout<<"point _trans\n"<< point_global<<std::endl;


			unrolled_.contours[j][k] = point_global;

		}
	}




}

void Cylinder::getTrafo2d(const Eigen::Vector3f& vec3d, float& Tx, float& alpha) {

	//	calculation of translation Tx and Ty
	//
	//	Tx by meansof arc length
	//	Tx corresponds to radius of cylinder
	float cos_alpha;
	Eigen::Vector2f vec2d, z_axis;

	if (debug_ == true) {
		//	Debug Output
		std::cout << "Point cylinder:" << std::endl << vec3d << std::endl;
	}

	//x and z components of point in local system
	vec2d << vec3d[0], vec3d[2];
	//y-axis in local system
	z_axis << 0, 1;

	//	angle between y-axis and point-origin connection
	cos_alpha = vec2d.dot(z_axis) / (vec2d.norm() * z_axis.norm());
	alpha = (acos(cos_alpha));

	//  calculation of arc length
	if (vec2d[0] < 0) {
		Tx = -(r_ * alpha);
	} else {
		Tx = r_ * alpha;

	}

	if (debug_ == true) {
		//	Debug Output
		std::cout << "avec" << std::endl << vec2d << std::endl;
		std::cout << "alpha = " << acos(cos_alpha) * (180 / 3.1459)
						<< std::endl;
		std::cout << "TX = " << Tx << std::endl << std::endl;

	}

}



void Cylinder::isMergeCandidate(const std::vector<CylinderPtr>& cylinder_array,
		const merge_config& limits, std::vector<int>& intersections) {

	this->unroll();

	for (size_t i = 0; i < cylinder_array.size(); i++) {

		Cylinder& c_map = *(cylinder_array[i]);
		//	   std::cout<<"criteria"<< fabs(c_map.axes_[3] .dot(axes_[3]))<<" "<<c_map.r_-r_;


		//compare symmetry axes and d
		if ((fabs(c_map.axes_[1] .dot(axes_[1])) > limits.angle_thresh))

			//	    	std::cout<<"transformed origins"<<std::endl<<transformation_from_world_to_cylinder_*origin_<<std::endl<<std::endl;
			//			std::cout<<transformation_from_world_to_cylinder_*c_map.origin_<<std::endl;

		{

			//	    	prepare cylinder
			c_map.unroll();
			std::cout << "merge criteria fulfilled" << std::endl;
			Polygon shifted_polygon_map;

			c_map.getShiftedPolygon(*this, shifted_polygon_map);
			//				shifted_polygon_map.debug_output("shifted Polygon");
			//				this->unrolled_.debug_output("# THis unrolled");
			//				std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;

			unrolled_.debug_output("merge A");
			//			shifted_polygon_map.debug_output("SPM");
			c_map.unrolled_.debug_output("merge B");

			//			double area_B =c_map.unrolled_.computeArea();
			//			std::cout<<"AREA MERGED"<<area_B<<std::endl;
			////
			//			double area_A =unrolled_.computeArea();
			//			std::cout<<"AREA MERGED"<<area_A<<std::endl;

			bool is_intersected = this->unrolled_.isMergeCandidate_intersect(
					shifted_polygon_map);

			if (is_intersected == false) {
				//std::cout << "no intersection with map " << i << std::endl;
				//std::cout << p.normal << std::endl;
				continue;
			}
			if (is_intersected == true) {
				intersections.push_back(i);

			}
			// std::cout << "intersection with map " << i << std::endl;

		}//if

	}//for

}

void Cylinder::merge(std::vector<CylinderPtr>& c_array) {

	//varibles for averaging new cylinder attributes
	Cylinder average_cyl;
	weightAttributes(c_array, average_cyl);

	std::vector<PolygonPtr> merge_polygons_B;

	//	transform unrolled_ to local system


	for (int i = 0; i < (int) c_array.size(); i++) {
		Cylinder & c_map = *(c_array[i]);

		PolygonPtr map_shifted_polygon = PolygonPtr(new Polygon());

		//	get shifted polygons with respect to THIS


		c_map.getShiftedPolygon(*this, *map_shifted_polygon);



		merge_polygons_B.push_back(map_shifted_polygon);


	}//for


	//	polygon operation for merging
	//	Polygon & p_av=*merge_polygons_B[0];

	//	unrolled_.merge_union(merge_polygons_B,average_cyl.unrolled_);
	unrolled_.merge_union(merge_polygons_B,unrolled_);



	Polygon& merge_polygon_C = *merge_polygons_B[0];


	Cylinder& c_map2 = *c_array[0];
	//	assign values to resulting cylinder
	c_map2.unrolled_ = merge_polygon_C;
	c_map2.r_ = average_cyl.r_;
	c_map2.axes_ =axes_;
	c_map2.origin_ = average_cyl.origin_;
	c_map2.unrolled_.transform_from_world_to_plane
	= unrolled_.transform_from_world_to_plane;
	c_map2.roll();
	c_map2.axes_=average_cyl.axes_;
	c_map2.unroll();


	c_map2.unrolled_.debug_output("unrolled average");



	for (int i = 0; i < (int) c_array.size(); ++i) {

		if (i != 0) {
			c_array.erase(c_array.begin() + i);
		}

	}

}

void

Cylinder::getShiftedPolygon(Cylinder& c, Polygon & shifted_polygon) {

	//	    	Transform normal of map polygon in cylinder system of THIS

	//	LOCAL
	Eigen::Vector3f transformed_normal =
			c.unrolled_.transform_from_world_to_plane.rotation()
			* unrolled_.normal;

	//		      calculate trafo parameters
	float x_shift, z_shift, alpha;
	Eigen::Vector3f temp_vec;

	temp_vec = (c.origin_ - origin_);
	z_shift = temp_vec.norm();

	getTrafo2d(transformed_normal, x_shift, alpha);

	Eigen::Affine3f shift_trafo;
	pcl::getTransformation(x_shift, 0, z_shift, 0, alpha, 0, shift_trafo);

	float roll, pitch, yaw, x, y, z;
	pcl::getTranslationAndEulerAngles(shift_trafo, x, y, z, roll, pitch, yaw);

	//						std::cout<<"2d trafo x= "<<x<<" y= "<<z<<" z= "<<z<<" roll= "<<roll<<" pitch= "<<pitch<<" yaw= "<<yaw<<std::endl;
	shifted_polygon.contours.resize(contours.size());
	shifted_polygon.holes.resize(contours.size());

	for (size_t j = 0; j < contours.size(); j++) {
		shifted_polygon.contours[j].resize(contours[j].size());

		for (size_t k = 0; k < contours[j].size(); k++) {
			//		  Transform  Points in Cylinder Coordinate System


			shifted_polygon.contours[j][k]		= c.unrolled_.transform_from_world_to_plane.inverse()
									* (shift_trafo
											* c.unrolled_.transform_from_world_to_plane
											* unrolled_.contours[j][k]);
			//END LOCAL

		}
	}

	shifted_polygon.assignMembers(c.axes_[1], c.axes_[2], origin_);



}

void Cylinder::weightAttributes(std::vector<CylinderPtr>& c_array,
		Cylinder& average_c) {

	// weight cylinder entries according to their merged counter

	//	first current cylinder entry
	average_c.r_ = r_ * merged;

	for (int axis_runner = 0; axis_runner < 3; ++axis_runner) {
		average_c.axes_.push_back(axes_[axis_runner] * merged);
	}
	average_c.merged = merged;
	average_c.origin_ = origin_;

	//		std::cout<<"axe_this"<<std::endl<<axes_[1]<<std::endl;
	//		secondly all merge candidates

	for (int array_runner = 0; array_runner < (int) c_array.size(); ++array_runner) {

		Cylinder & c_map = *c_array[array_runner];

		average_c.r_ += c_map.merged * c_map.r_;
		for (int axis_runner = 0; axis_runner < 3; ++axis_runner) {
			average_c.axes_[axis_runner] += c_map.axes_[axis_runner]
			                                            * c_map.merged;

		}
		average_c.merged += c_map.merged;

		average_c.origin_ += c_map.origin_;

		//			std::cout<<"axe_this"<<std::endl<<c_map.axes_[1]<<std::endl;


	}

	//		divide by merge counter
	average_c.r_ /= average_c.merged;
	for (int axis_runner = 0; axis_runner < 3; ++axis_runner) {
		average_c.axes_[axis_runner] /= average_c.merged;

	}

	//		Origin is not weighted !!


	average_c.origin_ /= average_c.merged;

	average_c.unrolled_.assignMembers(average_c.axes_[1], average_c.axes_[2],
			average_c.origin_);

	//calculate transformation from world to cylinder
	//		average_c.unroll();


}






void
Cylinder::completeCylinder()
{
	//		complete 3-leg
	axes_[0]=axes_[2].cross(axes_[1]);

	//		unrolled polygon including >Trafo World 2 Plane
	unrolled_.assignMembers(axes_[1], axes_[2], origin_);

	//		calculate radius


	std::vector<std::vector<Eigen::Vector3f> >temp_vec;
	Eigen::Vector2f pp;
	float temp_r=0;
	int counter=0;

	this->getTransformedContours(unrolled_.transform_from_world_to_plane,temp_vec);


	for (int i = 0; i < (int) temp_vec.size(); ++i) {
		for (int j = 0; j < (int) temp_vec[i].size(); ++j) {

			pp << temp_vec[i][j][1],temp_vec[i][j][2];
			temp_r += pp.norm();
			counter++;
		}
	}


	r_=temp_r /counter;
	this->unroll();


}

void
Cylinder::assignMembers()
{
	std::cout<< "not yet implemented";
}

}//namespace
