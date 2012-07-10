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


void Cylinder::allocate()
{
	//allocate axes
	axes_.resize(3);
	axes_[0].resize(3);
	axes_[1].resize(3);
	axes_[2].resize(3);
}

void Cylinder::getCyl3D(std::vector<std::vector<Eigen::Vector3f> >& contours3D) {
	//	std::cout<<"getCyl3d"<<std::endl;
	//Transform to local coordinate system
	Polygon poly_plane;

	for (size_t j = 0; j < contours.size(); j++) {

		poly_plane.holes.resize(contours.size());
		poly_plane.contours.resize(contours.size());

		for (size_t k = 0; k < contours[j].size(); k++) {
			poly_plane.contours[j].resize(contours[j].size());
			//				std::cout<<"DEBUG: point before trafo\n"<<unrolled_.contours[j][k]<<std::endl;

			//		      Eigen::Vector3f point_trans = transformation_from_world_to_cylinder_*unrolled_.contours[j][k];
			Eigen::Vector3f point_trans =
					transform_from_world_to_plane
					* contours[j][k];

			poly_plane.contours[j][k] = point_trans;

		}
	}

	// transform into cylinder shape via polar coordinates
	for (size_t j = 0; j < poly_plane.contours.size(); j++) {

		contours3D.resize(poly_plane.contours.size());
		holes.resize(poly_plane.contours.size());

		for (size_t k = 0; k < poly_plane.contours[j].size(); k++) {

			contours3D[j].resize(poly_plane.contours[j].size());
			float alpha;
			Eigen::Vector3f point_temp;
			//		      alpha= B/r
			//			alpha = unrolled_.contours[j][k][0] / r_;
			alpha = poly_plane.contours[j][k][0] / r_;


			//				 use polar coordinates to create cylinder points
			point_temp << r_ * sin(alpha), poly_plane.contours[j][k][1], r_	* cos(alpha);


			//	      transform back in world system
			//			point_temp=transformation_from_world_to_cylinder_.inverse()* point_temp;
			point_temp = transform_from_world_to_plane.inverse()
																													* point_temp;

			//			std::cout<<"DEBUG: point after trafo\n"<<point_temp<<std::endl;

			contours3D[j][k] = point_temp;
			//		  		    std::cout<<std::endl;


		}
	}

}

void
Cylinder::ContoursFromList( std::vector<std::vector<Eigen::Vector3f> >& in_list)
{

	assignMembers(axes_[1], axes_[2], origin_);	//	configure unrolled polygon
	contours.resize(in_list.size());
	//		unrolled_.transform_from_world_to_plane=transformation_from_world_to_cylinder_;
	for (size_t j = 0; j < in_list.size(); j++) {

		contours[j].resize(in_list[j].size());
		holes.resize(in_list[j].size());

		for (size_t k = 0; k < in_list[j].size(); k++) {

			//		  Transform  Points in Cylinder Coordinate System
			Eigen::Vector3f point_trans =
					transform_from_world_to_plane * in_list[j][k];

			float Tx, alpha;
			//	      flatten polygon
			getTrafo2d(point_trans, Tx, alpha);
			// New coordinates( p_y = 0 because now on a plane)
			point_trans[0] = Tx;
			point_trans[2] = 0;
			//	      	std::cout<<"point _trans\n"<< point_trans<<std::endl;
			//	      transform back in world system
			Eigen::Vector3f point_global =
					transform_from_world_to_plane.inverse()
					* point_trans;
			//	      	std::cout<<"point _trans\n"<< point_global<<std::endl;


			contours[j][k] = point_global;
		}
	}
}



void Cylinder::ContoursFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud) {

	/* Assign Contours to Cylinder using border points
	 *
	 * Input:	in_cloud ........... ConstPtr to PointCloud containing the boarder points -> in right order?!?!
	 *
	 *
	 * TODO: Just single contours ->
	 */

	for (int i = 0; i < 1; ++i) {		//SINGLE CONTOUR

		contours.resize(1);

		for(int j = 0; j < (int)in_cloud->width; ++j) {
			contours[i].resize(in_cloud->width);

			contours[i][j]=in_cloud->at(i).getVector3fMap ();

		}
	}



}

void
Cylinder::ParamsFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud , std::vector<int>& indices)//, Eigen::Vector3f c_pt)//, Eigen::Vector3f& sym_axis)
{

	/* compute cylinder parameters from point cloud
	 *
	 * input: 	in_cloud ............ ConstPtr to PointCloud
	 * 			indices ............ vector, containing indices belonging to current cylinder
	 *
	 *
	 * params:	r .............. radius of cylinder
	 * 			axes ........... triad , defining cylinder coordinate system
	 * 			centroid ....... centroid of cylinder strip
	 * 			origin ......... point on symmetry axis
	 */


	//	Establish coordinate system for projection to horizontal plane

	//	get arbitrary axis, orthogonal to symmetry axis: local x-axis  (lx)
	Eigen::Vector3f lx;
	lx=axes_[1].unitOrthogonal();
	//	complete triad with local z axis (lz)
	Eigen::Vector3f lz;
	lz=lx.cross(axes_[1]);

	Eigen::Affine3f trafo_hor2w;
	Eigen::Vector3f centroid3f ;
	centroid3f <<centroid[0] , centroid[1] , centroid[2];
	this->getTransformationFromPlaneToWorld(axes_[1],centroid3f,trafo_hor2w);

	////	transform  points to horizontal coordinate system
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
	pcl::transformPointCloud(*in_cloud,indices,*trans_cloud,trafo_hor2w.inverse());




	//	use points in lcs to estimate r, origin

	// Inliers of circle model
	pcl::PointIndices inliers;
	// Coefficients of circle model
	pcl::ModelCoefficients coeff;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optimize coefficients
	seg.setOptimizeCoefficients (true);
	// Set type of method
	seg.setMethodType (pcl::SAC_LMEDS);
	// Set number of maximum iterations
	seg.setMaxIterations (10);
	// Set type of model
	seg.setModelType (pcl::SACMODEL_CIRCLE2D);
	// Set threshold of model
	seg.setDistanceThreshold (0.010);
	// Give as input the filtered point cloud
	seg.setInputCloud (trans_cloud);
	// Call the segmenting method
	seg.setOptimizeCoefficients(true);
	//	seg.getOptimizeCoefficients();
	seg.segment(inliers,coeff);




	//	origin in lcs
	Eigen::Vector3f l_origin;
	l_origin << coeff.values[0],coeff.values[1],0;
	//origin in wcs
	origin_ = trafo_hor2w * l_origin  ;
	//calculate axis from origin to centroid


	axes_[2]= centroid3f - origin_;

	if (axes_[2][2] > 0 ) {

		axes_[2] = origin_ - centroid3f;
	}




	axes_[0] = axes_[1].cross(axes_[2]);

	r_=coeff.values[2];

	//		unrolled polygon including >Trafo World 2 Plane
	this->assignMembers(axes_[1], axes_[2], origin_);


}


void
Cylinder::ParamsFromShapeMsg(){

	/*
	 * Function ParamsFromROSMsg is used to complete the cylinder parameters,
	 * with the given set of parameters obtained from a ROS ShapeMsg.
	 * (cob_environment_perception/cob_3d_mapping_msgs/msg/Shape.msg)
	 *
	 * abbreviations:	wcs ............................. World coordinate System
	 * 					lcs ............................. local coordinate system
	 *
	 *
	 * Already set:		r_ ............................... Radius of the cylinder
	 * 					origin_ .......................... Origin, point on symmetry axis, in lcs , same y coordinate as centroid
	 * 					axes_[1] ......................... Symmetry axis of c ylinder, y- axis in lcs
	 * 					axes_[2] ......................... axis, connecting origin and centroid, Z-axis
	 *
	 *
	 * To be set:		centroid ........................ centroid of cylinder strip
	 * 					normal .......................... equivalent to axes_[2], serves as normal to unrolled cylinder plane
	 * 					axes_[0] ........................ completes the triade of axes
	 * 					transform_from_world_to_plane ...
	 * 					d ............................... distance to origin of wcs
	 *
	 */

	//	axes_[0]
	axes_[0]=axes_[1].cross(axes_[2]);
	axes_[0].normalize();//= axes_[0];
	axes_[1].normalize();//= axes_[1];
	axes_[2].normalize();//= axes_[2].normalize();


	//	centroid
	//	centroid = origin_+(r_*axes_[2]);

	//		normal, d and transform_from_world_to_plane set within Polygon::assignMembers
	this->assignMembers(axes_[1], axes_[2], origin_);



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
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//methods for merging~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Cylinder::isMergeCandidate(const std::vector<CylinderPtr>& cylinder_array,
		const merge_config& limits, std::vector<int>& intersections) {


	for (size_t i = 0; i < cylinder_array.size(); i++) {

		Cylinder& c_map = *(cylinder_array[i]);


		//compare symmetry axes and d
//		std::cout<<"r1 "<<c_map.r_<<"\n";
//		std::cout<<"r2 "<<r_<<"\n\n";

		Eigen::Vector3f connection=c_map.origin_-origin_;

		double diff_origin = connection.norm();
		connection.normalize();


		if ((fabs(c_map.axes_[1] .dot(axes_[1])) > limits.angle_thresh)  && fabs(c_map.r_ - r_) < (0.2 ) )


		{

			if (fabs(diff_origin)> 0.20 && fabs(connection.dot(axes_[1])) < 0.95 )
			{
				continue;
			}
//			}
//			else if (fabs(diff_origin)> 0.1 && fabs(connection.dot(axes_[1])) > 0.97 )
//			{
//
//			}

			Polygon shifted_polygon_map;

			c_map.getShiftedPolygon(*this,shifted_polygon_map);


			bool is_intersected = this->isMergeCandidate_intersect(
					shifted_polygon_map);

			if (is_intersected == false) {

				continue;
			}
			if (is_intersected == true) {
				//				std::cout<<"has intersection"<<std::endl;
				intersections.push_back(i);

			}
			// std::cout << "intersection with map " << i << std::endl;
//			}//elseif

		}//if

	}//for

}

void Cylinder::merge(std::vector<CylinderPtr>& c_array) {

	//varibles for averaging new cylinder attributes
	Cylinder average_cyl;
	weightAttributes(c_array, average_cyl);

	std::vector<PolygonPtr> merge_polygons_B;

	//	transform unrolled_ to local system

	//	std::cout<<"DEBUG [1]\n";
	for (int i = 0; i < (int) c_array.size(); i++) {
		Cylinder & c_map = *(c_array[i]);

		PolygonPtr map_shifted_polygon = PolygonPtr(new Polygon());

		//	get shifted polygons with respect to THIS


		c_map.getShiftedPolygon(*this,*map_shifted_polygon);



		merge_polygons_B.push_back(map_shifted_polygon);

		//		std::cout<<"DEBUG [2]\n";

	}//for


	//	polygon operation for merging
	//	Polygon & p_av=*merge_polygons_B[0];

	//	unrolled_.merge_union(merge_polygons_B,average_cyl.unrolled_);
	merge_union(merge_polygons_B,*this);

	//	std::cout<<"DEBUG [3]\n";


	Polygon& merge_polygon_C = *merge_polygons_B[0];
	//	std::cout<<"DEBUG [3.1\n";


	Cylinder& c_map2 = *c_array[0];
	//	std::cout<<"DEBUG [3.1.1]\n";

	//	assign values to resulting cylinder
	c_map2.contours = merge_polygon_C.contours;
	//	std::cout<<"DEBUG [3.1.2]\n";

	c_map2.r_ = average_cyl.r_;
	c_map2.axes_ =axes_;
	c_map2.origin_ = average_cyl.origin_;
	c_map2.transform_from_world_to_plane
	= transform_from_world_to_plane;
	std::vector<std::vector<Eigen::Vector3f> > contours3d;
	//	std::cout<<"DEBUG [3.2]\n";

	c_map2.getCyl3D(contours3d);
	c_map2.axes_=average_cyl.axes_;
	//	c_map2.unroll();


	c_map2.debug_output("unrolled average");

	//	std::cout<<"DEBUG [4]\n";


	for (int i = 0; i < (int) c_array.size(); ++i) {

		if (i != 0) {
			c_array.erase(c_array.begin() + i);
			//			std::cout<<"DEBUG [4]\n";

		}

	}

}

void

Cylinder::getShiftedPolygon(Cylinder& c, Polygon & shifted_polygon) {

	//	    	Transform normal of map polygon in cylinder system of THIS

	//	LOCAL

	Eigen::Vector3f transformed_normal =
			c.transform_from_world_to_plane.rotation()
			* normal;

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


			shifted_polygon.contours[j][k]		= c.transform_from_world_to_plane.inverse()
																															* (shift_trafo
																																	* c.transform_from_world_to_plane
																																	* contours[j][k]);
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

	average_c.assignMembers(average_c.axes_[1], average_c.axes_[2],
			average_c.origin_);

	//calculate transformation from world to cylinder
	//		average_c.unroll();


}



//void
//Cylinder::completeCylinder()
//{
//
//	//		complete triade
//	axes_[0]=axes_[2].cross(axes_[1]);
//
//	//		unrolled polygon including >Trafo World 2 Plane
//	this->assignMembers(axes_[1], axes_[2], origin_);
//
//	//		calculate radius
//
//
//	std::vector<std::vector<Eigen::Vector3f> >temp_vec;
//	Eigen::Vector2f pp;
//	float temp_r=0;
//	int counter=0;
//
//	this->getTransformedContours(transform_from_world_to_plane,temp_vec);
//
//
//	for (int i = 0; i < (int) temp_vec.size(); ++i) {
//		for (int j = 0; j < (int) temp_vec[i].size(); ++j) {
//
//			pp << temp_vec[i][j][1],temp_vec[i][j][2];
//			temp_r += pp.norm();
//			counter++;
//		}
//	}
//
//
//	r_=temp_r /counter;
//	//	this->unroll();
//
//
//}
//void Cylinder::unroll() {
//
//	//calculate transformation from world to cylinder
//
//	assignMembers(axes_[1], axes_[2], origin_);
//
//	//	configure unroilled polygon
//	contours.resize(contours.size());
//	//		unrolled_.transform_from_world_to_plane=transformation_from_world_to_cylinder_;
//	for (size_t j = 0; j < contours.size(); j++) {
//
//		contours[j].resize(contours[j].size());
//		holes.resize(contours[j].size());
//
//		for (size_t k = 0; k < contours[j].size(); k++) {
//
//			//		  Transform  Points in Cylinder Coordinate System
//			Eigen::Vector3f point_trans =
//					transform_from_world_to_plane * contours[j][k];
//
//			float Tx, alpha;
//			//	      flatten polygon
//			getTrafo2d(point_trans, Tx, alpha);
//			// New coordinates( p_y = 0 because now on a plane)
//			point_trans[0] = Tx;
//			point_trans[2] = 0;
//			//	      	std::cout<<"point _trans\n"<< point_trans<<std::endl;
//			//	      transform back in world system
//			Eigen::Vector3f point_global =
//					transform_from_world_to_plane.inverse()
//					* point_trans;
//			//	      	std::cout<<"point _trans\n"<< point_global<<std::endl;
//
//
//			contours[j][k] = point_global;
//
//		}
//	}
//
//
//
//
//}


void
Cylinder::dbg_out(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,std::string & name){


	std::ofstream os;
	std::string path = "/home/goa-tz/debug/";
	path.append(name.c_str());
	os.open(path.c_str());

	for (int i = 0; i < (int)points->width; ++i) {
		os << points->points[i].x;
		os <<" ";

		os << points->points[i].y;
		os <<" ";

		os << points->points[i].z;


		os <<"\n";
	}




	os.close();





}

}//namespace
