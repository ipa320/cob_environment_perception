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
      //            contours3D[j][k] = contours[j][k];



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

void Cylinder::getCyl2D()
{
  /*
   * Convert Contours in cylindrical shape to polygonial shape
   */


  for (size_t j = 0; j < contours.size(); j++) {


    for (size_t k = 0; k < contours[j].size(); k++) {

      //      Transform  Points in Cylinder Coordinate System
      Eigen::Vector3f point_trans =
          transform_from_world_to_plane * contours[j][k];

      float Tx, alpha;
      //        flatten polygon
      getTrafo2d(point_trans, Tx, alpha);
      // New coordinates( p_y = 0 because now on a plane)
      point_trans[0] = Tx;
      point_trans[2] = 0;
      //          std::cout<<"point _trans\n"<< point_trans<<std::endl;
      //        transform back in world system
      Eigen::Vector3f point_global =
          transform_from_world_to_plane.inverse()
          * point_trans;
      //          std::cout<<"point _trans\n"<< point_global<<std::endl;


      contours[j][k] = point_global;

    }



  }

}

void Cylinder::ContoursFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {

  /* Assign Contours to Cylinder using border points
   * Transformation to internal representation of cylinder (characteristic polygon )
   *
   * Input:	cloud ........... ConstPtr to PointCloud containing the boarder points - still in cylinder shape
   *
   *
   *    */

  std::vector<Eigen::Vector3f> pts;
  pts.resize(cloud->points.size());
  for(unsigned int j=0; j<cloud->points.size(); j++)
  {
    /*pts[j] = Eigen::Vector3f(p.polygons[i].points[j].x,
                                    p.polygons[i].points[j].y,
                                    p.polygons[i].points[j].z);*/
    pts[j](0) = cloud->points[j].x;
    pts[j](1) = cloud->points[j].y;
    pts[j](2) = cloud->points[j].z;
  }
  contours.push_back(pts);

  for (size_t j = 0; j < contours.size(); j++) {


    for (size_t k = 0; k < contours[j].size(); k++) {

      //      Transform  Points in Cylinder Coordinate System
      Eigen::Vector3f point_trans =
          transform_from_world_to_plane * contours[j][k];

      float Tx, alpha;
      //        flatten polygon
      getTrafo2d(point_trans, Tx, alpha);
      // New coordinates( p_y = 0 because now on a plane)
      point_trans[0] = Tx;
      point_trans[2] = 0;
      //          std::cout<<"point _trans\n"<< point_trans<<std::endl;
      //        transform back in world system
      Eigen::Vector3f point_global =
          transform_from_world_to_plane.inverse()
          * point_trans;
      //          std::cout<<"point _trans\n"<< point_global<<std::endl;


      contours[j][k] = point_global;
      std::cout<<"contour\n "<<contours[j][k]<<"\n";

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
  seg.setMethodType (pcl::SAC_MLESAC);
  // Set number of maximum iterations
  seg.setMaxIterations (10);
  // Set type of model
  seg.setModelType (pcl::SACMODEL_CIRCLE2D);
  // Set threshold of model
  seg.setDistanceThreshold (0.010);
  // Give as input the filtered point cloud
  seg.setInputCloud (trans_cloud);
  // Call the segmenting method
  //	seg.getOptimizeCoefficients();
  seg.segment(inliers,coeff);




  //	origin in lcs
  Eigen::Vector3f l_origin,l_centroid;
  l_centroid = trafo_hor2w.inverse() * centroid3f;
  l_origin << coeff.values[0],coeff.values[1],l_centroid[2];
  //origin in wcs
  origin_ = trafo_hor2w * l_origin  ;

  //calculate axis from origin to centroid


  axes_[2]= centroid3f - origin_;


  if (axes_[2][2] > 0 ) {

    axes_[2] = origin_ - centroid3f;
  }





  r_=coeff.values[2];

  axes_[0].normalize();
  axes_[1].normalize();
  axes_[2].normalize();
  //	 std::cout<<"axes 1\n"<<axes_[1]<<"\n";
  //   std::cout<<"axes 2\n"<<axes_[2]<<"\n";


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

  if (axes_[1][2] < 0 )
  {
    axes_[1]= axes_[1] *-1;
  }

  axes_[0]=axes_[1].cross(axes_[2]);
  axes_[0].normalize();//= axes_[0];
  axes_[1].normalize();//= axes_[1];
  axes_[2].normalize();//= axes_[2].normalize();




  //	centroid
  centroid<< origin_[0]+(r_*axes_[2][0]) , origin_[1]+(r_*axes_[2][1]) , origin_[2]+(r_*axes_[2][2]) , 0;

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

    //    double diff_origin = connection.norm();
    //    connection.normalize();
    //    //		Eigen::Vector3f axes_norm = axes_[1].normalize();
    //
    //
    //    //		debug
    //    float dbg_1 = fabs(c_map.axes_[1] .dot(axes_[1]));
    //    float dbg_2 = c_map.r_ - r_;
    //    float dbg_3 = fabs(connection.dot(axes_[1]));


    //    std::cout<<"_________CYL "<<i<<"___________\n";
    //    std::cout<<"SYM-AXES = "<<dbg_1<<"\n";
    //    std::cout<<"RADIUS   = "<<dbg_2<<"\n";



    if ((fabs(c_map.axes_[1] .dot(axes_[1])) > limits.angle_thresh)  && fabs(c_map.r_ - r_) < (0.4 ) )

    {

      //			if (fabs(diff_origin)> 0.20 && fabs(connection.dot(axes_norm)) < 0.95 )
      //			{
      //				continue;
      //			}



      //      std::cout<<"CYL "<< i << "is Merge Candidate\n";

      Cylinder shifted_cylinder_map;

      c_map.getShiftedCylinder(*this,shifted_cylinder_map);


      bool is_intersected = this->isMergeCandidate_intersect(
          shifted_cylinder_map);

      if (is_intersected == false) {
        //        std::cout<<"BUT NO INTERSECTION...\n";
        //        std::cout<<"______________________\n";

        continue;
      }
      if (is_intersected == true) {
        //				std::cout<<"has intersection"<<std::endl;
        intersections.push_back(i);
        //        std::cout<<"______________________\n";


      }
      // std::cout << "intersection with map " << i << std::endl;
      //			}//elseif

    }//if

  }//for

}

void Cylinder::mergeCylinder(std::vector<CylinderPtr>& c_array) {


  //varibles for averaging new cylinder attributes
  Cylinder average_cyl;
  //	weightAttributes(c_array, average_cyl);

  std::vector<PolygonPtr> merge_polygons;

  //	transform unrolled_ to local system

  for (int i = 0; i < (int) c_array.size(); i++) {
    Cylinder & c_map = *c_array[i];

    std::string name="MAP";
    c_map.printAttributes(name);

    name="NEW";
    this->printAttributes(name);

    PolygonPtr map_shifted_polygon = PolygonPtr(new Polygon());

    //	get shifted polygons with respect to THIS


    c_map.getShiftedCylinder(*this,*map_shifted_polygon);


    map_shifted_polygon->merge_weight_=c_map.merge_weight_;
    //    map_shifted_polygon->centroid = c_map.centroid;
    //    map_shifted_polygon->normal = c_map.normal;
    map_shifted_polygon->merged = c_map.merged;
    //    map_shifted_polygon->d = c_map.d;

    merge_polygons.push_back(map_shifted_polygon);



  }//for


  //	polygon operation for merging
  //	Polygon & p_av=*merge_polygons_B[0];

  //	unrolled_.merge_union(merge_polygons_B,average_cyl.unrolled_);
  //merge_union(merge_polygons_B,*this);

  this->merge(merge_polygons);



  //	Polygon& merge_polygon_C = *merge_polygons_B[0];


  //	Cylinder& c_map2 = *c_array[0];





  //	assign values to resulting cylinder
  //	c_map2.contours = merge_polygon_C.contours;

  //		std::cout<<"DEBUG [3.1.2]\n";

  //		 weightAttributes(c_array, *this);


  this->applyWeightingCylinder(c_array);

  std::string name="cyl after merge";
  this->printAttributes(name);
  //	c_map2.r_ = average_cyl.r_; //-->needs to be from cylider specific weighting
  //	c_map2.axes_ =axes_;       // "    "
  //	c_map2.origin_ = average_cyl.origin_;
  //	c_map2.transform_from_world_to_plane
  //	= average_cyl.transform_from_world_to_plane;
  //
  //		std::cout<<"DEBUG [3.2]\n";
  //
  //	c_map2.axes_=average_cyl.axes_;
  //	c_map2.unroll();


  //	c_map2.debug_output("unrolled average");

  //	std::cout<<"DEBUG [4]\n";


  //	for (int i = 0; i < (int) c_array.size(); ++i) {
  //		if (i != 0) {
  //			c_array.erase(c_array.begin() + i);
  //
  //						std::cout<<"DEBUG [4]\n";
  //
  //		}

  //	}

}

void

Cylinder::getShiftedCylinder(Cylinder& c, Polygon & shifted_polygon) {

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
  //  std::cout<<"ALPHA = "<< alpha <<"\n";
  //  std::cout<<"X-shift = "<<x_shift<<"\n";

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
//  //calculate transformation from world to cylinder
//
//  assignMembers(axes_[1], axes_[2], origin_);
//
//  //	configure unroilled polygon
//  contours.resize(contours.size());
//  //		unrolled_.transform_from_world_to_plane=transformation_from_world_to_cylinder_;
//  for (size_t j = 0; j < contours.size(); j++) {
//
//    contours[j].resize(contours[j].size());
//    holes.resize(contours[j].size());
//
//    for (size_t k = 0; k < contours[j].size(); k++) {
//
//      //		  Transform  Points in Cylinder Coordinate System
//      Eigen::Vector3f point_trans =
//          transform_from_world_to_plane * contours[j][k];
//      c_map2.r_ = average_cyl.r_; //-->needs to be from cylider specific weighting
//      c_map2.axes_ =axes_;       // "    "
//      c_map2.origin_ = average_cyl.origin_;
//      c_map2.transform_from_world_to_plane
//      = average_cyl.transform_from_world_to_plane;
//
//      std::cout<<"DEBUG [3.2]\n";
//
//      c_map2.axes_=average_cyl.axes_;
//      float Tx, alpha;
//      //	      flatten polygon
//      getTrafo2d(point_trans, Tx, alpha);
//      // New coordinates( p_y = 0 because now on a plane)
//      point_trans[0] = Tx;
//      point_trans[2] = 0;
//      //	      	std::cout<<"point _trans\n"<< point_trans<<std::endl;
//      //	      transform back in world system
//      Eigen::Vector3f point_global =
//          transform_from_world_to_plane.inverse()
//          * point_trans;
//      //	      	std::cout<<"point _trans\n"<< point_global<<std::endl;
//
//
//      contours[j][k] = point_global;
//
//    }
//  }
//
//
//
//
//}

void
Cylinder::applyWeightingCylinder(std::vector<CylinderPtr>& merge_candidates)
{
  /*
   * Already weighted:
   *  - normal -- set to unaveraged value--> needs to be refreshed
   *  - centroid
   *  - d
   *  - merge weight
   *
   *
   *  Still need to be weighted:
   *  - r
   *  - origin
   *  - axes
   *  - transform from world to plane
   *  - normal
   */

  //  axes

  Eigen::Vector3f temp_axis1,temp_axis0, temp_axis2,temp_origin;

  temp_axis0 = this->merge_weight_ * this->axes_[0];
  temp_axis1 = this->merge_weight_ * this->axes_[1];
  temp_axis2 = this->merge_weight_ * this->axes_[2];
  temp_origin = this->merge_weight_ * this->origin_;
  float   merge_weight_sum = this ->merge_weight_;
  float temp_r = merge_weight_sum * this->r_;






  for (int i = 0; i < (int)merge_candidates.size(); ++i) {


    temp_axis0 += merge_candidates[i]->merge_weight_ * merge_candidates[i]->axes_[0] ;
    temp_axis1 += merge_candidates[i]->merge_weight_ * merge_candidates[i]->axes_[1] ;
    temp_axis2 += merge_candidates[i]->merge_weight_ * merge_candidates[i]->axes_[2] ;


    temp_origin += merge_candidates[i]->merge_weight_ * merge_candidates[i]->origin_;

    temp_r += merge_candidates[i]->merge_weight_ * merge_candidates[i]->r_;

    merge_weight_sum += merge_candidates[i]->merge_weight_;

  }

  this->r_ = temp_r / merge_weight_sum;
  this->axes_[1] = temp_axis1 / merge_weight_sum;

  this->axes_[0] = temp_axis0 / merge_weight_sum;
  this->axes_[2] = temp_axis2 / merge_weight_sum;

  this->axes_[0].normalize();
  this->axes_[1].normalize();
  this->axes_[2].normalize();

  this->origin_ = temp_origin / merge_weight_sum;
  this->normal = this->axes_[2];


  this->assignMembers(this->axes_[1],this->axes_[2],this->origin_);










}

 void
Cylinder::transform2tf(Eigen::Affine3f & trafo)
{
  //transform contours

  this->TransformContours(trafo);

  //  transform parameters
  Eigen::Vector3f tf_axes_1 = trafo.rotation() * this->axes_[1];
  this->axes_[1] =  tf_axes_1;

  Eigen::Vector3f tf_axes_2 = trafo.rotation() * this->axes_[2];
  this->axes_[2] = tf_axes_2;

  Eigen::Vector3f tf_origin = trafo * this->origin_;
  this->origin_ =  tf_origin;

  Eigen::Vector3f centroid3f;
  centroid3f<<  this->centroid[0], this->centroid[1], this->centroid[2];
  centroid3f = trafo * centroid3f;
  this->centroid << centroid3f[0], centroid3f[1], centroid3f[2], 0;
  this->computeAttributes(axes_[1],axes_[2],centroid);

}

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

void
Cylinder::computeAttributes(const Eigen::Vector3f& z_axis, const Eigen::Vector3f &new_normal, const Eigen::Vector4f& centroid)
{


  //  TODO: recomputation of centroid
  normal=new_normal;
  d=fabs(centroid.head(3).dot(normal));

  Eigen::Affine3f transform_from_plane_to_world;
  getTransformationFromPlaneToWorld(z_axis,normal,centroid.head(3),transform_from_plane_to_world);
  transform_from_world_to_plane=transform_from_plane_to_world.inverse();
}

void
Cylinder::printAttributes(std::string & name)
{

  std::cout<< "_______"<<name.c_str() <<"______\n";
  std::cout<<"origin = \n"<< this->origin_<<"\n";
  std::cout<< "centroid = \n"<<this->centroid<<"\n";
  std::cout<<"radius = "<< this->r_<<"\n";
  std::cout<<"x-axis = \n"<<this->axes_[0]<<"\n";
  std::cout<<"normal = "<< this->normal<<"\n";
  std::cout<<"sym_axis = "<< this->axes_[1]<<"\n";
  std::cout<<"merged = " << this->merged<<"\n";
  std::cout<<"merge_weight = "<<this->merge_weight_<<"\n";
  std::cout<< "_________________\n";


}

}//namespace
