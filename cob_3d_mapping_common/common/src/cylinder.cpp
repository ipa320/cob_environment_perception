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

//##############Methods to initialize cylinder and its paramers#########

void
Cylinder::ContoursFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{

  /* Assign Contours to Cylinder using border points
   * Transformation to internal representation of cylinder (characteristic polygon )
   *
   * Input: cloud ........... ConstPtr to PointCloud containing the boarder points - still in cylinder shape
   *
   *
   *    */

  std::vector<Eigen::Vector3f> pts;
  pts.resize(cloud->points.size());
  for(unsigned int j=0; j<cloud->points.size(); j++)
  {

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
Cylinder::ContoursFromList( std::vector<std::vector<Eigen::Vector3f> >& in_list)
{

  computeAttributes(sym_axis, normal, origin_); //  configure unrolled polygon
  contours.resize(in_list.size());
  //    unrolled_.transform_from_world_to_plane=transformation_from_world_to_cylinder_;
  for (size_t j = 0; j < in_list.size(); j++) {

    contours[j].resize(in_list[j].size());
    holes.resize(in_list[j].size());

    for (size_t k = 0; k < in_list[j].size(); k++) {

      //      Transform  Points in Cylinder Coordinate System
      Eigen::Vector3f point_trans =
          transform_from_world_to_plane * in_list[j][k];

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

void
Cylinder::ParamsFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud , std::vector<int>& indices)//, Eigen::Vector3f c_pt)//, Eigen::Vector3f& sym_axis)
{

  /* compute cylinder parameters from point cloud
   *
   * input:   in_cloud ............ ConstPtr to PointCloud
   *      indices ............ vector, containing indices belonging to current cylinder
   *
   *
   * params:  r .............. radius of cylinder
   *      axes ........... triad , defining cylinder coordinate system
   *      centroid ....... centroid of cylinder strip
   *      origin ......... point on symmetry axis
   */


  //  Establish coordinate system for projection to horizontal plane

  Eigen::Affine3f trafo_hor2w;
  Eigen::Vector3f centroid3f ;
  centroid3f <<centroid[0] , centroid[1] , centroid[2];
  // note: this transformation is only for coefficient estimation purposes and is not used for the final cylinder
  this->getTransformationFromPlaneToWorld(sym_axis,centroid3f,trafo_hor2w);

  ////  transform  points to horizontal coordinate system
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
  pcl::transformPointCloud(*in_cloud,indices,*trans_cloud,trafo_hor2w.inverse());



  //  use points in lcs to estimate r, origin

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
  //  seg.getOptimizeCoefficients();
  seg.segment(inliers,coeff);


  //  origin in lcs
  Eigen::Vector3f l_origin,l_centroid;
  l_centroid = trafo_hor2w.inverse() * centroid3f;
  l_origin << coeff.values[0],coeff.values[1],l_centroid[2];
  //origin in wcs
  origin_ = trafo_hor2w * l_origin  ;

  //calculate axis from origin to centroid


  normal= centroid3f - origin_;





  //  if (normal[2] > 0 ) {
  //
  //    normal = origin_ - centroid3f;
  //  }



  r_=coeff.values[2];

  sym_axis.normalize();
  normal.normalize();


  //    unrolled polygon including >Trafo World 2 Plane
  this->computeAttributes(sym_axis, normal, origin_);


  //  //  compute height of cylinder
  //  Eigen::Vector4f min_pts,max_pts;
  //  pcl::transformPointCloud(*in_cloud,indices,*trans_cloud,this->transform_from_world_to_plane);
  //  pcl::getMinMax3D(*trans_cloud,min_pts,max_pts);



}

void
Cylinder::ParamsFromShapeMsg(){

  /*
   * Function ParamsFromROSMsg is used to complete the cylinder parameters,
   * with the given set of parameters obtained from a ROS ShapeMsg.
   * (cob_environment_perception/cob_3d_mapping_msgs/msg/Shape.msg)
   *
   * abbreviations: wcs ............................. World coordinate System
   *          lcs ............................. local coordinate system
   *
   *
   * Already set:   r_ ............................... Radius of the cylinder
   *          origin_ .......................... Origin, point on symmetry axis, in lcs , same y coordinate as centroid
   *          sym_axis ......................... Symmetry axis of c ylinder, y- axis in lcs
   *          centroid ......................... centroid of cylinder piece
   *
   *
   * To be set:   centroid ........................ centroid of cylinder strip
   *          normal .......................... equivalent to axes_[2], serves as normal to unrolled cylinder plane
   *          transform_from_world_to_plane ...
   *          d ............................... distance to origin of wcs
   *
   */




  if (sym_axis[2] < 0 )
  {
    sym_axis= sym_axis *-1;
  }


  sym_axis.normalize();//= sym_axis;
  normal.normalize();//= normal.normalize();




  //  centroid
  //  centroid<< origin_[0]+(r_*normal[0]) , origin_[1]+(r_*normal[1]) , origin_[2]+(r_*normal[2]) , 0;

  //    normal, d and transform_from_world_to_plane set within Polygon::assignMembers
  this->computeAttributes(sym_axis, normal, origin_);






}

void
Cylinder::computeAttributes(const Eigen::Vector3f& sym_axis, const Eigen::Vector3f &new_normal, const Eigen::Vector3f& new_origin)
{


  //  TODO: recomputation of centroid
  origin_=new_origin;
  d=fabs(this->centroid.head(3).dot(normal));
  //  normal = sym_axis.unitOrthogonal();
  //  pcl::getTransformationFromTwoUnitVectorsAndOrigin(
  //          this->sym_axis,this->normal,this->origin_,this->transform_from_world_to_plane);


  normal=new_normal;
  Eigen::Affine3f transform_from_plane_to_world;
  getTransformationFromPlaneToWorld(sym_axis,normal,origin_,transform_from_plane_to_world);
  transform_from_world_to_plane=transform_from_plane_to_world.inverse();


  //  calculation of h_min and h_max
  double min,max;
  min = 1000;
  max = -1000;
  std::vector<std::vector<Eigen::Vector3f> > trans_contours;
  this->getTransformedContours(this->transform_from_world_to_plane);
  for (size_t i = 0; i < contours.size(); ++i) {
    for (size_t j = 0; j < contours[i].size(); ++j) {
      if (contours[i][j][1]< min) min = contours[i][j][1];
      if (contours[i][j][1]> min) max = contours[i][j][1];
    }
  }

  h_max_ = max;
  h_min_ = min;

  //  std::cout<<"h = "<<h_max_ - h_min_<<"\n";
}


void
Cylinder::transform2tf(Eigen::Affine3f & trafo)
{
  //transform contours

  this->TransformContours(trafo);

  //  transform parameters
  Eigen::Vector3f tf_axes_1 = trafo.rotation() * this->sym_axis;
  this->sym_axis =  tf_axes_1;

  Eigen::Vector3f tf_axes_2 = trafo.rotation() * this->normal;
  this->normal = tf_axes_2;

  Eigen::Vector3f tf_origin = trafo * this->origin_;
  this->origin_ =  tf_origin;

  Eigen::Vector3f centroid3f;
  centroid3f<<  this->centroid[0], this->centroid[1], this->centroid[2];
  centroid3f = trafo * centroid3f;
  this->centroid << centroid3f[0], centroid3f[1], centroid3f[2], 0;
  this->computeAttributes(sym_axis,normal,origin_);

}

void
Cylinder::GrabParams(Cylinder& c_src)
{
  this->centroid = c_src.centroid;
  this->origin_ = c_src.origin_;
  this->d     = c_src.d;
  this->r_ = c_src.r_;
  this->transform_from_world_to_plane = c_src.transform_from_world_to_plane;
  this->normal = c_src.normal;
  this->sym_axis = c_src.sym_axis;
  this->h_max_ =c_src.h_max_;
  this->h_min_ =c_src.h_min_;
  this->merge_weight_ = c_src.merge_weight_;
  this->merged = c_src.merged;
  this->frame_stamp = c_src.frame_stamp;
  this->holes = c_src.holes;

}



//################## methods to roll and unroll cylinder###############

void Cylinder::getCyl3D(std::vector<std::vector<Eigen::Vector3f> >& contours3D) {
  //	std::cout<<"getCyl3d"<<std::endl;
  //Transform to local coordinate system
  Polygon poly_plane;

  for (size_t j = 0; j < contours.size(); j++) {

    poly_plane.holes.resize(contours.size());
    poly_plane.contours.resize(contours.size());

    for (size_t k = 0; k < contours[j].size(); k++) {
      poly_plane.contours[j].resize(contours[j].size());

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
      alpha = poly_plane.contours[j][k][0] / r_;


      //				 use polar coordinates to create cylinder points
      point_temp << r_ * sin(alpha), poly_plane.contours[j][k][1], r_	* cos(alpha);
      //      point_temp = poly_plane.contours[j][k];


      //	      transform back in world system
      point_temp = transform_from_world_to_plane.inverse()
																													                                                                                                                        * point_temp;



      contours3D[j][k] = point_temp;
      //      contours3D[j][k] = contours[j][k];




    }
  }

}
void Cylinder::makeCyl3D() {
  //  std::cout<<"getCyl3d"<<std::endl;
  //Transform to local coordinate system
  Polygon poly_plane;

  for (size_t j = 0; j < contours.size(); j++) {

    poly_plane.holes.resize(contours.size());
    poly_plane.contours.resize(contours.size());

    for (size_t k = 0; k < contours[j].size(); k++) {
      poly_plane.contours[j].resize(contours[j].size());

      Eigen::Vector3f point_trans =
          transform_from_world_to_plane
          * contours[j][k];

      poly_plane.contours[j][k] = point_trans;


    }
  }

  // transform into cylinder shape via polar coordinates
  for (size_t j = 0; j < poly_plane.contours.size(); j++) {

    holes.resize(poly_plane.contours.size());

    for (size_t k = 0; k < poly_plane.contours[j].size(); k++) {

      float alpha;
      Eigen::Vector3f point_temp;
      //          alpha= B/r
      alpha = poly_plane.contours[j][k][0] / r_;


      //         use polar coordinates to create cylinder points
      point_temp << r_ * sin(alpha), poly_plane.contours[j][k][1], r_ * cos(alpha);
      //      point_temp = poly_plane.contours[j][k];


      //        transform back in world system
      point_temp = transform_from_world_to_plane.inverse()
                                                                                                                                                                                  * point_temp;



      contours[j][k] = point_temp;
      //      contours3D[j][k] = contours[j][k];




    }
  }

}



void Cylinder::makeCyl2D()
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
void Cylinder::getCyl2D(Cylinder& c2d)
{
  /*
   * Convert Contours in cylindrical shape to polygonial shape
   */
  c2d.GrabParams(*this);
  c2d.contours.resize(contours.size());

  for (size_t j = 0; j < contours.size(); j++) {
    c2d.contours[j].resize(contours[j].size());

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


      c2d.contours[j][k] = point_global;

    }

  }

}



//################## methods for merging############################


void Cylinder::isMergeCandidate(const std::vector<CylinderPtr>& cylinder_array,
    const merge_config& limits, std::vector<int>& intersections) {


  for (size_t i = 0; i < cylinder_array.size(); i++) {

    Cylinder& c_map = *(cylinder_array[i]);


    Eigen::Vector3f connection=c_map.origin_-origin_;


    if ((fabs(c_map.sym_axis .dot(sym_axis)) > limits.angle_thresh)  && fabs(c_map.r_ - r_) < (0.01 ) )

    {


      Cylinder c_map_work,c_this_work;


      //      c_map.getShiftedCylinder(*this,*this,shifted_cylinder,false);
      c_map.t2t(*this,c_map_work);


      c_map_work.makeCyl2D();
      this->getCyl2D(c_this_work);


      std::string a,b;
      a="1";
      b="2";



      bool is_intersected = c_this_work.isMergeCandidate_intersect(
          c_map_work);

      if (is_intersected == false) {
        continue;
      }
      if (is_intersected == true) {
        intersections.push_back(i);
      }

    }//if

  }//for

}

//void Cylinder::merge(std::vector<CylinderPtr>& c_array) {
//
//  std::vector<CylinderPtr> merge_cylinders;
//
//  //create average cylinder for  averaging
//  CylinderPtr average_cyl =CylinderPtr(new Cylinder());
//  *average_cyl = *this;
//  average_cyl->applyWeighting(c_array);
//
//  //	transform  to local system
//  for (int i = 0; i < (int) c_array.size(); i++) {
//    Cylinder & c_map = *c_array[i];
//
//    //shifted cylinder is computed with respect to "this"- system
//
//    CylinderPtr shifted_cylinder = CylinderPtr(new Cylinder());
//
//    c_map.getShiftedCylinder(*average_cyl,*shifted_cylinder);
//
//    merge_cylinders.push_back(shifted_cylinder);
//  }
//
//  //  cast CylinderPtr to PolygonPtr to use merge_union   -- is  a better way possible ?!
//  std::vector<PolygonPtr> merge_polygons;
//  for (size_t i = 0; i < merge_cylinders.size(); ++i) {
//    PolygonPtr tmp_ptr= merge_cylinders[i];
//    merge_polygons.push_back(tmp_ptr);
//  }
//  PolygonPtr average_polygon= average_cyl;
//
//  this->merge_union(merge_polygons,average_polygon);
//
//  this->r_ = average_cyl->r_;
//  this->computeCentroid();
//  this->computeAttributes(average_cyl->sym_axis,average_cyl->normal,average_cyl->origin_);
//  this->assignWeight();
//
//  //  std::cout<<" weight after merge: "<<this->merge_weight_<<std::endl;
//
//
//}
void Cylinder::merge(std::vector<CylinderPtr>& c_array) {
  std::vector<CylinderPtr> merge_cylinders;

  //create average cylinder for  averaging
  CylinderPtr average_cyl =CylinderPtr(new Cylinder());
  *average_cyl = *this;
  average_cyl->applyWeighting(c_array);

  //  transform  to local system



  //  this->getShiftedCylinder(*c_array[0],*average_cyl,*shifted_cylinder,true);
  this->t2t(*average_cyl,*this);
  this->makeCyl2D();



  for (int i = 0; i < (int) c_array.size(); i++) {

    std::cout<<"//////MERGE SIZE = "<<c_array.size()<<"\n";
    Cylinder & c_map = *c_array[i];

    //shifted cylinder is computed with respect to "this"- system

    //    c_map.getShiftedCylinder(*c_array[0],*average_cyl,*shifted_cylinder,true);
    c_array[i]->t2t(*average_cyl,*c_array[i]);
    c_array[i]->makeCyl2D();


    merge_cylinders.push_back(c_array[i]);
  }

  //  cast CylinderPtr to PolygonPtr to use merge_union   -- is  a better way possible ?!
  std::vector<PolygonPtr> merge_polygons;
  for (size_t i = 0; i < merge_cylinders.size(); ++i) {
    PolygonPtr tmp_ptr= merge_cylinders[i];
    merge_polygons.push_back(tmp_ptr);
  }
  PolygonPtr average_polygon= average_cyl;


  this->merge_union(merge_polygons,average_polygon);
  this->GrabParams(*average_cyl);
  this->assignWeight();
  this->makeCyl3D();

  //  std::cout<<" weight after merge: "<<this->merge_weight_<<std::endl;


}

void
Cylinder::applyWeighting(std::vector<CylinderPtr>& merge_candidates)
{
  /*
   * Already weighted:
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


  Eigen::Vector3f temp_sym_axis,temp_origin;

  temp_sym_axis = this->merge_weight_ * this->sym_axis;
  temp_origin = this->merge_weight_ * this->origin_;

  double   merge_weight_sum = this ->merge_weight_;
  double temp_r = merge_weight_sum * this->r_;
  int merged_sum = this->merged;




  for (int i = 0; i < (int)merge_candidates.size(); ++i) {


    temp_sym_axis += merge_candidates[i]->merge_weight_ * merge_candidates[i]->sym_axis ;
    temp_origin += merge_candidates[i]->merge_weight_ * merge_candidates[i]->origin_;

    temp_r += merge_candidates[i]->merge_weight_ * merge_candidates[i]->r_;
    merge_weight_sum += merge_candidates[i]->merge_weight_;
    merged_sum  += merge_candidates[i]->merged;

    if (merge_candidates[i]->h_max_ > this-> h_max_)this->h_max_ = merge_candidates[i]->h_max_;
    if (merge_candidates[i]->h_max_  < this-> h_min_)this->h_min_ = merge_candidates[i]->h_max_;


  }

  this->sym_axis = temp_sym_axis / merge_weight_sum;
  this->origin_ = temp_origin / merge_weight_sum;
  this->sym_axis.normalize();

  this->r_ = temp_r / merge_weight_sum;


  if (merged_sum < 9)
  {
    this->merged=merged_sum;
  }
  else
  {
    this->merged=9;
  }



  Eigen::Vector3f x_axis;
  x_axis << this->sym_axis[1], - this->sym_axis[0], this->sym_axis[2];
  this->normal = this->sym_axis.cross(x_axis);
  std::cout<<"av normal"<<this->normal<<"\n";

  //  !overide! use initial values for normal and sym_axis
  this->normal = merge_candidates[0]->normal;
  //  this->sym_axis = merge_candidates[0]-> sym_axis;
  //  this->origin_ = merge_candidates[0]-> origin_;



  Eigen::Affine3f transform_from_plane_to_world;
  getTransformationFromPlaneToWorld(this->sym_axis,this->normal,this->origin_,transform_from_plane_to_world);
  this->transform_from_world_to_plane=transform_from_plane_to_world.inverse();





  //  std::cout<<"merged cyl sym axis \n"<<this->sym_axis<<"\n";
  //  std::cout<<"merged normal\n"<<this->normal<<"\n";


  //  this->computeAttributes(this->sym_axis,this->normal,this->origin_);

  //  this->normal = this->normal;
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
Cylinder::printAttributes(std::string & name)
{

  std::cout<< "_______"<<name.c_str() <<"______\n";
  std::cout<<"origin = \n"<< this->origin_<<"\n";
  std::cout<< "centroid = \n"<<this->centroid<<"\n";
  std::cout<<"radius = "<< this->r_<<"\n";
  std::cout<<"normal = "<< this->normal<<"\n";
  std::cout<<"sym_axis = "<< this->sym_axis<<"\n";
  std::cout<<"merged = " << this->merged<<"\n";
  std::cout<<"merge_weight = "<<this->merge_weight_<<"\n";
  std::cout<< "_________________\n";


}

void Cylinder::getTrafo2d(const Eigen::Vector3f& vec3d, float& Tx, float& alpha) {

  //  calculation of translation Tx and Ty
  //
  //  Tx by meansof arc length
  //  Tx corresponds to radius of cylinder
  float cos_alpha;
  Eigen::Vector2f vec2d, z_axis;

  if (debug_ == true) {
    //  Debug Output
    std::cout << "Point cylinder:" << std::endl << vec3d << std::endl;
  }

  //x and z components of point in local system
  vec2d << vec3d[0], vec3d[2];
  //y-axis in local system
  z_axis << 0, 1;

  //  angle between y-axis and point-origin connection
  cos_alpha = vec2d.dot(z_axis) / (vec2d.norm() * z_axis.norm());
  alpha = (acos(cos_alpha));
  //  std::cout<<"alpha = "<<alpha<<"\n";
  //  calculation of arc length
  if (vec2d[0] < 0) {
    Tx = -(r_ * alpha);
  } else {
    Tx = r_ * alpha;

  }

  if (debug_ == true) {
    //  Debug Output
    std::cout << "avec" << std::endl << vec2d << std::endl;
    std::cout << "TX = " << Tx << std::endl << std::endl;

  }

}
//void
//Cylinder::getShiftedCylinder(Cylinder& c, Cylinder & shifted_cylinder) {
//
//  //        Transform normal of map polygon in cylinder system of THIS
//
//  //  LOCAL
//
//  Eigen::Vector3f transformed_normal =
//      c.transform_from_world_to_plane.rotation()
//      * normal;
//
//  //          calculate trafo parameters
//  float x_shift, z_shift, alpha;
//  Eigen::Vector3f temp_vec;
//
//  temp_vec = (c.origin_ - origin_);
//  z_shift = temp_vec.norm();
//
//  getTrafo2d(transformed_normal, x_shift, alpha);
//  //  std::cout<<"ALPHA = "<< alpha <<"\n";
//  //  std::cout<<"X-shift = "<<x_shift<<"\n";
//
//  Eigen::Affine3f shift_trafo;
//  pcl::getTransformation(x_shift, 0, z_shift, 0, alpha, 0, shift_trafo);
//
//
//  shifted_cylinder.contours.resize(contours.size());
//  shifted_cylinder.holes.resize(contours.size());
//
//  for (size_t j = 0; j < contours.size(); j++) {
//    shifted_cylinder.contours[j].resize(contours[j].size());
//
//    for (size_t k = 0; k < contours[j].size(); k++) {
//      //      Transform  Points in Cylinder Coordinate System
//
//
//      shifted_cylinder.contours[j][k]   = c.transform_from_world_to_plane.inverse()
//                                                                                                                                                              * (shift_trafo
//                                                                                                                                                                  * c.transform_from_world_to_plane
//                                                                                                                                                                  * contours[j][k]);
//      //END LOCAL
//
//    }
//  }
//
//  shifted_cylinder.merge_weight_=c.merge_weight_;
//  shifted_cylinder.merged = c.merged;
//  shifted_cylinder.computeAttributes(c.sym_axis, c.normal, origin_);
//
//}
void
Cylinder::getShiftedCylinder(Cylinder& c2,Cylinder& c3, Cylinder & result,bool dbg) {

  /*
   * Three coordinate systems involved:
   * - system of this ---- S1
   * - reference system system, the shift is supposed to be calculated to ---- S2
   * - target system the result is supposed to be described in  ---- S3
   *
   *
   * Step 1  - calculate T12 - the trafo from S1 to S2
   * Step 2  - calculate T12x - T12x with additional shift
   * Step 3  - calculate T23 - trafo from S2 to S3
   * Step 4  - accumulate trafos - apply to c1
   *
   */




  //Step 1______________________________

  Eigen::Vector3f n12 =
      c2.transform_from_world_to_plane.rotation()
      * this->normal;



  Eigen::Vector3f s12 =
      c2.transform_from_world_to_plane.rotation()
      * this->sym_axis;



  Eigen::Vector3f o12 =
      c2.transform_from_world_to_plane
      *this->origin_;


  Eigen::Affine3f T12;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(s12,n12,o12,T12);
  //  Debug
  float roll,pitch,yaw;
  pcl::getEulerAngles(T12,roll,pitch,yaw);

  float alpha;
  alpha = -pitch;
  if (n12[0]>0 && n12[2] <0 ) {
    alpha = M_PI -  alpha ;
  }
  //  else if (transformed_normal[0]<0 && transformed_normal[2] <0 ) {
  //    pitch = -M_PI-pitch ;
  //
  //  }


  //Step2___________________________________
  Eigen::Vector3f  shift;
  shift = pcl::getTranslation(T12);
  shift[0] +=  c3.r_ * alpha;
  shift[1] = 0;
  if (dbg == true) {
    std::cout<<"_____________________\n";
    std::cout<<"roll= "<<roll<<" pitch= "<<pitch<<" yaw= "<<yaw<<std::endl;

    std::cout<<"norma = \n"<<n12<<"\n";
    std::cout<<"angles = \n";
    std::cout<<"alpha = "<<pitch  <<"\n";
    std::cout<<"tx = "<<shift[0]<<std::endl;
    std::cout<<"_____________________\n";
  }

  Eigen::Affine3f T12x ;
  pcl::getTransformation(shift[0],shift[1],shift[2],roll,pitch,yaw,T12x);


  //Step3________________________________
  Eigen::Vector3f n23 =
      c3.transform_from_world_to_plane.rotation()
      * c2.normal;



  Eigen::Vector3f s23 =
      c3.transform_from_world_to_plane.rotation()
      * c2.sym_axis;



  Eigen::Vector3f o23 =
      c3.transform_from_world_to_plane
      *c2.origin_;


  Eigen::Affine3f T23;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(s23,n23,o23,T23);


  //Step4__________________________________





  result.contours.resize(contours.size());
  result.holes.resize(contours.size());

  for (size_t j = 0; j < contours.size(); j++) {
    result.contours[j].resize(contours[j].size());

    for (size_t k = 0; k < contours[j].size(); k++) {
      //      Transform  Points in Cylinder Coordinate System


      result.contours[j][k]   =
          c3.transform_from_world_to_plane.inverse()* (T23*T12x* c3.transform_from_world_to_plane* contours[j][k]);
      //END LOCAL

    }
  }

  result.merge_weight_=c3.merge_weight_;
  result.merged = c3.merged;
  result.computeAttributes(c3.sym_axis, c3.normal, origin_);

}


void
Cylinder::t2t(Cylinder& c_target,Cylinder& c_result)
{
  //Transform "this" cylinder to c_target

  //  transform parameters

  c_result.GrabParams(c_target);

  //Step 1______________________________

  Eigen::Vector3f n12 =
      c_target.transform_from_world_to_plane.rotation()
      * this->normal;



  Eigen::Vector3f s12 =
      c_target.transform_from_world_to_plane.rotation()
      * this->sym_axis;



  Eigen::Vector3f o12 =
      c_target.transform_from_world_to_plane
      *this->origin_;


  Eigen::Affine3f T12;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(s12,n12,o12,T12);

  c_result.contours.resize(this->contours.size());
  for (size_t i = 0; i < this->contours.size(); ++i) {
    c_result.contours[i].resize(this->contours[i].size());
    for (size_t j = 0; j < this->contours[i].size(); ++j) {
      c_result.contours[i][j] = this-> transform_from_world_to_plane.inverse() *T12.inverse()*transform_from_world_to_plane * this->contours[i][j];
    }
  }


}







}//namespace
