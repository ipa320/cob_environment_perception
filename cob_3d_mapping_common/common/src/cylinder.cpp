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


//void
//Cylinder::Cylinder(Cylinder c)
//{
//  c.GrabParams(*this);
//  c.contours
//}

void
Cylinder::ContoursFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{

  /* Assign Contours to Cylinder using border points
   * Transformation to internal representation of cylinder
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



      contours[j][k] =in_list[j][k];

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



  //  Eigen::Vector3f o2p = in_cloud->points[indices[0]].getVector3fMap()-origin_;
  //
  //  normal[1] =( sym_axis[0]*o2p[0]+sym_axis[2]*o2p[2])/-sym_axis[1];
  //  normal[0] = o2p[0];
  //  normal[2] = o2p[2];

  normal= centroid3f - origin_;
  //  normal = origin_.unitOrthogonal();





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



  origin_=new_origin;
  this ->sym_axis = sym_axis;




  d=fabs(this->centroid.head(3).dot(normal));



  normal=new_normal;

  Eigen::Affine3f transform_from_plane_to_world;
  getTransformationFromPlaneToWorld(sym_axis,normal,origin_,transform_from_plane_to_world);
  transform_from_world_to_plane=transform_from_plane_to_world.inverse();


  //  calculation of h_min and h_max
  double min,max;
  min = 1000;
  max = -1000;
  std::vector<std::vector<Eigen::Vector3f> > trans_contours;
  trans_contours = this->getTransformedContours(this->transform_from_world_to_plane);
  for (size_t i = 0; i < trans_contours.size(); ++i) {
    for (size_t j = 0; j < trans_contours[i].size(); ++j) {
      if (trans_contours[i][j][1]< min) min = trans_contours[i][j][1];
      if (trans_contours[i][j][1]> min) max = trans_contours[i][j][1];
    }
  }

  h_max_ = max;
  h_min_ = min;





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

void Cylinder::getCyl3D(Cylinder& c3d)
{

  c3d= *this;
  c3d.makeCyl3D();

}


void Cylinder::makeCyl3D() {
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
      alpha = poly_plane.contours[j][k][0]/ r_ ;


      //      if (alpha > 2* M_PI) {
      //        alpha = alpha- 2*M_PI;
      //      }

      //         use polar coordinates to create cylinder points
      point_temp <<  r_ * sin(-alpha), poly_plane.contours[j][k][1],  r_*  cos(-alpha);
      //      point_temp = poly_plane.contours[j][k];


      //        transform back in world system
      point_temp = transform_from_world_to_plane.inverse() * point_temp;




      contours[j][k] = point_temp;
      //      contours3D[j][k] = contours[j][k];




    }
  }

}



void Cylinder::makeCyl2D(bool debug)
{
  /*
   * Convert Contours in cylindrical shape to polygonial shape
   */




  bool start; // bool to indicate first point of contour
  float Tx_1,Tx_0;
  Eigen::Vector3f z_axis,p_0;


  for (size_t j = 0; j < contours.size(); j++) {

    Tx_0 = 0;

    for (size_t k = 0; k < contours[j].size(); k++) {

//      std::cout<<"\n k= "<<k<<"\n";
      Tx_1=0;


      //      Transform  Points in Cylinder Coordinate System
      Eigen::Vector3f point_trans =
          transform_from_world_to_plane * contours[j][k];



      if (k == 0) {
        z_axis << 0,0,1;
        start=true;
      }

      else {
        start = false;
        z_axis = transform_from_world_to_plane *z_axis;
      }
      //      Eigen::Vector3f trans_origin = transform_from_world_to_plane*origin_;
      //      Eigen::Vector3f check = (trans_origin +r_*z_axis);
      //      float d = sqrt((trans_origin[0]-check[0])*(trans_origin[0]-check[0]) +
      //          (trans_origin[1]-check[1])*(trans_origin[1]-check[1]) +
      //          (trans_origin[2]-check[2])*(trans_origin[2]-check[2]) );
      //
      //      if(d >r_ )
      //      {
      //        std::cout<<"THRESH EXCEEDED-----------------\n";
      //      }

      //      if (k==0) {
      //        debug=true;
      //      }
      //      else {
      //        debug = true;
      //      }
      getTrafo2d(point_trans,z_axis,Tx_1,debug,start);





      // New coordinates( p_y = 0 because now on a plane)
      point_trans[0] =  Tx_0+ Tx_1;
      point_trans[2] = 0;
      //        transform back in world system
      //      std::cout<<"coord"<<point_trans<<std::endl;
      //      std::cout<<"TX0"<<point_trans[0];

      Eigen::Vector3f point_global =
          transform_from_world_to_plane.inverse()
          * point_trans;

      Tx_0 = point_trans[0];
      z_axis =contours[j][k];

      contours[j][k] = point_global;

    }

  }

}
void Cylinder::getCyl2D(Cylinder& c2d,bool debug)
{

  c2d = *this;
  c2d.makeCyl2D(debug);

}



//################## methods for merging############################


void Cylinder::isMergeCandidate(const std::vector<CylinderPtr>& cylinder_array,
    const merge_config& limits, std::vector<int>& intersections) {


  for (size_t i = 0; i < cylinder_array.size(); i++) {

    Cylinder& c_map = *(cylinder_array[i]);


    Eigen::Vector3f connection=c_map.origin_-origin_;
    connection.normalize();
    
    // Test for geometrical attributes of cylinders
    if (
        fabs(c_map.sym_axis.dot(this->sym_axis) > limits.angle_thresh)
         && fabs(c_map.r_ - r_) < (0.01 )
         )
    {
         // Test for spatial attributes of cylinders
         Eigen::Vector3f d= c_map.origin_  - this->origin_   ;
         if( d.norm() < 0.05 || fabs(c_map.sym_axis .dot(connection)) > limits.angle_thresh )
         {


         Cylinder c1,c2;

         c1 = *this;




         c_map.transformToTarget(c1,c2);
         c1.makeCyl2D(false);
         c1.debug_output("MC_NEW");

         c2.makeCyl2D(false);
         c2.debug_output("MC_MAP");






         bool is_intersected = c1.isMergeCandidate_intersect(
             c2);

         if (is_intersected == false) {
           continue;
         }
         if (is_intersected == true) {
           intersections.push_back(i);
         }

    }}//if
    else
    {
      std::cout<<"merge_criteria_not fulfilled!!\n";
    }

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
  std::cout<<"BEFORE radius  ="<< this->r_<<"\n";

  *average_cyl = *this;

  average_cyl->applyWeighting(c_array);
  //  average_cyl->recomputeNormal();
  //  transform  to local system


  //  this->getShiftedCylinder(*c_array[0],*average_cyl,*shifted_cylinder,true);

  std::cout<<"AFTER radius  ="<< this->r_<<"\n";
  this->transformToTarget(*average_cyl,*this);


  bool debug = true;
  if(debug == true)
  {
    std::string s1;
    s1= "s1.c";

    this->debug_output(s1);

  }
  this->makeCyl2D(false);

  if(debug == true)
  {
    std::string s1;

    s1= "s1.p";

    this->debug_output(s1);

  }


  for (int i = 0; i < (int) c_array.size(); i++) {

    //    std::cout<<"//////MERGE SIZE = "<<c_array.size()<<"\n";
    //    Cylinder & c_map = *c_array[i];

    //shifted cylinder is computed with respect to "this"- system


    //    c_map.getShiftedCylinder(*c_array[0],*average_cyl,*shifted_cylinder,true);
    c_array[i]->transformToTarget(*average_cyl,*c_array[i]);


    if(debug ==true)
    {
      std::string s2;
      s2= "s2.c";

      c_array[i]->debug_output(s2);
    }
    c_array[i]->makeCyl2D(false);


    if(debug ==true)
    {
      std::string s2;

      s2= "s2.p";

      c_array[i]->debug_output(s2);
    }


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

  if(debug ==true)
  {
    std::string s3;

    s3= "s3.p";

    this->debug_output(s3);
  }
  //  average_cyl->recomputeNormal();

  this->makeCyl3D();


  if(debug ==true)
  {
    std::string s3;

    s3= "s3.c";

    this->debug_output(s3);
  }
  std::cout<<this->merge_weight_<<"\n";
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


  Eigen::Vector3f temp_sym_axis,temp_origin,temp_normal;

  temp_sym_axis = this->merge_weight_ * this->sym_axis;
  temp_origin = this->merge_weight_ * this->origin_;
  temp_normal = this->merge_weight_ * this->normal;


  double   merge_weight_sum = this ->merge_weight_;
  double temp_r = merge_weight_sum * this->r_;
  int merged_sum = this->merged;

  std::cout<<"NEW params"<<"\n";
  std::cout<<"merged = "<<this->merged<<"\n";
  std::cout<<"r = "<<this->r_<<"\n";
  std::cout<<"normal = "<<this->normal[0]<<" "<<this->normal[1]<<" "<<this->normal[2]<<" "<<"\n";
  std::cout<<"origin_ = "<<this->origin_[0]<<" "<<this->origin_[1]<<" "<<this->origin_[2]<<" "<<"\n";



  for (int i = 0; i < (int)merge_candidates.size(); ++i) {

    std::cout<<"OLD params"<<"\n";
    std::cout<<"merged = "<<merge_candidates[i]->merged<<"\n";
    std::cout<<"r = "<<merge_candidates[0]->r_<<"\n";
    std::cout<<"normal = "<<merge_candidates[i]->normal[0]<<" "<<merge_candidates[i]->normal[1]<<" "<<merge_candidates[i]->normal[2]<<" "<<"\n";
    std::cout<<"origin_ = "<<merge_candidates[i]->origin_[0]<<" "<<merge_candidates[i]->origin_[1]<<" "<<merge_candidates[i]->origin_[2]<<" "<<"\n";



    temp_sym_axis += merge_candidates[i]->merge_weight_ * merge_candidates[i]->sym_axis ;

    temp_normal += merge_candidates[i]->merge_weight_ * merge_candidates[i]->normal ;


    temp_origin += merge_candidates[i]->merge_weight_ * merge_candidates[i]->origin_;

    temp_r += merge_candidates[i]->merge_weight_ * merge_candidates[i]->r_;
    merge_weight_sum += merge_candidates[i]->merge_weight_;
    merged_sum  += merge_candidates[i]->merged;

    if (merge_candidates[i]->h_max_ > this-> h_max_)this->h_max_ = merge_candidates[i]->h_max_;
    if (merge_candidates[i]->h_max_  < this-> h_min_)this->h_min_ = merge_candidates[i]->h_max_;


  }

  this->sym_axis = temp_sym_axis / merge_weight_sum;

  this->normal = temp_normal / merge_weight_sum;


  this->origin_ = temp_origin / merge_weight_sum;
  this->sym_axis.normalize();

  this->r_ = temp_r / merge_weight_sum;

  merged_limit=50;
  if (merged_sum < merged_limit)
  {
    this->merged=merged_sum;

  }
  else
  {
    this->merged=merged_limit;
  }


  std::cout<<"AVERAGE params"<<"\n";
  std::cout<<"merged = "<<this->merged<<"\n";
  std::cout<<"r = "<<this->r_<<"\n";
  std::cout<<"normal = "<<this->normal[0]<<" "<<this->normal[1]<<" "<<this->normal[2]<<" "<<"\n";
  std::cout<<"origin_ = "<<this->origin_[0]<<" "<<this->origin_[1]<<" "<<this->origin_[2]<<" "<<"\n";


  //  Eigen::Vector3f x_axis;
  //  x_axis << this->sym_axis[1], - this->sym_axis[0], this->sym_axis[2];
  //  this->normal = this->sym_axis.cross(x_axis);

  //  !overide! use initial values for normal and sym_axis
//    this->normal = merge_candidates[0]->normal;
  //    this->sym_axis = merge_candidates[0]-> sym_axis;
  //    this->origin_ = merge_candidates[0]-> origin_;



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
Cylinder::dump_params(std::string  name){


  std::string path = "/home/goa-tz/debug/eval/";
  path.append(name.c_str());
  std::ofstream os(path.c_str(),std::ofstream::app );

  //  os<<"$ r  origin  centroid  normal  sym_axis";
  os<<frame_stamp<<" "<<r_<<" "<<
      this->origin_[0]<<" "<<this->origin_[1]<<" "<<this->origin_[2]<<" "<<
      this->centroid[0]<<" "<<this->centroid[1]<<" "<<this->centroid[2]<<" "<<
      this->normal[0]<<" "<<this->normal[1]<<" "<<this->normal[2]<<" "<<
      this->sym_axis[0]<<" "<<this->sym_axis[1]<<" "<<this->sym_axis[2]<<"\n";
  os.close();
//  std::string points= "points";
//  path.append(points.c_str());
//  std::ofstream os_points(path.c_str(),std::ofstream::app );
//
//  for (size_t i = 0; i < this->contours.size(); ++i) {
//    for (size_t j = 0; j < this->contours[i].size(); ++j) {
//
//      os_points << this->contours[i][j][0]<<" "<< this->contours[i][j][2]<<" "<< this->contours[i][j][2]<<"\n";
//
//    }
//  }
//  os_points.close();


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


void Cylinder::getTrafo2d(const Eigen::Vector3f& vec_new,const Eigen::Vector3f& vec_old, float& Tx,bool debug,bool start) {

  //  calculation of translation Tx and Ty
  //
  //  Tx by meansof arc length
  //  Tx corresponds to radius of cylinder
  //  float cos_alpha;



  //x and z components of point in local system


  //  double a,b;
  //  //  angle between y-axis and point-origin connection
  //  if (vec_new[0]> vec_old[0]) {
  //    a = vec_new[0]-vec_old[0];
  //    b = vec_new[2]-vec_old[2];
  //
  //  }
  //  else {
  //    a = vec_old[0]-vec_new[0];
  //    b = vec_old[2]-vec_new[2];
  //
  //  }
  //  double  alpha = atan2(b,a);

  Eigen::Vector2f a, b;
  a << vec_old[0], vec_old[2];
  b << vec_new[0],vec_new[2];
  a.normalize();
  b.normalize();


  double alpha = 0;
  double cos_alpha = a.dot(b) / (b.norm() * a.norm());
  if (cos_alpha > 0.99995)
  {
    Tx = 0;
  }
  else
  {


    if (start==true) {
      alpha = acos((fabs(cos_alpha)));

    }
    else
    {
      alpha = fabs(acos((cos_alpha)));
    }

    //    std::cout << "alpha = " << alpha << std::endl;
    //        make sure alpha < M_PI
    if (alpha >= M_PI) {
      alpha -=M_PI;
    }
    Tx =  r_ *  alpha;



    Eigen::Vector2f  d_ba=b-a;
    //    if (b[0]< 0 && b[2 ]<0) {
    //      alpha = alpha-M_PI/2;
    //    }



    //    1st section ---> above x axis

    if ((a[1]>=0 && b[1]>=0) && (a[0]<b[0]) ) {
//      std::cout<<"sec 1\n";
      Tx*=-1;
    }

    //   2nd section --> positive x values
    else if ((a[0]>=0 && b[0]>=0) && (a[1]> b[1])) {
      Tx*=-1;
//      std::cout<<"sec 2\n";

    }

    //    3rd section --> negative y values
    else if ((a[1]<=0 && b[1]<=0) && (a[0]>b[0])) {
      Tx*=-1;
//      std::cout<<"sec3\n";

    }
    //   4th section --> negative x values
    else if ((a[0]<=0 && b[0]<=0) && (a[1]< b[1])) {
      Tx*=-1;
//      std::cout<<"sec 4\n";

    }



    //    //    1st case
    //    if ((a[1]>0 && b[1]> 0) && (a[0]>0 && b[0]>0)) {
    //      if (a[1]>b[1]) {
    //        Tx = -Tx;
    //      }
    //    }
    //
    //
    //    //    2nd case
    //    if ((a[1]>0 && b[1]>0) && (a[0]<0 && b[0] > 0)) {
    //
    //      Tx = -Tx;
    //    }
    //
    //    //    3rd case
    //    if ((a[1]>0 && b[1]>0) && (a[0]<0 && b[0]) < 0) {
    //      if (a[1]<b[1]) {
    //        Tx = -Tx;
    //      }
    //    }
    //
    //    //    4th case
    //    if ((a[1]<0 && b[1]>0) && (a[0]<0 && b[0] < 0)) {
    //
    //      Tx =   -Tx;
    //    }
    //
    //    //    5th case
    //    if ((a[1]<0 && b[1]<0) && (a[0]<0 && b[0]) < 0) {
    //      if (a[1]<b[1]) {
    //        Tx =  -Tx;
    //      }
    //    }
    //
    //    //    6th case
    //    if ((a[1]<0 && b[1]<0) && (a[0]>0 && b[0] < 0)) {
    //
    //      Tx = -Tx;
    //    }
    //
    //    //    7th case
    //    if ((a[1]<0 && b[1]<0) && (a[0]>0 && b[0]) > 0) {
    //      if (a[1]>b[1]) {
    //        Tx = -Tx;
    //      }
    //    }
    //
    //
    //    //    8th case
    //    if ((a[1]<0 && b[1]>0) && (a[0]<0 && b[0] < 0)) {
    //
    //      Tx = -Tx;
    //    }




    //    if (a[1] < 0 && b[1]> 0) {
    //      Tx = -Tx;
    ////      std::cout<<"corrected\n";
    //    }
    //    if (a[0] > 0 && b[0]< 0) {
    //      Tx = -Tx;
    ////      std::cout<<"corrected\n";
    //
    //    }
    //
    //
    //    else if (d_ba[0]<0 && d_ba[1]<0) {
    //      Tx = -Tx;
    ////      std::cout<<"corrected\n";
    //
    //
    //    }
    //    else if (a[1]<0 || b[1]<0) {
    //      if (a[0]<b[0]) {
    //        Tx = -Tx;
    ////        std::cout<<"corrected\n";
    //
    //      }
    //    }
    //    else {
    //      if (a[0]>b[0])
    //      {
    //        Tx = -Tx;
    ////        std::cout<<"corrected\n";
    //
    //      }
    //    }

  }



  if (debug == true) {
    //  Debug Output
    std::cout << " avec NEW\n"<<b << std::endl;
    std::cout << " avec OLD\n" << a<<std::endl;
    std::cout << "D avec\n" << b[0]-a[0]<<"\n"<<b[1]-a[1] << std::endl;
    std::cout << "alpha = " << alpha << std::endl;
    std::cout<<"TX = "<<Tx<<std::endl;

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
Cylinder::transformToTarget(Cylinder& c_target,Cylinder& c_result)
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
  n12.normalize();
  s12.normalize();
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(s12,n12,o12,T12);

  //debug output
  float roll,pitch, yaw;
  pcl::getEulerAngles(T12,roll,pitch,yaw);
  Eigen::Vector3f  t = pcl::getTranslation(T12);

  std::cout<<"----TRAFO 2 TARGET-----\n";
  std::cout<<"roll pitch yaw"<<roll <<" "<< pitch <<" "<< yaw <<" \n";
  std::cout<<"trans "<<t<<"\n";


  c_result.contours.resize(this->contours.size());
  for (size_t i = 0; i < this->contours.size(); ++i) {
    c_result.contours[i].resize(this->contours[i].size());
    for (size_t j = 0; j < this->contours[i].size(); ++j) {
      c_result.contours[i][j] = this-> transform_from_world_to_plane.inverse() *T12.inverse()*transform_from_world_to_plane * this->contours[i][j];
    }
  }


}

void Cylinder::get_thresh(const Eigen::Vector3f& vec_1,const Eigen::Vector3f& vec_2,double& thresh)
{



  double dx= fabs(vec_2[0]-vec_1[0]);
  double dz = fabs(vec_2[2]-vec_1[2]);
  thresh = 3* sqrt(dx*dx + dz*dz);


  std::cout<<"THRESH= "<<thresh<<"\n";


}



}//namespace
