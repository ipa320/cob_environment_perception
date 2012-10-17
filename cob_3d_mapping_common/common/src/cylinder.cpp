/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Thomas Zwölfer, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 06/2012
*
* \brief
* Class representing cylinder shapes
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
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

  computeAttributes(sym_axis, normal, origin_); 
  contours.resize(in_list.size());

  for (size_t j = 0; j < in_list.size(); j++) 
  {

    contours[j].resize(in_list[j].size());
    holes.resize(in_list[j].size());
    for (size_t k = 0; k < in_list[j].size(); k++) 
    {
      contours[j][k] =in_list[j][k];
    }
  }
}



void
Cylinder::ParamsFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud , std::vector<int>& indices)
{

  //  Establish coordinate system for projection to horizontal plane
  Eigen::Affine3f trafo_hor2w;
  Eigen::Vector3f centroid3f ;
  centroid3f <<centroid[0] , centroid[1] , centroid[2];
  ////  transform  points to horizontal coordinate system
  this->getTransformationFromPlaneToWorld(sym_axis,centroid3f,trafo_hor2w);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
  pcl::transformPointCloud(*in_cloud,indices,*trans_cloud,trafo_hor2w.inverse());

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
  seg.segment(inliers,coeff);


  //  origin in lcs
  Eigen::Vector3f l_origin,l_centroid;
  l_centroid = trafo_hor2w.inverse() * centroid3f;
  l_origin << coeff.values[0],coeff.values[1],l_centroid[2];

  //Parameters  in wcs

  origin_ = trafo_hor2w * l_origin  ;
  normal= centroid3f - origin_;
  r_=coeff.values[2];
  sym_axis.normalize();
  normal.normalize();
  this->computeAttributes(sym_axis, normal, origin_);
}

void
Cylinder::ParamsFromShapeMsg()
{

  if (sym_axis[2] < 0 )
  {
    sym_axis= sym_axis *-1;
  }

  sym_axis.normalize();
  normal.normalize();
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
  this->getTransformedContours(this->transform_from_world_to_plane, trans_contours);
  for (size_t i = 0; i < trans_contours.size(); ++i) {
    for (size_t j = 0; j < trans_contours[i].size(); ++j) {
      if (trans_contours[i][j][1]< min) min = trans_contours[i][j][1];
      if (trans_contours[i][j][1]> max) max = trans_contours[i][j][1];
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
  this->color=c_src.color;
}


//################## methods to roll and unroll cylinder###############

void Cylinder::getCyl3D(Cylinder& c3d)
{

  c3d= *this;
  c3d.makeCyl3D();

}



void Cylinder::makeCyl3D() 
{
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
      getPt3D(poly_plane.contours[j][k],point_temp);
     // alpha = poly_plane.contours[j][k][0]/ r_ ;
     // //         use polar coordinates to create cylinder points
     // point_temp <<  r_ * sin(-alpha), poly_plane.contours[j][k][1],  r_*  cos(-alpha);

     // //        transform back in world system
      point_temp = transform_from_world_to_plane.inverse() * point_temp;
      contours[j][k] = point_temp;
      //      contours3D[j][k] = contours[j][k];
    }
  }
}

void Cylinder::getPt3D(Eigen::Vector3f& pt2d,Eigen::Vector3f& pt3d){

      double alpha = pt2d[0]/ r_ ;
      //         use polar coordinates to create cylinder points
       pt3d<<  r_ * sin(-alpha), pt2d[1],  r_*  cos(-alpha);

}

void Cylinder::makeCyl2D()
{
  bool start; // bool to indicate first point of contour
  float Tx_1,Tx_0;
  Eigen::Vector3f z_axis,p_0;


  for (size_t j = 0; j < contours.size(); j++) 
  {

    Tx_0 = 0;

    for (size_t k = 0; k < contours[j].size(); k++) 
    {

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
      getArc(point_trans,z_axis,Tx_1,start);

      // New coordinates( p_y = 0 because now on a plane)
      point_trans[0] =  Tx_0+ Tx_1;
      point_trans[2] = 0;
      Eigen::Vector3f point_global =
          transform_from_world_to_plane.inverse()
          * point_trans;

      Tx_0 = point_trans[0];
      z_axis =contours[j][k];

      contours[j][k] = point_global;

    }
  }
}

void
Cylinder::getCyl2D(Cylinder& c2d)
{
  c2d = *this;
  c2d.makeCyl2D();
}


//################## methods for merging############################

void
Cylinder::isMergeCandidate(const std::vector<Cylinder::Ptr>& cylinder_array,
    const merge_config& limits, std::vector<int>& intersections) 
{

  for (size_t i = 0; i < cylinder_array.size(); i++) 
  {
    Cylinder& c_map = *(cylinder_array[i]);
    Eigen::Vector3f connection=c_map.origin_-origin_;
    connection.normalize();
    Eigen::Vector3f d= c_map.origin_  - this->origin_   ;

    // Test for geometrical attributes of cylinders
    if(fabs(c_map.sym_axis.dot(this->sym_axis)) > limits.angle_thresh && (fabs(c_map.r_ - r_) < (0.1 )))
    {
      // Test for spatial attributes of cylinders
      if( d.norm() < (c_map.r_+0.1) || fabs(c_map.sym_axis .dot(connection)) > limits.angle_thresh )
      {
        Cylinder::Ptr c1(new Cylinder);
        Cylinder::Ptr c2(new Cylinder);
        *c1 = *this;


        *c2= c_map;
        c2->transform_from_world_to_plane = c1->transform_from_world_to_plane;
        c1->makeCyl2D();

        c2->makeCyl2D();

        if (c1->isIntersectedWith(c2))
        {
          intersections.push_back(i);
        }
      }
    }
  }
}


void
Cylinder::merge(std::vector<Cylinder::Ptr>& c_array) 
{
  std::cout << "START MERGING" <<std::endl;
  std::vector<Cylinder::Ptr> merge_cylinders;

  //create average cylinder for  averaging
  Cylinder::Ptr average_cyl =Cylinder::Ptr(new Cylinder());
  *average_cyl = *this;
  average_cyl->GrabParams(*this);
  average_cyl->applyWeighting(c_array);

  this->compensate_offset(average_cyl);
  this->makeCyl2D();

  for (int i = 0; i < (int) c_array.size(); i++) 
  {
    c_array[i]->compensate_offset(average_cyl);
    c_array[i]->makeCyl2D();
    merge_cylinders.push_back(c_array[i]);
  }

  std::vector<Polygon::Ptr> merge_polygons;
  for (size_t i = 0; i < merge_cylinders.size(); ++i) 
  {
    Polygon::Ptr tmp_ptr= merge_cylinders[i];
    merge_polygons.push_back(tmp_ptr);
  }
  Polygon::Ptr average_polygon= average_cyl;

  this->merge_union(merge_polygons,average_polygon);
  this->assignID(merge_polygons);
  this->GrabParams(*average_cyl);
  this->assignWeight();
  this->makeCyl3D();
}




void
Cylinder::applyWeighting(std::vector<Cylinder::Ptr>& merge_candidates)
{

  Eigen::Vector3f temp_sym_axis,temp_origin,temp_normal;
  temp_sym_axis = this->merge_weight_ * this->sym_axis;
  temp_origin = this->merge_weight_ * this->origin_;
  temp_normal = this->merge_weight_ * this->normal;

  double   merge_weight_sum = this ->merge_weight_;
  double temp_r = merge_weight_sum * this->r_;
  int merged_sum = this->merged;


  for (int i = 0; i < (int)merge_candidates.size(); ++i) {


    temp_sym_axis += merge_candidates[i]->merge_weight_ * merge_candidates[i]->sym_axis ;
    temp_normal += merge_candidates[i]->merge_weight_ * merge_candidates[i]->normal ;
    temp_origin += merge_candidates[i]->merge_weight_ * merge_candidates[i]->origin_;
    temp_r += merge_candidates[i]->merge_weight_ * merge_candidates[i]->r_;
    merge_weight_sum += merge_candidates[i]->merge_weight_;
    merged_sum  += merge_candidates[i]->merged;

    if (merge_candidates[i]->h_max_ > this-> h_max_)this->h_max_ = merge_candidates[i]->h_max_;
    if (merge_candidates[i]->h_min_  < this-> h_min_)this->h_min_ = merge_candidates[i]->h_min_;

  }

  this->sym_axis = temp_sym_axis / merge_weight_sum;
  this->normal = temp_normal / merge_weight_sum;
  this->origin_ = temp_origin / merge_weight_sum;
  this->sym_axis.normalize();
  this->r_ = temp_r / merge_weight_sum;

  if (merged_sum < merged_limit)
  {
    this->merged=merged_sum;

  }
  else
  {
    this->merged=merged_limit;
  }

  Eigen::Affine3f transform_from_plane_to_world;
  getTransformationFromPlaneToWorld(this->sym_axis,this->normal,this->origin_,transform_from_plane_to_world);
  this->transform_from_world_to_plane=transform_from_plane_to_world.inverse();
}

//##############Methods for Debug #################################################

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
Cylinder::dump_params(std::string  name)
{

  std::string path = "/home/goa-tz/eval/params/";
  path.append(name.c_str());
  std::ofstream os(path.c_str(),std::ofstream::app );

  os<<frame_stamp<<" "<<r_<<" "<<
  this->origin_[0]<<" "<<this->origin_[1]<<" "<<this->origin_[2]<<" "<<
  this->centroid[0]<<" "<<this->centroid[1]<<" "<<this->centroid[2]<<" "<<
  this->normal[0]<<" "<<this->normal[1]<<" "<<this->normal[2]<<" "<<
  this->sym_axis[0]<<" "<<this->sym_axis[1]<<" "<<this->sym_axis[2]<<"\n";
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


void Cylinder::getArc(const Eigen::Vector3f& goal,const Eigen::Vector3f& start, float& Tx,bool first) 
{
  Eigen::Vector2f a, b;
  a << start[0], start[2];
  b << goal[0],goal[2];
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
    if (first==true) {
        alpha = acos((fabs(cos_alpha)));
    }
    else
    {
      alpha = fabs(acos((cos_alpha)));
    }

    //        make sure alpha < M_PI
    if (alpha >= M_PI) {
      alpha -=M_PI;
    }
    Tx =  r_ *  alpha;

    Eigen::Vector2f  d_ba=b-a;

    //    1st section ---> above x axis

    if ((a[1]>=0 && b[1]>=0) && (a[0]<b[0]) ) {
      Tx*=-1;
    }

    //   2nd section --> positive x values
    else if ((a[0]>=0 && b[0]>=0) && (a[1]> b[1])) {
      Tx*=-1;

    }

    //    3rd section --> negative y values
    else if ((a[1]<=0 && b[1]<=0) && (a[0]>b[0])) {
      Tx*=-1;

    }
    //   4th section --> negative x values
    else if ((a[0]<=0 && b[0]<=0) && (a[1]< b[1])) {
      Tx*=-1;
    }
  }
}

void
Cylinder::compensate_offset(Cylinder::Ptr& c_ref)
{
    Eigen::Vector3f n12 = c_ref->transform_from_world_to_plane.rotation()* c_ref->normal;                          
    Eigen::Vector3f s12 = c_ref->transform_from_world_to_plane.rotation()* this->sym_axis;
                                        
    Eigen::Vector3f o12 = c_ref->transform_from_world_to_plane* this->origin_;
    o12[1]=0;
                                                      
    Eigen::Affine3f T12;
    n12.normalize();
    s12.normalize();
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(s12,n12,o12,T12);
                                                              
    for (size_t i = 0; i < this->contours.size(); ++i) 
    {
        for (size_t j = 0; j < this->contours[i].size(); ++j)
        {
            this->contours[i][j] = this-> transform_from_world_to_plane.inverse() * T12.inverse() * transform_from_world_to_plane * this->contours[i][j];
        }
    }
   this->transform_from_world_to_plane=c_ref->transform_from_world_to_plane;                                                       
                                                           
}

    
}//namespace
