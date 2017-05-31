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
#include <ros/console.h>
#include "cob_3d_mapping_common/cylinder.h"
#include <math.h>
#include <pcl/io/pcd_io.h>

namespace cob_3d_mapping {


  double
  radiusAndOriginFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud ,
                  std::vector<int>& indices,
                  Eigen::Vector3f& origin,
                  const Eigen::Vector3f& sym_axis)
  {
    //  Transform into cylinder coordinate frame
    Eigen::Affine3f t;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(sym_axis.unitOrthogonal(), sym_axis, origin, t);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_trans( new pcl::PointCloud<pcl::PointXYZRGB>() );
    pcl::transformPointCloud(*in_cloud, indices, *pc_trans, t);

    // Inliers of circle model
    pcl::PointIndices inliers;
    // Coefficients of circle model
    pcl::ModelCoefficients coeff;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optimize coefficients
    seg.setOptimizeCoefficients (true);
    // Set type of method
    //seg.setMethodType (pcl::SAC_MLESAC);
    seg.setMethodType (pcl::SAC_RANSAC);
    // Set number of maximum iterations
    seg.setMaxIterations (10);
    // Set type of model
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    // Set threshold of model
    seg.setDistanceThreshold (0.010);
    // Give as input the filtered point cloud
    seg.setInputCloud (pc_trans/*in_cloud*/);
    //boost::shared_ptr<std::vector<int> > indices_ptr(&indices);
    //seg.setIndices(indices_ptr);
    // Call the segmenting method
    seg.segment(inliers, coeff);


    // origin in cylinder coordinates
    Eigen::Vector3f l_origin;
    l_origin << coeff.values[0],coeff.values[1],0;
    origin = t.inverse() * l_origin;

    return coeff.values[2];
  }

  //##############Methods to initialize cylinder and its parameters#########


  Cylinder::Cylinder(unsigned int id,
                     Eigen::Vector3f origin,
                     Eigen::Vector3f sym_axis,
                     double radius,
                     std::vector<std::vector<Eigen::Vector3f> >& contours_3d,
                     std::vector<bool> holes,
                     std::vector<float> color)
   : Polygon()
  {
    //Cylinder();
    id_ = id;
    sym_axis_ = sym_axis;
    r_ = radius;
    holes_ = holes;
    color_ = color;
    if (sym_axis_[2] < 0 )
    {
      sym_axis_ = sym_axis_ * -1;
    }
    computePose(origin, contours_3d);
    setContours3D(contours_3d);
    computeHeight();
  }

  Cylinder::Cylinder(unsigned int id,
                     Eigen::Vector3f origin,
                     Eigen::Vector3f sym_axis,
                     double radius,
                     std::vector<pcl::PointCloud<pcl::PointXYZ> >& contours_3d,
                     std::vector<bool> holes,
                     std::vector<float> color)
  : Polygon()
  {
    std::vector<std::vector<Eigen::Vector3f> > contours_eigen;
    for(unsigned int i = 0; i < contours_3d.size(); i++)
    {
      std::vector<Eigen::Vector3f> c_eigen;
      for(unsigned int j = 0; j < contours_3d[i].size(); j++)
      {
        Eigen::Vector3f pt = contours_3d[i].points[j].getVector3fMap();
        c_eigen.push_back(pt);
      }
      contours_eigen.push_back(c_eigen);
    }
    id_ = id;
    sym_axis_ = sym_axis;
    r_ = radius;
    //std::cout << "origin " << origin << std::endl;
    //std::cout << "r " << r_ << std::endl;
    holes_ = holes;
    color_ = color;
    if (sym_axis_[2] < 0 )
    {
      sym_axis_ = sym_axis_ * -1;
    }
    computePose(origin, contours_eigen);
    setContours3D(contours_eigen);
    computeHeight();
    //Cylinder(id, origin, sym_axis, radius, contours_eigen, holes, color);
  }


  /*void
  Cylinder::computePose(Eigen::Vector3f origin)
  {
    Eigen::Affine3f t;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(sym_axis_, sym_axis_.unitOrthogonal(), origin, t);
    pose_ = t.inverse();
  }*/

    void
  Cylinder::setContours3D(std::vector<std::vector<Eigen::Vector3f> >& contours_3d)
  {
    contours_.clear();
    for(unsigned int i = 0; i < contours_3d.size(); i++)
    {
      std::vector<Eigen::Vector2f> c;
      for(unsigned int j = 0; j < contours_3d[i].size(); j++)
      {
        Eigen::Vector3f pt = (pose_.inverse()*contours_3d[i][j]);
        //std::cout << contours_3d[i][j](0) << "," << contours_3d[i][j](1) << "," << contours_3d[i][j](2) << " ==> ";
        //std::cout << pt(0) << "," << pt(1) << "," << pt(2) << " ==> ";
        float alpha = atan2(pt(0),pt(2));
        Eigen::Vector2f pt_2d;
        pt_2d(0) = r_*alpha;
        pt_2d(1) = pt(1);
        //std::cout << pt_2d(0) << "," << pt_2d(1) << std::endl;
        c.push_back(pt_2d);
      }
      contours_.push_back(c);
    }
    /*for(unsigned int i = 0; i < contours_.size(); i++)
    {
      for(unsigned int j = 0; j < contours_[i].size(); j++)
      {
        std::cout << contours_[i][j](0) << "," << contours_[i][j](1) << std::endl;
      }
    }*/
  }

std::vector<std::vector<Eigen::Vector3f> >
  Cylinder::getContours3D()
  {
    std::vector<std::vector<Eigen::Vector3f> > contours_3d;
    for(unsigned int i = 0; i < contours_.size(); i++)
    {
      std::vector<Eigen::Vector3f> c;
      for(unsigned int j = 0; j < contours_[i].size(); j++)
      {
        Eigen::Vector3f pt;
        double alpha = contours_[i][j](0) / r_;
        pt(0) = sin(alpha)*r_;
        pt(1) = contours_[i][j](1);
        pt(2) = cos(alpha)*r_;
        c.push_back(pose_*pt);
      }
      contours_3d.push_back(c);
    }
    /*for(unsigned int i = 0; i < contours_.size(); i++)
    {
      for(unsigned int j = 0; j < contours_[i].size(); j++)
      {
        std::cout << contours_[i][j](0) << "," << contours_[i][j](1) << std::endl;
      }
    }*/
    return contours_3d;
  }

void
  Cylinder::computePose(Eigen::Vector3f origin, std::vector<std::vector<Eigen::Vector3f> >& contours_3d)
  {
    Eigen::Affine3f t;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(sym_axis_, sym_axis_.unitOrthogonal(), origin, t);
    Eigen::Vector3f z_cyl = t * contours_3d[0][0];
    z_cyl(1) = 0;
    Eigen::Vector3f z_axis = t.inverse().rotation() * z_cyl;
    computePose(origin, z_axis.normalized());
  }

  void
  Cylinder::computePose(Eigen::Vector3f origin, Eigen::Vector3f z_axis)
  {
    Eigen::Affine3f t;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(sym_axis_, z_axis, origin, t);
    pose_ = t.inverse();
  }

    void Cylinder::computeHeight()
  {
    //  calculation of h_min and h_max
    double min,max;
    min = 1000;
    max = -1000;
    //std::vector<std::vector<Eigen::Vector3f> > trans_contours;
    //this->getTransformedContours(this->transform_from_world_to_plane, trans_contours);
    for (size_t i = 0; i < contours_.size(); ++i) {
      for (size_t j = 0; j < contours_[i].size(); ++j) {
        if (contours_[i][j][1] < min) min = contours_[i][j][1];
        if (contours_[i][j][1] > max) max = contours_[i][j][1];
      }
    }
    h_max_ = max;
    h_min_ = min;
  }

  void
  Cylinder::updateAttributes(const Eigen::Vector3f& sym_axis, const Eigen::Vector3f& origin, const Eigen::Vector3f& z_axis)
  {
    //origin_= new_origin;
    sym_axis_ = sym_axis;
    if (sym_axis_[2] < 0 )
    {
      sym_axis_ = sym_axis_ * -1;
    }
    computePose(origin, z_axis.normalized());
    //d_ = fabs(pose_.translation().dot(normal_));
    //normal_ = new_normal;

    /*Eigen::Affine3f transform_from_plane_to_world;
  getTransformationFromPlaneToWorld(sym_axis,normal,origin_,transform_from_plane_to_world);
  transform_from_world_to_plane=transform_from_plane_to_world.inverse();*/
  }

  void Cylinder::setParamsFrom(Polygon::Ptr& p)
  {
    Cylinder::Ptr c(boost::dynamic_pointer_cast<Cylinder>(p));
    BOOST_ASSERT(c);
    this->pose_ = c->pose_;
    this->r_ = c->r_;
    this->sym_axis_ = c->sym_axis_;
    this->h_max_ = c->h_max_;
    this->h_min_ = c->h_min_;
    if(this->merged_ < 9) { this->merged_ = c->merged_; }
    else { this->merged_ = 9; }
  }

void
  Cylinder::transform(Eigen::Affine3f & trafo)
  {
    //transform contours
    //this->TransformContours(trafo);
    pose_ = trafo * pose_;

    //  transform parameters
    sym_axis_ = trafo.rotation() * sym_axis_;

    //Eigen::Vector3f tf_axes_2 = trafo.rotation() * this->normal_;
    //this->normal_ = tf_axes_2;

    //Eigen::Vector3f tf_origin = trafo * this->origin_;
    //this->origin_ =  tf_origin;

    /*Eigen::Vector3f centroid3f;
  centroid3f<<  this->centroid[0], this->centroid[1], this->centroid[2];
  centroid3f = trafo * centroid3f;
  this->centroid << centroid3f[0], centroid3f[1], centroid3f[2], 0;*/
    //this->computeAttributes(sym_axis_,normal_,origin_);

  }

  void
  Cylinder::triangulate(std::list<TPPLPoly>& tri_list) const
  {
    TPPLPartition pp;
    std::list<TPPLPoly> polys;
    TPPLPoly poly;
    TPPLPoint pt;

    double d_alpha = 0.5;
    double alpha_max = 0, alpha_min = std::numeric_limits<double>::max();
    for(size_t i = 0; i < contours_[0].size(); ++i)
    {
      double alpha = contours_[0][i](0) / r_;
      if (alpha > alpha_max) alpha_max = alpha;
      if (alpha < alpha_min) alpha_min = alpha;
    }
    std::cout << "r " << r_ << std::endl;
    std::cout << "alpha " << alpha_min << "," << alpha_max << std::endl;
    std::vector<std::vector<std::vector<Eigen::Vector2f> > > contours_split;
    for(size_t j = 0; j < contours_.size(); j++)
    {
      for(double i = alpha_min + d_alpha; i <= alpha_max; i += d_alpha)
      {
        std::vector<Eigen::Vector2f> contour_segment;
        for(size_t k = 0; k < contours_[j].size(); ++k)
        {
          double alpha = contours_[j][k](0) / r_;
          if( alpha >= i - d_alpha - 0.25 && alpha < i + 0.25)
          {
            contour_segment.push_back(contours_[j][k]);
          }
        }
        //std::cout << "c " << j << i << " has " << contour_segment.size() << " points" << std::endl;
        if(contour_segment.size() < 3) continue;
        poly.Init(contour_segment.size());
        poly.SetHole(holes_[j]);
        for( unsigned int l = 0; l < contour_segment.size(); l++)
        {
          pt.x = contour_segment[l](0);
          pt.y = contour_segment[l](1);
          poly[l] = pt;
        }
        if (holes_[j])
          poly.SetOrientation(TPPL_CW);
        else
          poly.SetOrientation(TPPL_CCW);
        polys.push_back(poly);
      }
    }
    // triangulation into monotone triangles
    pp.Triangulate_EC (&polys, &tri_list);
  }

  void
  Cylinder::getMergeCandidates(const std::vector<Polygon::Ptr>& cylinder_array,
                               std::vector<int>& intersections)
  {

    for (size_t i = 0; i < cylinder_array.size(); i++)
    {
      Cylinder::Ptr c(boost::dynamic_pointer_cast<Cylinder>(cylinder_array[i]));
      BOOST_ASSERT(c);
      Cylinder& c_map = *c;
      Eigen::Vector3f connection = c_map.pose_.translation() - pose_.translation();
      //connection.normalize();
      //Eigen::Vector3f d= c_map.origin_  - this->origin_   ;

      // Test for geometrical attributes of cylinders
      if(fabs(c_map.sym_axis_.dot(sym_axis_)) > merge_settings_.angle_thresh && (fabs(c_map.r_ - r_) < (0.1 ))) //TODO: add param
      {
        // Test for spatial attributes of cylinders
        if( connection.norm() < (c_map.r_ + 0.1) || fabs(c_map.sym_axis_.dot(connection.normalized())) > merge_settings_.angle_thresh )
        {
          /*Cylinder::Ptr c1(new Cylinder);
        Cylinder::Ptr c2(new Cylinder);
           *c1 = *this;
           *c2 = c_map;
        c2->pose_ = c1->pose_;
        //c2->transform_from_world_to_plane = c1->transform_from_world_to_plane;
        c1->makeCyl2D();
        c2->makeCyl2D();*/
          if(isIntersectedWith(cylinder_array[i]))
            //if (c1->isIntersectedWith(c2))
          {
            intersections.push_back(i);
          }
        }
      }
    }
  }


void
  Cylinder::computeAverage(const std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr& p_average)
  {
    std::cout << merge_weight_ << "," << merged_ << std::endl;
    Eigen::Vector3f temp_sym_axis,temp_origin,temp_z_axis;
    temp_sym_axis = this->merge_weight_ * this->sym_axis_;
    temp_origin = this->merge_weight_ * this->pose_.translation();
    temp_z_axis = this->merge_weight_ * this->pose_.rotation() * Eigen::Vector3f(0, 0, 1);
    //temp_normal = this->merge_weight_ * this->normal_;

    double   merge_weight_sum = this ->merge_weight_;
    double temp_r = merge_weight_sum * this->r_;
    int merged_sum = this->merged_;

    double temp_h_max = h_max_, temp_h_min = h_min_;
    for (int i = 0; i < (int)poly_vec.size(); ++i)
    {
      Cylinder::Ptr c(boost::dynamic_pointer_cast<Cylinder>(poly_vec[i]));
      BOOST_ASSERT(c);
      std::cout << c->merge_weight_ << "," << c->merged_ << std::endl;
      temp_sym_axis += c->merge_weight_ * c->sym_axis_;
      //temp_normal += c->merge_weight_ * c->normal_;
      temp_z_axis += c->merge_weight_ * c->pose_.rotation() * Eigen::Vector3f(0, 0, 1);
      temp_origin += c->merge_weight_ * c->pose_.translation();
      temp_r += c->merge_weight_ * c->r_;
      merge_weight_sum += c->merge_weight_;
      merged_sum  += c->merged_;

      if (c->h_max_ > temp_h_max) temp_h_max = c->h_max_;
      if (c->h_min_ < temp_h_min) temp_h_min = c->h_min_;

    }
    Cylinder::Ptr c_avg(boost::dynamic_pointer_cast<Cylinder>(p_average));
    BOOST_ASSERT(c_avg);
    //c_avg->sym_axis_ = temp_sym_axis / merge_weight_sum;
    //c_avg->normal_ = temp_normal / merge_weight_sum;
    //c_avg->origin_ = temp_origin / merge_weight_sum;
    c_avg->sym_axis_.normalize();
    c_avg->r_ = temp_r / merge_weight_sum;
    c_avg->h_max_ = temp_h_max;
    c_avg->h_min_ = temp_h_min;

    if (merged_sum < merged_limit_)
    {
      c_avg->merged_ = merged_sum;

    }
    else
    {
      c_avg->merged_ = merged_limit_;
    }
    c_avg->updateAttributes(temp_sym_axis / merge_weight_sum, temp_origin / merge_weight_sum, temp_z_axis / merge_weight_sum);
  }

void
  Cylinder::merge(std::vector<Polygon::Ptr>& poly_vec)
  {
    assignID(poly_vec);
    //if(this->id==0) std::cout << "merge_weight before: " << this->merge_weight_ << "," << merged << std::endl;
    Polygon::Ptr p_average = Polygon::Ptr(new Cylinder);
    computeAverage(poly_vec, p_average);
    mergeUnion(poly_vec, p_average);
    std::cout << "c_merged (r, hmax, hmin, sym_axis): " << r_ << "," << h_max_ << "," << h_min_ << "," <<  sym_axis_ << std::endl;
    for(unsigned int i=0; i<contours_[0].size(); i++)
    {
      std::cout << contours_[0][i](0) << "," << contours_[0][i](1) << std::endl;
    }
    assignWeight();
    //if(this->id==0) std::cout << "merge_weight after: " << this->merge_weight_ << "," << merged  << std::endl;
  }

  /*void
  Cylinder::grabParams(Cylinder& c_src)
  {
    //this->centroid_ = c_src.centroid_;
    this->pose_ = c_src.pose_;
    this->origin_ = c_src.origin_;
    this->d_ = c_src.d_;
    this->r_ = c_src.r_;
    //this->transform_from_world_to_plane = c_src.transform_from_world_to_plane;
    this->normal_ = c_src.normal_;
    this->sym_axis_ = c_src.sym_axis_;
    this->h_max_ = c_src.h_max_;
    this->h_min_ = c_src.h_min_;
    this->merge_weight_ = c_src.merge_weight_;
    this->merged_ = c_src.merged_;
    this->frame_stamp_ = c_src.frame_stamp_;
    this->holes_ = c_src.holes_;
    this->color_ = c_src.color_;
  }*/


  //################## methods to roll and unroll cylinder###############

  /*void Cylinder::getCyl3D(Cylinder& c3d)
  {
    c3d= *this;
    c3d.makeCyl3D();
  }



  void Cylinder::makeCyl3D()
  {
    //Transform to local coordinate system
    Polygon poly_plane;

  for (size_t j = 0; j < contours_.size(); j++) {

    poly_plane.holes_.resize(contours_.size());
    poly_plane.contours_.resize(contours_.size());

    for (size_t k = 0; k < contours_[j].size(); k++) {
      poly_plane.contours_[j].resize(contours_[j].size());

      Eigen::Vector3f point_trans =
          transform_from_world_to_plane
     * contours[j][k];

      poly_plane.contours[j][k] = point_trans;

    }
  }*/

    // transform into cylinder shape via polar coordinates
    /*for (size_t j = 0; j < contours_.size(); j++) {

    holes_.resize(contours_.size());

    for (size_t k = 0; k < contours_[j].size(); k++) {

      Eigen::Vector3f point_temp;
      getPt3D(contours_[j][k],point_temp);
      point_temp = pose_ * point_temp;
      contours_[j][k] = point_temp;
    }
  }*/
  /*}

  void Cylinder::getPt3D(Eigen::Vector3f& pt2d, Eigen::Vector3f& pt3d){

    double alpha = pt2d[0]/ r_ ;
    //         use polar coordinates to create cylinder points
    pt3d<<  r_ * sin(-alpha), pt2d[1],  r_*  cos(-alpha);

  }

  void Cylinder::makeCyl2D()
  {
    /*bool start; // bool to indicate first point of contour
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
  }*/
  /*}

  void
  Cylinder::getCyl2D(Cylinder& c2d)
  {
    c2d = *this;
    c2d.makeCyl2D();
  }*/


  //################## methods for merging############################

        //##############Methods for Debug #################################################

  void
  Cylinder::dbgOut(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,std::string & name){

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
  Cylinder::dumpParams(std::string  name)
  {

    std::string path = "/home/goa-tz/eval/params/";
    path.append(name.c_str());
    std::ofstream os(path.c_str(),std::ofstream::app );

    os<<frame_stamp_<<" "<<r_<<" "<<
        //this->origin_[0]<<" "<<this->origin_[1]<<" "<<this->origin_[2]<<" "<<
        //this->centroid_[0]<<" "<<this->centroid[1]<<" "<<this->centroid[2]<<" "<<
        //this->normal_[0]<<" "<<this->normal_[1]<<" "<<this->normal_[2]<<" "<<
        this->sym_axis_[0]<<" "<<this->sym_axis_[1]<<" "<<this->sym_axis_[2]<<"\n";
    os.close();
  }


  void
  Cylinder::printAttributes(std::string & name)
  {
    ROS_DEBUG_STREAM( "_______"<<name.c_str() <<"______\n");
    //ROS_DEBUG_STREAM("origin = \n"<< this->origin_<<"\n");
    //ROS_DEBUG_STREAM( "centroid = \n"<<this->centroid<<"\n");
    ROS_DEBUG_STREAM("radius = "<< this->r_<<"\n");
    //ROS_DEBUG_STREAM("normal = "<< this->normal_<<"\n");
    ROS_DEBUG_STREAM("sym_axis = "<< this->sym_axis_<<"\n");
    ROS_DEBUG_STREAM("merged = " << this->merged_<<"\n");
    ROS_DEBUG_STREAM("merge_weight = "<<this->merge_weight_<<"\n");
    ROS_DEBUG_STREAM( "_________________\n");
  }


  /*void Cylinder::getArc(const Eigen::Vector3f& goal,const Eigen::Vector3f& start, float& Tx,bool first)
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

      //Eigen::Vector2f  d_ba=b-a;

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
  }*/

  /*void
  Cylinder::compensateOffset(Cylinder::Ptr& c_ref)
  {
    Eigen::Vector3f n12 = c_ref->pose_.inverse().rotation() * c_ref->normal_;
    Eigen::Vector3f s12 = c_ref->pose_.inverse().rotation() * this->sym_axis_;
    Eigen::Vector3f o12 = c_ref->pose_.inverse() * this->origin_;
    o12[1]=0;
    Eigen::Affine3f T12;
    n12.normalize();
    s12.normalize();
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(s12,n12,o12,T12);
    /*for (size_t i = 0; i < this->contours_.size(); ++i)
    {
        for (size_t j = 0; j < this->contours_[i].size(); ++j)
        {
            this->contours_[i][j] = c_ref->pose_ * T12.inverse() * pose_.inverse() * this->contours_[i][j];
        }
    }
    this->pose_ = c_ref->pose_;
  }*/

  /*void
  Cylinder::contoursFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    std::vector<std::vector<Eigen::Vector3f> > contours_eigen;
    std::vector<Eigen::Vector3f> c_eigen;
    for(unsigned int j = 0; j < cloud->size(); j++)
    {
      Eigen::Vector3f pt = cloud->points[j].getVector3fMap();
      c_eigen.push_back(pt);
    }
    contours_eigen.push_back(c_eigen);
    //computePose(contours_eigen);
    setContours3D(contours_eigen);

    std::vector<Eigen::Vector2f> pts;
  pts.resize(cloud->points.size());
  for(unsigned int j=0; j<cloud->points.size(); j++)
  {

    pts[j](0) = cloud->points[j].x;
    pts[j](1) = cloud->points[j].y;
    //pts[j](2) = cloud->points[j].z;
  }
    contours_.push_back(pts);
  }*/


  // Replaced by setContours3D
  /*void
Cylinder::ContoursFromList( std::vector<std::vector<Eigen::Vector3f> >& in_list)
{

  computeAttributes(sym_axis_, normal_, origin_);
  contours_.resize(in_list.size());

  for (size_t j = 0; j < in_list.size(); j++)
  {

    contours_[j].resize(in_list[j].size());
    holes_.resize(in_list[j].size());
    for (size_t k = 0; k < in_list[j].size(); k++)
    {
      contours_[j][k] = in_list[j][k];
    }
  }
}*/


  /*void
  Cylinder::paramsFromShapeMsg()
  {
    if (sym_axis_[2] < 0 )
    {
      sym_axis_ = sym_axis_ *-1;
    }*/

    /*sym_axis_.normalize();
    normal_.normalize();
    this->computeAttributes(sym_axis_, normal_, origin_);*/
  //}

  /*void
  Cylinder::merge(std::vector<Cylinder::Ptr>& c_array)
  {
    ROS_DEBUG_STREAM("START MERGING");
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

    this->mergeUnion(merge_polygons,average_polygon);
    this->assignID(merge_polygons);
    this->GrabParams(*average_cyl);
    this->assignWeight();
    this->makeCyl3D();
  }*/


}//namespace
