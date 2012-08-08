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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
 * ToDo:
 *e
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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

#include "cob_3d_mapping_common/polygon.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#ifdef PCL_VERSION_COMPARE
#include <pcl/common/transforms.h>
#else
#include <pcl/common/common.h>
#include <pcl/common/transform.h>
#include <pcl/registration/transforms.h>
#endif
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <boost/shared_ptr.hpp>


namespace cob_3d_mapping
{
// NON MEMBER FUNCTIONS
void
getPointOnPolygon(const Eigen::Vector3f &normal,double d,Eigen::Vector3f &point)
{
  float value=fabs(normal(0));
  int direction=0;

  if(fabs(normal(1))>value)
  {

    direction=1;
    value=fabs(normal(1));
  }

  if(fabs(normal(2))>value)

  {
    direction=2;
    value=fabs(normal(2));
  }
  point << 0,0,0;
  point(direction)=-d/normal(direction);
}

void
getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
    Eigen::Vector3f &u, Eigen::Vector3f &v)
{
  v = normal.unitOrthogonal ();
  u = normal.cross (v);
}


//##########methods for instantiation##############

void
Polygon::computeAttributes(const Eigen::Vector3f &new_normal, const Eigen::Vector4f& new_centroid)
{


//  TODO: recomputation of centroid
  normal=new_normal;
  centroid = new_centroid;
  d=fabs(centroid.head(3).dot(normal));
//  d=centroid.norm();

  pcl::getTransformationFromTwoUnitVectorsAndOrigin(
        this->normal.unitOrthogonal(),this->normal,this->centroid.head(3),this->transform_from_world_to_plane);
}



void Polygon::transform2tf(const Eigen::Affine3f& trafo)
{
  //transform contours
    this->TransformContours(trafo);

    //transform parameters
    //  transform parameters
    Eigen::Vector3f tf_normal = trafo.rotation() *this->normal;
    this->normal =tf_normal;
    Eigen::Vector3f tf_centroid3f = this->centroid.head(3);
    tf_centroid3f = trafo * tf_centroid3f;
    this->centroid.head(3) = tf_centroid3f;
    this->computeAttributes(this->normal,this->centroid);
}


//###########methods for merging##################

void
Polygon::isMergeCandidate(std::vector<PolygonPtr>& poly_vec,merge_config& config,std::vector<int>& intersections)
{


  for(size_t i=0; i< poly_vec.size(); ++i)
  {
    Polygon& p_map = *(poly_vec[i]);

    Eigen::Vector4f temp=this->centroid-p_map.centroid;
    Eigen::Vector3f connection;
    connection << temp[0], temp[1] , temp[2];


    if(fabs(p_map.normal.dot(this->normal)) > (merge_settings_.angle_thresh) &&
        fabs(connection.dot(this->normal)) < merge_settings_.d_thresh)

    {



      bool is_intersected= this->isMergeCandidate_intersect(p_map);
      if(is_intersected == true)
      {

        intersections.push_back(i);
        continue;
      }
      if (is_intersected == false)
      {
        //          std::cout<<" criteria fulfilled, but no intersection\n";
      }

      Eigen::Vector4f temp=this->centroid-p_map.centroid;
      Eigen::Vector3f connection;
    }

  }
}


bool
Polygon::isMergeCandidate_intersect(Polygon& p_map)
{
  bool is_intersected;
  gpc_polygon gpc_result;
  gpc_polygon gpc_p_merge;
  gpc_polygon gpc_p_map;


  this->GpcStructureUsingMap(p_map.transform_from_world_to_plane, &gpc_p_merge);
  p_map.GpcStructureUsingMap(p_map.transform_from_world_to_plane, &gpc_p_map);

  gpc_polygon_clip(GPC_INT,&gpc_p_merge,&gpc_p_map,&gpc_result);

  if(gpc_result.num_contours == 0)
  {
    is_intersected=false;
  }
  else
  {
    is_intersected=true;
  }

  return is_intersected;
}

void
Polygon::merge(std::vector<PolygonPtr>& poly_vec)
{
  PolygonPtr p_average= PolygonPtr(new Polygon);
  this->applyWeighting(poly_vec,p_average);
  this->merge_union(poly_vec,p_average);
  this->assignWeight();
}

void
Polygon::merge_union(std::vector<PolygonPtr>& poly_vec,  PolygonPtr& p_average)
{
  gpc_polygon gpc_C, gpc_B;

  this->GpcStructureUsingMap(p_average->transform_from_world_to_plane, &gpc_C);

  for(size_t i=0;i<poly_vec.size();++i)
  {
    //std::cout << poly_vec[i]->contours.size() << " " << std::endl;
    poly_vec[i]->GpcStructureUsingMap(p_average->transform_from_world_to_plane,&gpc_B);

    gpc_polygon_clip(GPC_UNION, &gpc_B, &gpc_C, &gpc_C);
  }

  // fill in parameters for "this" polygon
  this->transform_from_world_to_plane = p_average->transform_from_world_to_plane;
  this->d = p_average->d;
  this->normal = p_average->normal;
  this->centroid = p_average->centroid;
  if(this->merged<9) { this->merged = p_average->merged; }
  else { this->merged = 9; }
  this->merge_weight_ = p_average->merge_weight_;

  // clear contours, at the end gpc_C contains everything we need!
  this->contours.clear();
  this->holes.clear();
  for(int j=0; j<gpc_C.num_contours; ++j)
  {
    this->contours.push_back(std::vector<Eigen::Vector3f>());
    this->holes.push_back(gpc_C.hole[j]);
    float last_x = 0.0, last_y=0.0;
    for (int k=0; k<gpc_C.contour[j].num_vertices; ++k)
    {
      // check if points are too close to each other (so remove similar points)
      if (fabs(gpc_C.contour[j].vertex[k].x - last_x) < 0.0001 && fabs(gpc_C.contour[j].vertex[k].y - last_y) < 0.0001)
        continue;
      last_x = gpc_C.contour[j].vertex[k].x;
      last_y = gpc_C.contour[j].vertex[k].y;
      this->contours.back().push_back(p_average->transform_from_world_to_plane.inverse() * Eigen::Vector3f(last_x,last_y,0));
    }

    if (this->contours.back().size() <= 2)  // drop empty contour lists
    {
      std::cout << "Drop! New size: " << this->contours.size() - 1 << std::endl;
      this->contours.pop_back();
      this->holes.pop_back();
    }
  }
  if (this->contours.size() == 0) std::cout << "!!!! NO CONTOURS ANYMORE" << std::endl;
}

void
Polygon::assignWeight()
{
  if (std::strcmp(merge_settings_.weighting_method.c_str(), "COUNTER")== 0)
  {
    //USE
    merge_weight_=merged;
  }
  else if (std::strcmp(merge_settings_.weighting_method.c_str(), "AREA")== 0)
  {
    //DO NOT USE
    //THIS IS WORK IN PROGRESS
    merge_weight_ = computeArea3d();
  }
  else if (std::strcmp(merge_settings_.weighting_method.c_str(), "COMBINED")== 0)
  {
    //USE
    merge_weight_ = merged + sqrt(computeArea3d());
  }
  else if (std::strcmp(merge_settings_.weighting_method.c_str(), "DIST")== 0)
  {
    // Do not use d has to be substituted with value for distance to sensor , not distance in global
    // coordinate system
    float dist_factor=d*d;

    if (fabs(d)<0.5) {
      dist_factor=0.5;
    }
    dist_factor *= 0.5;
    dist_factor=1/dist_factor;
    merge_weight_ =merged + dist_factor;

  }
}


void
Polygon::applyWeighting(const std::vector<PolygonPtr>& poly_vec, PolygonPtr & p_average)
{

  //std::cout<<"MERGE WEIGHT: "<<merge_weight_<<std::endl;
  Eigen::Vector3f average_normal=normal*merge_weight_;
  Eigen::Vector4f average_centroid=centroid*merge_weight_;
  double average_d=d*merge_weight_;
  double sum_w=merge_weight_;
  int sum_merged=merged;

  for(int i=0 ; i< (int) poly_vec.size();i++)
  {
    Polygon& p_map1 =*(poly_vec[i]);

    if(normal.dot(p_map1.normal)<0){
      //if (p.normal.dot(p_map.normal)<-0.95){
      p_map1.normal=-p_map1.normal;
      // p_map1.d=-p_map1.d;
    }

    average_normal += p_map1.merge_weight_* p_map1.normal;
    average_centroid += p_map1.merge_weight_* p_map1.centroid;
    average_d +=p_map1.merge_weight_ * p_map1.d;
    sum_w += p_map1.merge_weight_;

    sum_merged += p_map1.merged;


  }

  average_normal=average_normal/sum_w;
  average_centroid=average_centroid/sum_w;
  average_d=average_d/sum_w;
  average_normal.normalize();
//  average_d /= average_normal.norm();

  if (sum_merged < 9)
  {
    p_average->merged=sum_merged;
  }
  else
  {
    p_average->merged=9;
  }


  p_average->computeAttributes(average_normal,average_centroid);



}


void
Polygon::GpcStructureUsingMap(const Eigen::Affine3f& external_trafo, gpc_polygon* gpc_p)
{
  // get transformed contours
  std::vector< std::vector <Eigen::Vector3f> > transformed_contours;
  transformed_contours = this->getTransformedContours(external_trafo);

  gpc_p->num_contours = contours.size();
  gpc_p->hole = (int*)malloc(contours.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(contours.size()*sizeof(gpc_vertex_list));
  //std::cout << "num_contours: " << gpc_p->num_contours << std::endl;
  for(size_t j=0; j<contours.size(); j++)
  {
    //std::cout << j << std::endl;
    gpc_p->hole[j] = holes[j];
    gpc_p->contour[j].num_vertices = contours[j].size();
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));

    for(size_t k=0; k<contours[j].size(); k++)
    {
      Eigen::Vector3f point_trans = transformed_contours[j][k];
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);

    }
  }
}

void
Polygon::GpcStructure( gpc_polygon* gpc_p)
{
  // get transformed contours
  std::vector< std::vector <Eigen::Vector3f> > transformed_contours;
  transformed_contours= getTransformedContours(transform_from_world_to_plane);
  //printMapEntry(p);
  gpc_p->num_contours = contours.size();
  gpc_p->hole = (int*)malloc(contours.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(contours.size()*sizeof(gpc_vertex_list));
  for(size_t j=0; j<contours.size(); j++)
  {
    //std::cout << j << std::endl;
    gpc_p->contour[j].num_vertices = contours[j].size();
    gpc_p->hole[j] = 0;
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));
    for(size_t k=0; k<contours[j].size(); k++)
    {
      Eigen::Vector3f point_trans = transformed_contours[j][k];
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);

    }
  }
}


//#######methods for calculation#####################


void
Polygon::computeCentroid()
{
  //find largest non-hole contour
  unsigned int idx = 0;
  for (unsigned int i = 0; i < contours.size (); i++)
  {
    int max_pts = 0;
    if(!holes[i])
    {
      if((int)contours[i].size()>(int)max_pts)
      {
        max_pts = contours[i].size();
        idx = i;
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ> poly_cloud;
  for (unsigned int j = 0; j < contours[idx].size () ; j++)
  {
    pcl::PointXYZ p;
    p.x = contours[idx][j][0];
    p.y = contours[idx][j][1];
    p.z = contours[idx][j][2];
    poly_cloud.push_back(p);
  }
  pcl::compute3DCentroid(poly_cloud,centroid);
}


double
Polygon::computeArea()
{
  double xi, xi_1, yi, yi_1, area=0;

  //area_. (poly_ptr_->contours.size ());
  double sum;
  for (unsigned int i = 0; i < contours.size (); i++)
  {
    if(holes[i]) continue;
    sum = 0;
    //area_[i] = 0;
    for (unsigned int j = 0; j < contours[i].size (); j++)
    {
      xi = contours[i][j][0];
      yi = contours[i][j][1];
      if (j == (contours[i].size ()) - 1)
      {
        xi_1 = contours[i][0][0];
        yi_1 = contours[i][0][1];
      }
      else
      {
        xi_1 = contours[i][j + 1][0];
        yi_1 = contours[i][j + 1][1];
      }
      sum = sum + (xi * yi_1 - xi_1 * yi);

    }
    area += fabs (sum / 2);
    //std::cout << "\n\t*** Area of polygon ( " << i << " ) = " << area_[i] << std::endl;
  }
  return area;
}

double
Polygon::computeArea3d()
{
  Eigen::Vector3f vi, vi_1;
  double area=0;

  //area_. (poly_ptr_->contours.size ());
  for (unsigned int i = 0; i < contours.size (); i++)
  {
    if(holes[i]) continue;
    Eigen::Vector3f temp_vec;
    temp_vec << 0, 0, 0;

    //area_[i] = 0;
    for (unsigned int j = 0; j < contours[i].size (); j++)
    {
      vi << contours[i][j][0],contours[i][j][1],contours[i][j][2];

      if (j == (contours[i].size ()) - 1)
      {
        vi_1 << contours[i][0][0],contours[i][0][1],contours[i][0][2];
      }
      else
      {
        vi_1 << contours[i][j + 1][0],contours[i][j + 1][1],contours[i][j+1][2];
      }

      temp_vec = temp_vec + (vi.cross(vi_1));
    }
    area += normal.dot(temp_vec) /2;
    //std::cout << "\n\t*** Area of polygon ( " << i << " ) = " << area_[i] << std::endl;
  }
  return std::fabs(area);
}
void
Polygon::getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
{
  Eigen::Vector3f u, v;
  getCoordinateSystemOnPlane(normal, u, v);
  // std::cout << "u " << u << std::endl << " v " << v << std::endl;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal, origin, transformation);
  transformation = transformation.inverse();
}

void
Polygon::getTransformationFromPlaneToWorld(const Eigen::Vector3f z_axis,const Eigen::Vector3f &normal,const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
{
  // Eigen::Vector3f u, v;
  // getCoordinateSystemOnPlane(normal, u, v);
  // std::cout << "u " << u << std::endl << " v " << v << std::endl;
  this->normal = normal;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(z_axis, normal, origin, transformation);
  transformation = transformation.inverse();
}

void
Polygon::TransformContours(const Eigen::Affine3f& trafo)
{

  for(size_t j=0; j<contours.size(); j++)
  {
    for(size_t k=0; k<contours[j].size(); k++)
    {
      // std::cout<<trafo.matrix()<<std::endl;
      contours[j][k] = trafo * contours[j][k];
    }
  }
}


std::vector< std::vector<Eigen::Vector3f> >
Polygon::getTransformedContours(const Eigen::Affine3f& trafo)
{
  std::vector<std::vector<Eigen::Vector3f> > t_contours;
  t_contours.resize(contours.size());
  for(size_t j=0; j<contours.size(); j++)
  {
    t_contours[j].resize(contours[j].size());
    for(size_t k=0; k<contours[j].size(); k++)
    {
      // std::cout<<trafo.matrix()<<std::endl;
      t_contours[j][k] = trafo*contours[j][k];
    }
  }

  return t_contours;
}

void
Polygon::computePoseAndBoundingBox(Eigen::Affine3f& pose, Eigen::Vector4f& min_pt, Eigen::Vector4f& max_pt)
{
  pcl::PointCloud<pcl::PointXYZ> poly_cloud;
  unsigned int idx = 0;
  for (unsigned int j = 0; j < contours[idx].size () ; j++)
  {
    pcl::PointXYZ p;
    p.x = contours[idx][j][0];
    p.y = contours[idx][j][1];
    p.z = contours[idx][j][2];
    poly_cloud.push_back(p);
  }
  Eigen::Matrix3f cov;
  pcl::computeCovarianceMatrix (poly_cloud, centroid, cov);
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (cov, eigen_vectors, eigen_values);
  //Eigen::Affine3f pose;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(eigen_vectors.col(1),eigen_vectors.col(0),centroid.head(3),pose);

  pcl::PointCloud<pcl::PointXYZ> cloud_trans;
  pcl::transformPointCloud(poly_cloud, cloud_trans, pose);
  //Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(cloud_trans, min_pt, max_pt);
}

//#############debugging methods#######################

void Polygon::debug_output(std::string name)
{
  std::ofstream os;
  std::string path = "/home/goa-tz/debug/";
  path.append(name.c_str());
  os.open(path.c_str());
  //std::cout<< "name~~~~~~~~~~~~"<<std::endl;
  std::cout<<"saving polygon nodes to "<<path.c_str()<<std::endl;

  for (int i = 0; i < (int) this->contours.size(); ++i)
  {
    for (int j = 0; j < (int) this->contours[i].size(); ++j)
    {
      os << contours[i][j]<<std::endl;
    }
  }

  os.close();
}

}//namespace
