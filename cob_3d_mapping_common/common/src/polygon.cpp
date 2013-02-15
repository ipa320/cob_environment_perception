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
* Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 03/2012
*
* \brief
* Class representing polygon shapes
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
//ros includes
#include <ros/console.h>
//cob includes
#include "cob_3d_mapping_common/polygon.h"
//pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#ifdef PCL_VERSION_COMPARE
  #include <pcl/common/transforms.h>
#else
  #include <pcl/common/transform.h>
  #include <pcl/registration/transforms.h>
#endif
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

//boost includes
#include <boost/shared_ptr.hpp>

//custom definitons
#define MOD(a,b) ( ((a%b)+b)%b )

namespace cob_3d_mapping
{
// NON MEMBER FUNCTIONS

/**
* \brief Get point on polygon.
*
* Point on polygon is calculated with distance d in normal direction.
* \param normal Normal of the polygon
* \param d Parameter from plane equation ax+by+cz=d
*/
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

/**
* \brief Get axes of coordinate system on plane.
*
* Calculation of axes cartesian coordinate system using one given axis.
* \param normal Axis coordinate system is oriented to.
* \param v Axis orthogonal to normal
* \param u Axis completing the Gaussian three-leg
*/
void
getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
    Eigen::Vector3f &u, Eigen::Vector3f &v)
{
  v = normal.unitOrthogonal ();
  u = normal.cross (v);
}


/**
* \brief Copy GPC structure
*/
void
copyGpcStructure(const gpc_polygon* source, gpc_polygon* dest)
{
  dest->num_contours = source->num_contours;
  dest->hole = (int*)malloc(source->num_contours*sizeof(int));
  dest->contour = (gpc_vertex_list*)malloc(source->num_contours*sizeof(gpc_vertex_list));
  for(int j=0; j<source->num_contours; ++j)
  {
    dest->hole[j] = source->hole[j];
    dest->contour[j].num_vertices = source->contour[j].num_vertices;
    dest->contour[j].vertex = (gpc_vertex*)malloc(source->contour[j].num_vertices*sizeof(gpc_vertex));

    for(int k=0; k<source->contour[j].num_vertices; ++k)
    {
      dest->contour[j].vertex[k].x = source->contour[j].vertex[k].x;
      dest->contour[j].vertex[k].y = source->contour[j].vertex[k].y;
    }
  }
}

/**
* \brief Smooth contours of GPC structure
*
* Outline of GPC structure is smoothed using a path smoothing algorithm.
* \param gpc_in Input GPC structure
* \param gpc_out Output GPC structure
*/
void
smoothGpcStructure(const gpc_polygon* gpc_in, gpc_polygon* gpc_out)
{
  copyGpcStructure(gpc_in, gpc_out);
  float weight_data = 0.5, weight_smooth = 0.01, tolerance = 1.0f;
  float change = tolerance;

  for(int j=0; j<gpc_in->num_contours; ++j)
  {
    int num_iteration=0;
    int l = gpc_in->contour[j].num_vertices; // length
    gpc_vertex* pn = gpc_out->contour[j].vertex; // new polygon
    gpc_vertex* po = gpc_in->contour[j].vertex; // old polygon

    while(change >= tolerance && num_iteration<=5)
    {
      change = 0.0f;
      for(int k=0; k<l; ++k)
      {
        //if( !k%10 ) continue;
        // do x value:
        float before = pn[k].x;
        pn[k].x += weight_data * (po[k].x - pn[k].x);
        pn[k].x += weight_smooth * (pn[ MOD(k-1,l) ].x + pn[ (k+1)%l ].x - (2.0f * pn[k].x));
        pn[k].x += 0.5f * weight_smooth * (2.0f * pn[ MOD(k-1,l) ].x - pn[ MOD(k-2,l) ].x - pn[k].x);
        pn[k].x += 0.5f * weight_smooth * (2.0f * pn[    (k+1)%l ].x - pn[    (k+2)%l ].x - pn[k].x);
        change += fabs(before - pn[k].x);

        // do y value:
        before = pn[k].y;
        pn[k].y += weight_data * (po[k].y - pn[k].y);
        pn[k].y += weight_smooth * (pn[ MOD(k-1,l) ].y + pn[ (k+1)%l ].y - (2.0f * pn[k].y));
        pn[k].y += 0.5f * weight_smooth * (2.0f * pn[ MOD(k-1,l) ].y - pn[ MOD(k-2,l) ].y - pn[k].y);
        pn[k].y += 0.5f * weight_smooth * (2.0f * pn[    (k+1)%l ].y - pn[    (k+2)%l ].y - pn[k].y);
        change += fabs(before - pn[k].y);
      }
      ++num_iteration;
    }
    ROS_DEBUG_STREAM( "Needed " << num_iteration << " iterations for polygon of size " << l );
  }
}


//##########methods for instantiation##############
void
Polygon::computeAttributes(const Eigen::Vector3f &new_normal, const Eigen::Vector4f& new_centroid)
{
  this->d = new_centroid.head(3).dot(new_normal);
  if (this->d > 0) { this->normal = -new_normal; d = -d; }
  else { this->normal = new_normal; }
  centroid = new_centroid;

  pcl::getTransformationFromTwoUnitVectorsAndOrigin(
      this->normal.unitOrthogonal(),this->normal,this->centroid.head(3),this->transform_from_world_to_plane);
}

void Polygon::transform2tf(const Eigen::Affine3f& trafo)
{
  //transform contours
  this->TransformContours(trafo);

  //  transform parameters
  Eigen::Vector3f tf_normal = trafo.rotation() *this->normal;
  this->normal =tf_normal;
  Eigen::Vector3f tf_centroid3f = this->centroid.head(3);
  tf_centroid3f = trafo * tf_centroid3f;
  this->centroid.head(3) = tf_centroid3f;
  this->computeAttributes(this->normal,this->centroid);
}


void
Polygon::smoothPolygon()
{
  gpc_polygon *before, *after;
  this->getGpcStructure(this->transform_from_world_to_plane, before);
  smoothGpcStructure(before,after);
  this->applyGpcStructure(this->transform_from_world_to_plane.inverse(), after);
  gpc_free_polygon(before);
  gpc_free_polygon(after);
}


//###########methods for merging##################


void
Polygon::getMergeCandidates(const std::vector<Polygon::Ptr>& poly_vec, std::vector<int>& intersections) const
{
  for(size_t i=0; i< poly_vec.size(); ++i)
  {
    if(this->hasSimilarParametersWith(poly_vec[i]) && this->isIntersectedWith(poly_vec[i])) intersections.push_back(i);
  }
}

bool
Polygon::isIntersectedWith(const Polygon::Ptr& poly) const
{
  gpc_polygon *gpc_res = new gpc_polygon();
  this->getIntersection(poly,gpc_res);
  bool res = (gpc_res->num_contours != 0);
  gpc_free_polygon(gpc_res);
  return (res);
}


void
Polygon::getIntersection(const Polygon::Ptr& poly, gpc_polygon* gpc_intersection) const
{
  gpc_polygon *gpc_poly = new gpc_polygon(), *gpc_here = new gpc_polygon();

  this->getGpcStructure(poly->transform_from_world_to_plane, gpc_here);
  poly->getGpcStructure(poly->transform_from_world_to_plane, gpc_poly);
  gpc_polygon_clip(GPC_INT, gpc_here, gpc_poly, gpc_intersection);
  gpc_free_polygon(gpc_poly);
  gpc_free_polygon(gpc_here);
}

bool
Polygon::getContourOverlap(const Polygon::Ptr& poly, float& rel_overlap, int& abs_overlap) const
{
  gpc_polygon *gpc_a = new gpc_polygon(), *gpc_b = new gpc_polygon();
  gpc_polygon *gpc_res_int = new gpc_polygon(), *gpc_res_union = new gpc_polygon();
  this->getGpcStructure(poly->transform_from_world_to_plane, gpc_a);
  poly->getGpcStructure(poly->transform_from_world_to_plane, gpc_b);
  gpc_polygon_clip(GPC_INT, gpc_a, gpc_b, gpc_res_int);
  gpc_polygon_clip(GPC_UNION, gpc_a, gpc_b, gpc_res_union);

  int i_int = -1, i_union = -1;
  for(size_t i=0;i<gpc_res_int->num_contours;++i) { if(!gpc_res_int->hole[i]) { i_int = i; break; } }
  for(size_t i=0;i<gpc_res_union->num_contours;++i) { if(!gpc_res_union->hole[i]) { i_union = i; break; } }
  if(i_int == -1 || i_union == -1) return false;
  int overlap = 0;
  float d_th = pow( 0.01, 2 );
  for(size_t i=0;i<gpc_res_int->contour[i_int].num_vertices; ++i)
  {
    gpc_vertex* pv_int = &gpc_res_int->contour[i_int].vertex[i];
    for(size_t j=0;j<gpc_res_union->contour[i_union].num_vertices; ++j)
    {
      if( pow(gpc_res_union->contour[i_union].vertex[j].x - pv_int->x, 2) +
          pow(gpc_res_union->contour[i_union].vertex[j].y - pv_int->y, 2) < d_th )
      {
        ++overlap;
        break;
      }
    }
  }
  rel_overlap = (float)overlap/(float)gpc_res_int->contour[i_int].num_vertices;
  abs_overlap = overlap;
  ROS_DEBUG_STREAM("Overlap: " << overlap << "/"<<gpc_res_int->contour[i_int].num_vertices << " -> "
            << (float)overlap/(float)gpc_res_int->contour[i_int].num_vertices );
  gpc_free_polygon(gpc_a);
  gpc_free_polygon(gpc_b);
  gpc_free_polygon(gpc_res_int);
  gpc_free_polygon(gpc_res_union);
  return true;
}


float
Polygon::computeSimilarity(const Polygon::Ptr& poly) const
{
  float normal = (fabs(poly->normal.dot(this->normal)) - this->merge_settings_.angle_thresh) /
    (1-this->merge_settings_.angle_thresh);
  float d = fabs(fabs((this->centroid-poly->centroid).head(3).dot(this->normal))-this->merge_settings_.d_thresh) /
    this->merge_settings_.d_thresh;
  float overlap = 0.0;
  int abs_overlap;
  this->getContourOverlap(poly, overlap, abs_overlap);
  return ( 3.0f / (1.0f / normal + 1.0f / d + 1.0f / overlap) );
}

void
Polygon::merge(std::vector<Polygon::Ptr>& poly_vec)
{
  Polygon::Ptr p_average= Polygon::Ptr(new Polygon);
  this->applyWeighting(poly_vec,p_average);
  this->merge_union(poly_vec,p_average);
  this->assignWeight();
  this->assignID(poly_vec);
}

void
Polygon::merge_union(std::vector<Polygon::Ptr>& poly_vec,  Polygon::Ptr& p_average)
{
  gpc_polygon *gpc_C = new gpc_polygon, *smoothed = new gpc_polygon;
  this->getGpcStructure(p_average->transform_from_world_to_plane, gpc_C);

  for(size_t i=0;i<poly_vec.size();++i)
  {
    gpc_polygon *gpc_B = new gpc_polygon;
    poly_vec[i]->getGpcStructure(p_average->transform_from_world_to_plane, gpc_B);
    gpc_polygon_clip(GPC_UNION, gpc_B, gpc_C, gpc_C);
    gpc_free_polygon(gpc_B);
    d_color_.addColor(poly_vec[i]->color[0]*255,poly_vec[i]->color[1]*255,poly_vec[i]->color[2]*255);
    //std::cout << "\t" << poly_vec[i]->color[0] << "," << poly_vec[i]->color[1] << "," << poly_vec[i]->color[2] << std::endl;
  }
  d_color_.addColor(this->color[0]*255,this->color[1]*255,this->color[2]*255);

  // fill in parameters for "this" polygon
  this->transform_from_world_to_plane = p_average->transform_from_world_to_plane;
  this->d = p_average->d;
  this->normal = p_average->normal;
  this->centroid = p_average->centroid;
  if(this->merged<9) { this->merged = p_average->merged; }
  else { this->merged = 9; }
  this->merge_weight_ = p_average->merge_weight_;
  copyGpcStructure(gpc_C, smoothed);
  smoothGpcStructure(gpc_C, smoothed);
  this->applyGpcStructure(p_average->transform_from_world_to_plane.inverse(), smoothed);
  gpc_free_polygon(gpc_C);
  gpc_free_polygon(smoothed);
  uint8_t r,g,b;
  d_color_.getColor(r,g,b);
  //std::cout << "\t" << (int)r << "," << (int)g << "," << (int)b << std::endl;
  float temp_inv = 1.0f/255.0f;
  //std::cout << r* temp_inv << "," << g* temp_inv << "," << b* temp_inv << std::endl;
  this->color[0] = r * temp_inv;
  this->color[1] = g * temp_inv;
  this->color[2] = b * temp_inv;
  //d_color_.getColor(this->color[0],this->color[1],this->color[2]);
}



void
Polygon::assignWeight()
{
  if (std::strcmp(merge_settings_.weighting_method.c_str(), "COUNTER")== 0)
  {
    merge_weight_=merged;
  }
  else if (std::strcmp(merge_settings_.weighting_method.c_str(), "AREA")== 0)
  {
    double area = computeArea3d();
    merge_weight_=(merged*area);
  }
}

void
Polygon::assignID(const std::vector<Polygon::Ptr>& poly_vec)
{
   unsigned int tmp_id=poly_vec[0]->id;
   for(size_t i=0;i<poly_vec.size();++i)
   {
        if(poly_vec[i]->id<tmp_id)tmp_id=poly_vec[i]->id;
   }
   this->id=tmp_id;
}

void
Polygon::applyWeighting(const std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr & p_average)
{
  Eigen::Vector3f average_normal=normal*merge_weight_;
  Eigen::Vector4f average_centroid=centroid*merge_weight_;
  double average_d=d*merge_weight_;
  double sum_w=merge_weight_;
  int sum_merged=merged;

  for(int i=0 ; i< (int) poly_vec.size();i++)
  {
    Polygon& p_map1 =*(poly_vec[i]);

    if(normal.dot(p_map1.normal)<0)
    {
      average_normal += p_map1.merge_weight_* -p_map1.normal;
    }
    else
    {
      average_normal += p_map1.merge_weight_* p_map1.normal;
    }
    average_centroid += p_map1.merge_weight_* p_map1.centroid;
    average_d +=p_map1.merge_weight_ * p_map1.d;
    sum_w += p_map1.merge_weight_;
    sum_merged += p_map1.merged;
  }

  average_normal=average_normal/sum_w;
  average_centroid=average_centroid/sum_w;
  average_d=average_d/sum_w;
  average_normal.normalize();

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
Polygon::getGpcStructure(const Eigen::Affine3f& external_trafo, gpc_polygon* gpc_p) const
{
  // get transformed contours
  std::vector< std::vector <Eigen::Vector3f> > transformed_contours;
  this->getTransformedContours(external_trafo, transformed_contours);

  gpc_p->num_contours = contours.size();
  gpc_p->hole = (int*)malloc(contours.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(contours.size()*sizeof(gpc_vertex_list));
  for(size_t j=0; j<contours.size(); ++j)
  {
    gpc_p->hole[j] = holes[j];
    gpc_p->contour[j].num_vertices = contours[j].size();
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));

    for(size_t k=0; k<contours[j].size(); ++k)
    {
      Eigen::Vector3f point_trans = transformed_contours[j][k];
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);
    }
  }
}


void
Polygon::applyGpcStructure(const Eigen::Affine3f& external_trafo, const gpc_polygon* gpc_p)
{
  // clear contours, at the end gpc_C contains everything we need!
  this->contours.clear();
  this->holes.clear();
  for(int j=0; j<gpc_p->num_contours; ++j)
  {
    this->contours.push_back(std::vector<Eigen::Vector3f>());
    this->holes.push_back(gpc_p->hole[j]);
    float last_x = 0.0, last_y=0.0;
    for (int k=0; k<gpc_p->contour[j].num_vertices; ++k)
    {
      // check if points are too close to each other (so remove similar points)
      if (fabs(gpc_p->contour[j].vertex[k].x - last_x) < 0.01 && fabs(gpc_p->contour[j].vertex[k].y - last_y) < 0.01)
        continue;
      last_x = gpc_p->contour[j].vertex[k].x;
      last_y = gpc_p->contour[j].vertex[k].y;
      this->contours.back().push_back(external_trafo * Eigen::Vector3f(last_x,last_y,0));
    }

    if (this->contours.back().size() <= 2)  // drop empty contour lists
    {
      this->contours.pop_back();
      this->holes.pop_back();
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
Polygon::computeArea() const
{
  // IMPORTANT: computes area only for the projection of the polygon onto XY plane
  // therefore  the resulting area is not the true area of the polygon
  double xi, xi_1, yi, yi_1, area=0;
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
  }
  return area;
}



double
Polygon::computeArea3d() const
{
  double area=0;
  for (size_t i=0; i<contours.size(); ++i)
  {
    Eigen::Vector3f vec_sum(Eigen::Vector3f::Zero());

    for (int j=0; j<(int)contours[i].size()-1; ++j)
      vec_sum += contours[i][j].cross(contours[i][j+1]);

    vec_sum += contours[ i ][ contours[i].size()-1 ].cross(contours[i][0]);
    area += 0.5 * normal.dot(vec_sum);

    // special cases for holes can be ignored, since their vertex orientation is opposite to the
    // outer most contour. Therefore the their area sign are different.

    //if(holes[i]) area -= 0.5d * fabs(normal.dot(temp_vec));
    //else area += 0.5d * fabs(normal.dot(temp_vec));
  }
  return std::fabs(area);
}

void
Polygon::getTransformationFromPlaneToWorld(
  const Eigen::Vector3f &normal,
  const Eigen::Vector3f &origin,
  Eigen::Affine3f &transformation) const
{
  Eigen::Vector3f u, v;
  getCoordinateSystemOnPlane(normal, u, v);
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal, origin, transformation);
  transformation = transformation.inverse();
}

void
Polygon::getTransformationFromPlaneToWorld(
  const Eigen::Vector3f z_axis,
  const Eigen::Vector3f &normal,
  const Eigen::Vector3f &origin,
  Eigen::Affine3f &transformation)
{
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
      contours[j][k] = trafo * contours[j][k];
    }
  }
}

void
Polygon::getTransformedContours(const Eigen::Affine3f& trafo, std::vector< std::vector<Eigen::Vector3f> >& new_contours) const
{
  new_contours.resize(contours.size());
  for(size_t j=0; j<contours.size(); j++)
  {
    new_contours[j].resize(contours[j].size());
    for(size_t k=0; k<contours[j].size(); k++)
    {
      new_contours[j][k] = trafo*contours[j][k];
    }
  }
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
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(eigen_vectors.col(1),eigen_vectors.col(0),centroid.head(3),pose);

  pcl::PointCloud<pcl::PointXYZ> cloud_trans;
  pcl::transformPointCloud(poly_cloud, cloud_trans, pose);
  pcl::getMinMax3D(cloud_trans, min_pt, max_pt);
}

//#############debugging methods#######################

void Polygon::debug_output(std::string name)
{
  std::string path = "/home/goa-tz/debug/dbg/";
  path.append(name.c_str());
  std::ofstream os(path.c_str());

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
