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
#include <cob_3d_mapping_common/polygon.h>
//pcl includes
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
  /*void
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
  }*/

  /**
   * \brief Get axes of coordinate system on plane.
   *
   * Calculation of axes cartesian coordinate system using one given axis.
   * \param normal Axis coordinate system is oriented to.
   * \param v Axis orthogonal to normal
   * \param u Axis completing the Gaussian three-leg
   */
  /*void
getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
    Eigen::Vector3f &u, Eigen::Vector3f &v)
{
  v = normal.unitOrthogonal ();
  u = normal.cross (v);
}*/


  /**
   * \brief Copy GPC structure
   */
  /*void
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
  }*/

  /**
   * \brief Smooth contours of GPC structure
   *
   * Outline of GPC structure is smoothed using a path smoothing algorithm.
   * \param gpc_in Input GPC structure
   * \param gpc_out Output GPC structure
   */
  /*void
  smoothGpcStructure(const gpc_polygon* gpc_in, gpc_polygon* gpc_out)
  {
  }*/


  //##########methods for instantiation##############
  /*Polygon::Polygon(Polygon::Ptr polygon)
  {
    this->normal = polygon->normal;
    this->d = polygon->d;
    this->transform_from_world_to_plane = polygon->transform_from_world_to_plane;
    this->merge_weight_ = polygon->merge_weight_;
  }*/

  Polygon::Polygon(unsigned int id,
                   Eigen::Vector3f normal,
                   Eigen::Vector3f centroid,
                   std::vector<std::vector<Eigen::Vector3f> >& contours_3d,
                   std::vector<bool> holes,
                   std::vector<float> color)
  : normal_(Eigen::Vector3f::Zero()),
    d_(0.0),
    merge_weight_(1.0)
  {
    id_ = id;
    d_ = centroid.dot(normal);
    if (d_ > 0) {
      normal_ = -normal;
      d_ = -d_;
    }
    else { normal_ = normal; }
    holes_ = holes;
    color_ = color;
    computePose(contours_3d);
    setContours3D(contours_3d);
  }

  Polygon::Polygon(unsigned int id,
                   Eigen::Vector3f normal,
                   Eigen::Vector3f centroid,
                   std::vector<pcl::PointCloud<pcl::PointXYZ> >& contours_3d,
                   std::vector<bool> holes,
                   std::vector<float> color)
  : normal_(Eigen::Vector3f::Zero()),
    d_(0.0),
    merge_weight_(1.0)
  {
    id_ = id;
    d_ = centroid.dot(normal);
    if (d_ > 0) {
      normal_ = -normal;
      d_ = -d_;
    }
    else { normal_ = normal; }
    //normal_ = normal;
    holes_ = holes;
    color_ = color;
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
    computePose(contours_eigen);
    setContours3D(contours_eigen);
  }

  void
  Polygon::setContours3D(std::vector<std::vector<Eigen::Vector3f> >& contours_3d)
  {
    contours_.clear();
    for(unsigned int i = 0; i < contours_3d.size(); i++)
    {
      std::vector<Eigen::Vector2f> c;
      for(unsigned int j = 0; j < contours_3d[i].size(); j++)
      {
        Eigen::Vector2f pt_2d = (pose_.inverse()*contours_3d[i][j]).head(2);
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
  Polygon::getContours3D() const
  {
    std::vector<std::vector<Eigen::Vector3f> > contours_3d;
    for(unsigned int i = 0; i < contours_.size(); i++)
    {
      std::vector<Eigen::Vector3f> c;
      for(unsigned int j = 0; j < contours_[i].size(); j++)
        c.push_back((*this)[contours_[i][j]]);
      contours_3d.push_back(c);
    }
    return contours_3d;
  }

  void
  Polygon::updateAttributes(const Eigen::Vector3f &new_normal, const Eigen::Vector3f& new_origin)
  {
    //TODO: optionally provide x- or y-axis
    d_ = new_origin.dot(new_normal);
    if (d_ > 0) {
      normal_ = -new_normal;
      d_ = -d_;
    }
    else { normal_ = new_normal; }
    //pose_.translation() = new_centroid;
    //centroid = new_centroid;

    Eigen::Affine3f t;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal_.unitOrthogonal(), normal_, new_origin, t);
    pose_ = t.inverse();
    //pcl::getTransformationFromTwoUnitVectorsAndOrigin(
    //    this->normal_.unitOrthogonal(), this->normal_, new_centroid, this->pose_);
  }

  void Polygon::transform(const Eigen::Affine3f& trafo)
  {
    //transform contours
    //this->TransformContours(trafo);
    pose_ = trafo*pose_;
    normal_ = trafo.rotation()*normal_;
    d_ = pose_.translation().dot(normal_);
    if (d_ > 0)
    {
      normal_ *= -1;
      d_ = -d_;
    }

    //  transform parameters
    /*Eigen::Vector3f tf_normal = trafo.rotation() *this->normal;
  this->normal =tf_normal;
  Eigen::Vector3f tf_centroid3f = this->centroid.head(3);
  tf_centroid3f = trafo * tf_centroid3f;
  this->centroid.head(3) = tf_centroid3f;
  this->computeAttributes(this->normal,this->centroid);*/
  }


  void
  Polygon::smoothContours()
  {
    /*gpc_polygon *before = new gpc_polygon();
    gpc_polygon *after = new gpc_polygon();
    this->getGpcStructure(before, contours_);
    copyGpcStructure(gpc_in, gpc_out);*/
    std::vector<std::vector<Eigen::Vector2f> > contours_sm = contours_;
    float weight_data = 0.5, weight_smooth = 0.01, tolerance = 1.0f;
    float change = tolerance;

    for(unsigned int j = 0; j < contours_.size(); ++j)
    {
      int num_iteration = 0;
      int l = contours_[j].size(); // length
      //gpc_vertex* contours_sm[j] = gpc_out->contour[j].vertex; // new polygon
      //gpc_vertex* po = gpc_in->contour[j].vertex; // old polygon

      while(change >= tolerance && num_iteration <= 5)
      {
        change = 0.0f;
        for(int k=0; k<l; ++k)
        {
          //if( !k%10 ) continue;
          // do x value:
          float before = contours_sm[j][k](0);
          contours_sm[j][k](0) += weight_data * (contours_[j][k](0) - contours_sm[j][k](0));
          contours_sm[j][k](0) += weight_smooth * (contours_sm[j][ MOD(k-1,l) ](0) + contours_sm[j][ (k+1)%l ](0) - (2.0f * contours_sm[j][k](0)));
          contours_sm[j][k](0) += 0.5f * weight_smooth * (2.0f * contours_sm[j][ MOD(k-1,l) ](0) - contours_sm[j][ MOD(k-2,l) ](0) - contours_sm[j][k](0));
          contours_sm[j][k](0) += 0.5f * weight_smooth * (2.0f * contours_sm[j][    (k+1)%l ](0) - contours_sm[j][    (k+2)%l ](0) - contours_sm[j][k](0));
          change += fabs(before - contours_sm[j][k](0));

          // do y value:
          before = contours_sm[j][k](1);
          contours_sm[j][k](1) += weight_data * (contours_[j][k](1) - contours_sm[j][k](1));
          contours_sm[j][k](1) += weight_smooth * (contours_sm[j][ MOD(k-1,l) ](1) + contours_sm[j][ (k+1)%l ](1) - (2.0f * contours_sm[j][k](1)));
          contours_sm[j][k](1) += 0.5f * weight_smooth * (2.0f * contours_sm[j][ MOD(k-1,l) ](1) - contours_sm[j][ MOD(k-2,l) ](1) - contours_sm[j][k](1));
          contours_sm[j][k](1) += 0.5f * weight_smooth * (2.0f * contours_sm[j][    (k+1)%l ](1) - contours_sm[j][    (k+2)%l ](1) - contours_sm[j][k](1));
          change += fabs(before - contours_sm[j][k](1));
        }
        ++num_iteration;
      }
      ROS_DEBUG_STREAM( "Needed " << num_iteration << " iterations for polygon of size " << l );
    }
    contours_ = contours_sm;
  }


  //###########methods for merging##################


  void
  Polygon::getMergeCandidates(const std::vector<Polygon::Ptr>& poly_vec, std::vector<int>& intersections) const
  {
    for(size_t i=0; i< poly_vec.size(); ++i)
    {
      //std::cout << this->hasSimilarParametersWith(poly_vec[i]) << "," << this->isIntersectedWith(poly_vec[i]) << std::endl;
      if(this->hasSimilarParametersWith(poly_vec[i]) && this->isIntersectedWith(poly_vec[i])) intersections.push_back(i);
    }
  }

  bool
  Polygon::isIntersectedWith(const Polygon::Ptr& poly) const
  {
    gpc_polygon *gpc_res = new gpc_polygon();
    this->getIntersection(poly, gpc_res);
    bool res = (gpc_res->num_contours != 0);
    gpc_free_polygon(gpc_res);
    return (res);
  }


  void
  Polygon::getIntersection(const Polygon::Ptr& poly, gpc_polygon* gpc_intersection) const
  {
    /*std::cout << "####### Proj of P0 ###########" << std::endl;
  for(unsigned int i = 0; i < contours_[0].size(); i++)
  {
    std::cout << contours_[0][i](0) << "," << contours_[0][i](1) << "," << std::endl;
  }*/
    std::vector<std::vector<Eigen::Vector2f> > contours_p;
    projectContour(*poly, contours_p);
    /*std::cout << "####### Proj of P1 ###########" << std::endl;
  for(unsigned int i = 0; i < contours_p[0].size(); i++)
  {
    std::cout << contours_p[0][i](0) << "," << contours_p[0][i](1) << "," << std::endl;
  }*/
    gpc_polygon *gpc_poly = new gpc_polygon(), *gpc_here = new gpc_polygon();
    this->getGpcStructure(gpc_here, contours_);
    poly->getGpcStructure(gpc_poly, contours_p);
    gpc_polygon_clip(GPC_INT, gpc_here, gpc_poly, gpc_intersection);
    gpc_free_polygon(gpc_poly);
    gpc_free_polygon(gpc_here);
  }

  bool
  Polygon::getContourOverlap(const Polygon::Ptr& poly, float& rel_overlap, int& abs_overlap) const
  {
    std::vector<std::vector<Eigen::Vector2f> > contours_p;
    projectContour(*poly, contours_p);
    gpc_polygon *gpc_a = new gpc_polygon(), *gpc_b = new gpc_polygon();
    gpc_polygon *gpc_res_int = new gpc_polygon(), *gpc_res_union = new gpc_polygon();
    this->getGpcStructure(gpc_a, contours_);
    poly->getGpcStructure(gpc_b, contours_p);
    gpc_polygon_clip(GPC_INT, gpc_a, gpc_b, gpc_res_int);
    gpc_polygon_clip(GPC_UNION, gpc_a, gpc_b, gpc_res_union);

    int i_int = -1, i_union = -1;
    for(int i=0;i<gpc_res_int->num_contours;++i) { if(!gpc_res_int->hole[i]) { i_int = i; break; } }
    for(int i=0;i<gpc_res_union->num_contours;++i) { if(!gpc_res_union->hole[i]) { i_union = i; break; } }
    if(i_int == -1 || i_union == -1) return false;
    int overlap = 0;
    float d_th = pow( 0.01, 2 );
    for(int i=0;i<gpc_res_int->contour[i_int].num_vertices; ++i)
    {
      gpc_vertex* pv_int = &gpc_res_int->contour[i_int].vertex[i];
      for(int j=0;j<gpc_res_union->contour[i_union].num_vertices; ++j)
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
  Polygon::getOverlap(const Polygon& poly)
  {	  
    std::vector<std::vector<Eigen::Vector2f> > contours_p;
    projectContour(poly, contours_p);
    gpc_polygon *gpc_a = new gpc_polygon(), *gpc_b = new gpc_polygon();
    gpc_polygon *gpc_res_union = new gpc_polygon();
    this->getGpcStructure(gpc_a, contours_);
    poly.getGpcStructure(gpc_b, contours_p);
    gpc_polygon_clip(GPC_INT, gpc_a, gpc_b, gpc_res_union);
    
    applyGpcStructure(gpc_res_union);

    gpc_free_polygon(gpc_a);
    gpc_free_polygon(gpc_b);
    gpc_free_polygon(gpc_res_union);
    return computeArea3d();
  }


  float
  Polygon::computeSimilarity(const Polygon::Ptr& poly) const
  {
    float normal = (fabs(poly->normal_.dot(this->normal_)) - this->merge_settings_.angle_thresh) /
        (1-this->merge_settings_.angle_thresh);
    float d = fabs(fabs((this->pose_.translation()-poly->pose_.translation()).head(3).dot(this->normal_))-this->merge_settings_.d_thresh) /
        this->merge_settings_.d_thresh;
    float overlap = 0.0;
    int abs_overlap;
    this->getContourOverlap(poly, overlap, abs_overlap);
    return ( 3.0f / (1.0f / normal + 1.0f / d + 1.0f / overlap) );
  }

  void
  Polygon::merge(std::vector<Polygon::Ptr>& poly_vec)
  {
    assignID(poly_vec);
    //if(this->id==0) std::cout << "merge_weight before: " << this->merge_weight_ << "," << merged << std::endl;
    Polygon::Ptr p_average = Polygon::Ptr(new Polygon);
    computeAverage(poly_vec, p_average);
    mergeUnion(poly_vec, p_average);
    assignWeight();
    //if(this->id==0) std::cout << "merge_weight after: " << this->merge_weight_ << "," << merged  << std::endl;
  }

  void
  Polygon::mergeDifference(Polygon::Ptr& p_merge)
  {
    std::vector<std::vector<Eigen::Vector2f> > contours_p;
    p_merge->projectContour(*this, contours_p);
    gpc_polygon *gpc_C = new gpc_polygon;
    this->getGpcStructure(gpc_C, contours_p);
    //std::cout << this->transform_from_world_to_plane.matrix() << std::endl;

    gpc_polygon *gpc_B = new gpc_polygon;
    p_merge->getGpcStructure(gpc_B, p_merge->contours_);

    gpc_polygon_clip(GPC_DIFF, gpc_C, gpc_B, gpc_B);
    //std::cout << "contours: " << gpc_B->num_contours << std::endl;
    gpc_free_polygon(gpc_C);
    //copyGpcStructure(gpc_C, smoothed);
    //smoothGpcStructure(gpc_C, smoothed);
    p_merge->applyGpcStructure(gpc_B);
    gpc_free_polygon(gpc_B);
    //gpc_free_polygon(smoothed);
  }

  void
  Polygon::mergeUnion(std::vector<Polygon::Ptr>& poly_vec,  Polygon::Ptr& p_average)
  {
    //std::cout << "p_avg pose " << p_average->pose_.matrix() << std::endl;
    //if(this->id==0) std::cout << "\tthis rgb: " << this->color[0]*255 << "," << this->color[1]*255 << "," << this->color[2]*255 << std::endl;
    //d_color_.reset();
    //d_color_.setID(this->id_);
    std::vector<std::vector<Eigen::Vector2f> > contours;
    p_average->projectContour(*this, contours);
    /*std::cout << "####### Proj of P0 ###########" << std::endl;
  //std::cout << "normal: " << normal_(0) << "," << normal_(1) << "," << normal_(2) << std::endl;
  std::cout << pose_.matrix() << std::endl;
  for(unsigned int i = 0; i < contours[0].size(); i++)
  {
    std::cout << contours[0][i](0) << "," << contours[0][i](1) << "," << std::endl;
  }*/
    gpc_polygon *gpc_C = new gpc_polygon, *smoothed = new gpc_polygon;
    this->getGpcStructure(gpc_C, contours);

    double min_weight = this->merge_weight_;
    for(size_t i=0;i<poly_vec.size();++i)
    {
      if(poly_vec[i]->merge_weight_ < min_weight)
        min_weight = poly_vec[i]->merge_weight_;
    }
    double normalizer = 1/min_weight;

    for(size_t i=0;i<poly_vec.size();++i)
    {
      std::vector<std::vector<Eigen::Vector2f> > contours_p;
      p_average->projectContour(*poly_vec[i], contours_p);
      /*std::cout << "####### Proj of P1 ###########" << std::endl;
    //std::cout << "normal: " << poly_vec[i]->normal_(0) << "," << poly_vec[i]->normal_(1) << "," << poly_vec[i]->normal_(2) << std::endl;
    std::cout << poly_vec[i]->pose_.matrix() << std::endl;
    for(unsigned int j = 0; j < contours_p[0].size(); j++)
    {
      std::cout << contours_p[0][j](0) << "," << contours_p[0][j](1) << "," << std::endl;
    }*/
      gpc_polygon *gpc_B = new gpc_polygon;
      poly_vec[i]->getGpcStructure(gpc_B, contours_p);
      gpc_polygon_clip(GPC_UNION, gpc_B, gpc_C, gpc_C);
      gpc_free_polygon(gpc_B);
      //d_color_.addColor(poly_vec[i]->color_[0]*255,poly_vec[i]->color_[1]*255,poly_vec[i]->color_[2]*255, round(normalizer*poly_vec[i]->merge_weight_));
      //if(id==0) std::cout << "\tm_weight " << poly_vec[i]->id << ": " << poly_vec[i]->merge_weight_ << "," << poly_vec[i]->computeArea3d() << std::endl;
    }
    //d_color_.addColor(this->color_[0]*255,this->color_[1]*255,this->color_[2]*255,round(normalizer*this->merge_weight_));
    //if(id==0) std::cout << "\tm_weight " << this->merge_weight_ <<  "," << this->computeArea3d() << std::endl;

    /*std::cout << "####### Proj of Paverage ###########" << std::endl;
  std::cout << "normal: " << p_average->normal_(0) << "," << p_average->normal_(1) << "," << p_average->normal_(2) << std::endl;
  std::cout << p_average->pose_.matrix() << std::endl;*/
    // fill in parameters for "this" polygon
    setParamsFrom(p_average);
    //this->merge_weight_ = p_average->merge_weight_;
    /*copyGpcStructure(gpc_C, smoothed);
  smoothGpcStructure(gpc_C, smoothed);
  this->applyGpcStructure(smoothed);*/
    this->applyGpcStructure(gpc_C);
    gpc_free_polygon(gpc_C);
    /*std::cout << "####### Proj of Pmerged ###########" << std::endl;
  std::cout << "normal: " << normal_(0) << "," << normal_(1) << "," << normal_(2) << std::endl;
  std::cout << pose_.matrix() << std::endl;
  for(unsigned int i = 0; i < contours_[0].size(); i++)
  {
    std::cout << contours_[0][i](0) << "," << contours_[0][i](1) << "," << std::endl;
  }*/
    //gpc_free_polygon(smoothed);
    /*uint8_t r,g,b;
    d_color_.getColor(r,g,b);
    //std::cout << "\t" << (int)r << "," << (int)g << "," << (int)b << std::endl;
    float temp_inv = 1.0f/255.0f;
    //std::cout << r* temp_inv << "," << g* temp_inv << "," << b* temp_inv << std::endl;
    this->color_[0] = r * temp_inv;
    this->color_[1] = g * temp_inv;
    this->color_[2] = b * temp_inv;*/
    
    this->color_ = p_average->color_;
    //if(this->id==0) std::cout << "\tthis rgb: " << this->color[0]*255 << "," << this->color[1]*255 << "," << this->color[2]*255 << std::endl;
  }

  void
  Polygon::projectContour(const Polygon& p, std::vector<std::vector<Eigen::Vector2f> >& contours) const
  {
    //std::cout << pose_.matrix() << std::endl << std::endl;
    //std::cout << p.pose_.inverse().matrix() << std::endl << std::endl;
    for(unsigned int i = 0; i < p.contours_.size(); i++)
    {
      std::vector<Eigen::Vector2f> c;
      for(unsigned int j = 0; j < p.contours_[i].size(); j++)
      {
        //std::cout << p.contours_[i][j](0) << "," << p.contours_[i][j](1) << std::endl;
        Eigen::Vector2f pt_p = (pose_.inverse() * p.pose_ * Eigen::Vector3f(p.contours_[i][j](0), p.contours_[i][j](1), 0)).head(2);
        c.push_back(pt_p);
        //std::cout << pt_p(0) << "," << pt_p(1) << std::endl;
      }
      contours.push_back(c);
    }
  }

  void
  Polygon::assignWeight()
  {
    if (std::strcmp(merge_settings_.weighting_method.c_str(), "COUNTER") == 0)
    {
      merge_weight_ = merged_;
    }
    else if (std::strcmp(merge_settings_.weighting_method.c_str(), "AREA") == 0)
    {
      double area = computeArea3d();
      merge_weight_ = (merged_ * area);
    }
  }

  void
  Polygon::assignID(const std::vector<Polygon::Ptr>& poly_vec)
  {
    unsigned int tmp_id = poly_vec[0]->id_;
    for(size_t i = 0; i < poly_vec.size(); ++i)
    {
      if(poly_vec[i]->id_ < tmp_id) tmp_id = poly_vec[i]->id_;
    }
    this->id_ = tmp_id;
  }

  void
  Polygon::computeAverage(const std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr & p_average)
  {
    //std::cout << "w: " << merge_weight_ << std::endl;
    //std::cout << "merged: " << merged_ << std::endl;
    Eigen::Vector3f average_normal = normal_*merge_weight_;
    Eigen::Vector3f average_centroid = pose_.translation()*merge_weight_;
    double average_d = d_*merge_weight_;
    double sum_w = merge_weight_;
    unsigned int sum_merged = merged_;
    std::vector<float> sum_color = color_;
    BOOST_FOREACH(float &c, sum_color) {c*=merge_weight_;}
    //std::cout << "poly centroid" << pose_.translation()(0) << "," << pose_.translation()(1) << "," << pose_.translation()(2) << "," << std::endl;
    //std::cout << "poly normal" << normal_(0) << "," << normal_(1) << "," << normal_(2) << "," << std::endl;
    //std::cout << pose_.translation()*merge_weight_ << std::endl;

    for(int i=0 ; i< (int) poly_vec.size();i++)
    {
      Polygon& p_map1 =*(poly_vec[i]);

      if(normal_.dot(p_map1.normal_) < 0)
      {
        average_normal += p_map1.merge_weight_* -p_map1.normal_;
      }
      else
      {
        average_normal += p_map1.merge_weight_* p_map1.normal_;
      }
      average_centroid += p_map1.merge_weight_* p_map1.pose_.translation();
      //std::cout << "poly centroid" << p_map1.pose_.translation()(0) << "," << p_map1.pose_.translation()(1) << "," << p_map1.pose_.translation()(2) << "," << std::endl;
      //std::cout << "poly normal" << p_map1.normal_(0) << "," << p_map1.normal_(1) << "," << p_map1.normal_(2) << "," << std::endl;
      average_d += p_map1.merge_weight_ * p_map1.d_;
      sum_w += p_map1.merge_weight_;
      sum_merged += p_map1.merged_;
      for(size_t i=0; p_map1.color_.size()==sum_color.size() && i<p_map1.color_.size(); i++)
		sum_color[i] += p_map1.color_[i]*p_map1.merge_weight_;
    }

    average_normal = average_normal/sum_w;
    average_centroid = average_centroid/sum_w;
    average_d = average_d/sum_w;
    average_normal.normalize();

    if (sum_merged < merged_limit_)
    {
      p_average->merged_ = sum_merged;
    }
    else
    {
      p_average->merged_ = merged_limit_;
    }
    
    p_average->color_ = sum_color;
    BOOST_FOREACH(float &c, p_average->color_) {c/=sum_w;}
    //std::cout << "average centroid" << average_centroid(0) << "," << average_centroid(1) << "," << average_centroid(2) << "," << std::endl;
    //std::cout << "average_normal" << average_normal(0) << "," << average_normal(1) << "," << average_normal(2) << "," << std::endl;
    p_average->updateAttributes(average_normal, average_centroid);
    //std::cout << "average p: " << p_average->pose_.matrix() << std::endl;
  }

  void
  Polygon::setParamsFrom(Polygon::Ptr& p)
  {
    this->pose_ = p->pose_;
    this->d_ = p->d_;
    this->normal_ = p->normal_;
    if(this->merged_ < 9) { this->merged_ = p->merged_; }
    else { this->merged_ = 9; }
  }

  void
  Polygon::getGpcStructure(gpc_polygon* gpc_p, const std::vector<std::vector<Eigen::Vector2f> >& contours) const
  {
    // get transformed contours
    //std::vector< std::vector <Eigen::Vector3f> > transformed_contours;
    //this->getTransformedContours(external_trafo, transformed_contours);

    gpc_p->num_contours = contours.size();
    gpc_p->hole = (int*)malloc(contours.size()*sizeof(int));
    gpc_p->contour = (gpc_vertex_list*)malloc(contours.size()*sizeof(gpc_vertex_list));
    for(size_t j = 0; j<contours.size(); ++j)
    {
      gpc_p->hole[j] = holes_[j];
      gpc_p->contour[j].num_vertices = contours[j].size();
      gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));

      for(size_t k = 0; k < contours[j].size(); ++k)
      {
        //Eigen::Vector3f point_trans = transformed_contours[j][k];
        gpc_p->contour[j].vertex[k].x = contours[j][k](0);//point_trans(0);
        gpc_p->contour[j].vertex[k].y = contours[j][k](1);//point_trans(1);
      }
    }
  }


  void
  Polygon::applyGpcStructure(const gpc_polygon* gpc_p)
  {
    // clear contours, at the end gpc_C contains everything we need!
    this->contours_.clear();
    this->holes_.clear();
    for(int j = 0; j < gpc_p->num_contours; ++j)
    {
      this->contours_.push_back(std::vector<Eigen::Vector2f>());
      this->holes_.push_back(gpc_p->hole[j]);
      float last_x = 0.0, last_y = 0.0;
      for (int k = 0; k < gpc_p->contour[j].num_vertices; ++k)
      {
        // check if points are too close to each other (so remove similar points)
        if (fabs(gpc_p->contour[j].vertex[k].x - last_x) < 0.01 && fabs(gpc_p->contour[j].vertex[k].y - last_y) < 0.01)
          continue;
        last_x = gpc_p->contour[j].vertex[k].x;
        last_y = gpc_p->contour[j].vertex[k].y;
        this->contours_.back().push_back(Eigen::Vector2f(last_x,last_y));
      }

      if (this->contours_.back().size() <= 2)  // drop empty contour lists
      {
        this->contours_.pop_back();
        this->holes_.pop_back();
      }
    }
  }


  //#######methods for calculation#####################

  /*void
Polygon::computePose()
{
  Eigen::Vector3f centroid = computeCentroid();
  //Eigen::Vector3f u, v;
  //getCoordinateSystemOnPlane(normal_, u, v);
  Eigen::Affine3f t;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal_.unitOrthogonal(), normal_, centroid, t);
  pose_ = t.inverse();
}*/

  void
  Polygon::computePose(std::vector<std::vector<Eigen::Vector3f> >& contours_3d)
  {
    Eigen::Vector3f origin = d_*normal_;//contours_3d[0][0];//computeCentroid(contours_3d);
    //std::cout << "c: " << centroid << std::endl;
    //Eigen::Vector3f u, v;
    //getCoordinateSystemOnPlane(normal_, u, v);
    Eigen::Affine3f t;
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal_.unitOrthogonal(), normal_, origin, t);
    pose_ = t.inverse();
  }

  Eigen::Vector3f
  Polygon::computeCentroid() const
  {
    std::vector<std::vector<Eigen::Vector3f> > contours_3d = getContours3D();
    return computeCentroid(contours_3d);
    //find largest non-hole contour
    /*unsigned int idx = 0;
    for (unsigned int i = 0; i < contours_.size (); i++)
    {
      int max_pts = 0;
      if(!holes_[i])
      {
        if((int)contours_[i].size()>(int)max_pts)
        {
          max_pts = contours_[i].size();
          idx = i;
        }
      }
    }
    pcl::PointCloud<pcl::PointXYZ> poly_cloud;
    for (unsigned int j = 0; j < contours_[idx].size () ; j++)
    {
      Eigen::Vector4f pt_c_4f(Eigen::Vector4f::Zero());
      pt_c_4f.head(2) = contours_[idx][j];
      pcl::PointXYZ p;
      p.getVector4fMap() = pose_*pt_c_4f;
      poly_cloud.push_back(p);
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(poly_cloud, centroid);
    return centroid.head(3);*/
  }

  Eigen::Vector3f
  Polygon::computeCentroid(std::vector<std::vector<Eigen::Vector3f> >& contours_3d) const
  {
    if(contours_3d.size()<1) return Eigen::Vector3f::Zero();
    
    //find largest non-hole contour
    size_t idx = 0;
    for (size_t i = 0; i < contours_3d.size (); i++)
    {
      int max_pts = 0;
      if(!holes_[i])
      {
        if((int)contours_3d[i].size()>(int)max_pts)
        {
          max_pts = contours_3d[i].size();
          idx = i;
        }
      }
    }
    pcl::PointCloud<pcl::PointXYZ> poly_cloud;
    for(size_t j = 0; j < contours_3d[idx].size() ; j++)
    {
      pcl::PointXYZ p;
      p.getVector3fMap() = contours_3d[idx][j];
      poly_cloud.push_back(p);
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(poly_cloud, centroid);
    return centroid.head(3);
  }


  double
  Polygon::computeArea3d() const
  {
    double area = 0;

    std::list<TPPLPoly> tri_list;
    triangulate(tri_list);
    for (std::list<TPPLPoly>::iterator it = tri_list.begin (); it != tri_list.end (); it++)
    {
      TPPLPoint pt1 = it->GetPoint(0);
      TPPLPoint pt2 = it->GetPoint(1);
      TPPLPoint pt3 = it->GetPoint(2);
      area += 0.5*fabs((pt2.x - pt1.x)*(pt3.y - pt1.y) - (pt3.x -pt1.x)*(pt2.y - pt1.y));
    }
    return area;
  }

  void
  Polygon::triangulate(std::list<TPPLPoly>& tri_list) const
  {
    TPPLPartition pp;
    std::list<TPPLPoly> polys;
    TPPLPoly poly;
    TPPLPoint pt;

    for(size_t j=0;j<contours_.size();j++)
    {
      poly.Init(contours_[j].size());
      poly.SetHole(holes_[j]);
      for(size_t i=0; i<contours_[j].size(); ++i)
      {
        pt.x = contours_[j][i](0);
        pt.y = contours_[j][i](1);
        poly[i] = pt;
      }
      if (holes_[j])
        poly.SetOrientation(TPPL_CW);
      else
        poly.SetOrientation(TPPL_CCW);
      polys.push_back(poly);
    }
    // triangulation into monotone triangles
    pp.Triangulate_EC (&polys, &tri_list);
  }

  //#############debugging methods#######################

  void Polygon::debugOutput(std::string name)
  {
    std::string path = "/home/goa-tz/debug/dbg/";
    path.append(name.c_str());
    std::ofstream os(path.c_str());

    for (int i = 0; i < (int) this->contours_.size(); ++i)
    {
      for (int j = 0; j < (int) this->contours_[i].size(); ++j)
      {
        os << contours_[i][j] << std::endl;
      }
    }

    os.close();
  }

}//namespace



/*double
Polygon::computeArea() const
{
  // IMPORTANT: computes area only for the projection of the polygon onto XY plane
  // therefore  the resulting area is not the true area of the polygon
  double xi, xi_1, yi, yi_1, area=0;
  double sum;
  for (unsigned int i = 0; i < contours_.size (); i++)
  {
    if(holes_[i]) continue;
    sum = 0;
    //area_[i] = 0;
    for (unsigned int j = 0; j < contours_[i].size (); j++)
    {
      xi = contours_[i][j][0];
      yi = contours_[i][j][1];
      if (j == (contours_[i].size ()) - 1)
      {
        xi_1 = contours_[i][0][0];
        yi_1 = contours_[i][0][1];
      }
      else
      {
        xi_1 = contours_[i][j + 1][0];
        yi_1 = contours_[i][j + 1][1];
      }
      sum = sum + (xi * yi_1 - xi_1 * yi);

    }
    area += fabs (sum / 2);
  }
  return area;
}*/

/*void
Polygon::computePoseAndBoundingBox(Eigen::Affine3f& pose, Eigen::Vector4f& min_pt, Eigen::Vector4f& max_pt)
{
  pcl::PointCloud<pcl::PointXYZ> poly_cloud;
  unsigned int idx = 0;
  for (unsigned int j = 0; j < contours_[idx].size () ; j++)
  {
    Eigen::Vector4f pt_c_4f(Eigen::Vector4f::Zero());
    pt_c_4f.head(2) = contours_[idx][j];
    pcl::PointXYZ p;
    p.getVector4fMap() = pose_*pt_c_4f;
    poly_cloud.push_back(p);
  }
  Eigen::Matrix3f cov;
  Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
  centroid.head(3) = pose_.translation();
  pcl::computeCovarianceMatrix (poly_cloud, centroid, cov);
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (cov, eigen_vectors, eigen_values);
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(eigen_vectors.col(1),eigen_vectors.col(0),pose_.translation(),pose);

  pcl::PointCloud<pcl::PointXYZ> cloud_trans;
  pcl::transformPointCloud(poly_cloud, cloud_trans, pose);
  pcl::getMinMax3D(cloud_trans, min_pt, max_pt);
}*/

/*void
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
this->normal_ = normal;
pcl::getTransformationFromTwoUnitVectorsAndOrigin(z_axis, normal, origin, transformation);
transformation = transformation.inverse();
}*/

/*void
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
}*/
