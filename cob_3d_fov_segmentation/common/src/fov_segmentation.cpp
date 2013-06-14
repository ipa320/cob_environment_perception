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
 * ROS package name: cob_3d_fov_segmentation
 *
 * \author
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 02/2013
 *
 * \brief
 * Computes field of view segmentation of a shape array.
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


//##################
//#### includes ####

#include <map>
#include <cob_3d_fov_segmentation/fov_segmentation.h>
//#include <pcl/surface/convex_hull.h>

using namespace cob_3d_mapping;


// Constructor
FOVSegmentation::FOVSegmentation()
{
}

bool FOVSegmentation::ccw(Eigen::Vector2f& a, Eigen::Vector2f& b, Eigen::Vector2f& c)
{
        return (c(1)-a(1))*(b(0)-a(0)) > (b(1)-a(1))*(c(0)-a(0));
}

void FOVSegmentation::compute(std::vector<Polygon::Ptr>& polygons)
{
  for(polygon_iterator it = polygons_in_.begin(); it != polygons_in_.end(); it++)
  {
    std::vector<Eigen::Vector3f> intersections;
    clipFOVandPlane(*it, intersections);
    std::vector<std::vector<Eigen::Vector3f> > intersections2;
    intersections2.push_back(intersections);
    /*if( intersections.size() < 3)
    {
      //check if centroid is inside tetraeder
      //find clipping point of fov plane and vector between 0 and p
      std::vector<Eigen::Vector3d> p(5);
      fov_.getFOV(p[0], p[1], p[2], p[3], p[4]);
      Eigen::Vector3d normal = (p[2]-p[1]).cross(p[3]-p[2]).normalized();
      Eigen::Vector4f centroid;
      centroid << p[2].cast<float>(),1;
      //std::cout << normal << std::endl;
      //std::cout << p[2].cast<float>().head(3) << std::endl;
      Polygon::Ptr poly(new Polygon);
      poly->computeAttributes(normal.cast<float>(), centroid);
      double div = normal.dot(poly->centroid.topLeftCorner(3, 1).cast<double>()-p[0]);
      double lambda = (normal.dot(p[2] - p[0]))/div;
      Eigen::Vector3d intersection = p[0] + lambda*(poly->centroid.topLeftCorner(3, 1).cast<double>()-p[0]);
      Eigen::Vector2f inters_plane = (poly->transform_from_world_to_plane * intersection.cast<float>()).head(2);
      Eigen::Vector2f p1_plane = (poly->transform_from_world_to_plane * p[1].cast<float>()).head(2);
      Eigen::Vector2f p2_plane = (poly->transform_from_world_to_plane * p[2].cast<float>()).head(2);
      Eigen::Vector2f p3_plane = (poly->transform_from_world_to_plane * p[3].cast<float>()).head(2);
      Eigen::Vector2f p4_plane = (poly->transform_from_world_to_plane * p[4].cast<float>()).head(2);
      std::cout << "inters: " << inters_plane(0) << "," << inters_plane(1) << std::endl;
      std::cout << "p1: " << p1_plane(0) << "," << p1_plane(1) << std::endl;
      std::cout << "p2: " << p2_plane(0) << "," << p2_plane(1) << std::endl;
      std::cout << "p3: " << p3_plane(0) << "," << p3_plane(1) << std::endl;
      std::cout << "p4: " << p4_plane(0) << "," << p4_plane(1) << std::endl;
      //check if clipping point is inside fov rectangle
      if(ccw(p2_plane,p1_plane,inters_plane) &&
          ccw(p3_plane,p2_plane,inters_plane) &&
          ccw(p4_plane,p3_plane,inters_plane) &&
          ccw(p1_plane,p4_plane,inters_plane))
          {
            //if all ccw are true => inside, else outside
        std::cout << "polygon is inside" << std::endl;
            continue;
          }
    }*/
    if(intersections.size() != 0)
    {
      Polygon::Ptr fov_poly(new Polygon(**it));
      fov_poly->contours_.clear();
      fov_poly->holes_.clear();
      //fov_poly->contours_.push_back(intersections);
      fov_poly->setContours3D(intersections2);
      fov_poly->holes_.push_back(false);
      (*it)->mergeDifference(fov_poly);
      std::cout << "clipped poly:" << fov_poly->contours_.size() << std::endl;
      if (fov_poly->contours_.size() > 0)
        polygons.push_back(fov_poly);
    }
    else
      polygons.push_back(*it);
    /*std::cout << fov_poly->contours[0][0](0) << "," <<  fov_poly->contours[0][0](1) << "," << fov_poly->contours[0][0](2) << std::endl;
    std::cout << fov_poly->contours[0][1](0) << "," <<  fov_poly->contours[0][1](1) << "," << fov_poly->contours[0][1](2) << std::endl;
    std::cout << fov_poly->contours[0][2](0) << "," <<  fov_poly->contours[0][2](1) << "," << fov_poly->contours[0][2](2) << std::endl;
    std::cout << fov_poly->contours[0][3](0) << "," <<  fov_poly->contours[0][3](1) << "," << fov_poly->contours[0][3](2) << std::endl;*/
  }
}

void FOVSegmentation::clipFOVandPlane(Polygon::Ptr& poly, std::vector<Eigen::Vector3f>& intersections)
{
  std::vector<Eigen::Vector3d> p(5);
  fov_.getFOV(p[0], p[1], p[2], p[3], p[4]);

  for( unsigned int i=1; i<p.size(); i++)
  {
    double div = poly->normal_.cast<double>().dot(p[i]-p[0]);
    /*std::cout << poly->normal << std::endl;
    std::cout << p[i] << std::endl;
    std::cout << p[0] << std::endl;*/
    //std::cout << "div: " << div << std::endl;
    if ( div == 0 ) continue;
    double lambda = (poly->normal_.cast<double>().dot(poly->pose_.translation().cast<double>() - p[0]))/div;
    //std::cout << "lambda: " << lambda << std::endl;
    if ( lambda > 1 || lambda < 0) continue;
    Eigen::Vector3d intersection = p[0] + lambda*(p[i]-p[0]);
    //std::cout << i << ": " << intersection(0) << "," << intersection(1) << "," << intersection(2) << std::endl;
    intersections.push_back(intersection.cast<float>());
  }
  //std::cout << std::endl;
  for( unsigned int i=1; i<p.size(); i++)
  {
    unsigned int j = i+1;
    if ( j == p.size() ) j = 1;
    double div = poly->normal_.cast<double>().dot(p[j]-p[i]);
    if ( div == 0 ) continue;
    double lambda = (poly->normal_.cast<double>().dot(poly->pose_.translation().cast<double>() - p[i]))/div;
    if ( lambda > 1  || lambda < 0) continue;
    Eigen::Vector3d intersection = p[i] + lambda*(p[j]-p[i]);
    //std::cout << i <<"," << j << ": " << intersection(0) << "," << intersection(1) << "," << intersection(2) << std::endl;
    /*std::cout << "div, l: "<< div << "," << lambda << std::endl;
    std::cout << "p" << i << ": " << p[i](0) << "," << p[i](1) << "," << p[i](2) << std::endl;
    std::cout << "p" << j << ": " << p[j](0) << "," << p[j](1) << "," << p[j](2) << std::endl;*/
    intersections.push_back(intersection.cast<float>());
  }
  std::cout << "found " << intersections.size() << "intersecting points for ID " << poly->id_ << std::endl;
  if(intersections.size() == 5)
  {
    //calc angle rel. to centroid
    std::vector<Eigen::Vector2f> cs;
    Eigen::Vector2f cent(0,0);
    for(unsigned int i=0; i<intersections.size(); i++)
      {
        cs.push_back((poly->pose_.inverse() * intersections[i]).head(2));
        //std::cout << "cs" << i << ": " << (poly->transform_from_world_to_plane * intersections[i]).head(2)(0) << "," << (poly->transform_from_world_to_plane * intersections[i]).head(2)(1) << std::endl;
        cent = cent + (poly->pose_.inverse() * intersections[i]).head(2);
      }

    cent = cent/5;
    //std::cout << "centroid " << cent(0) << "," << cent(1) << std::endl;
    Eigen::Vector2f ref = (cs[0] - cent).normalized();
    std::map<double, unsigned int> angles1;
    std::map<double, unsigned int> angles2;
    for(unsigned int i=1; i<cs.size(); i++)
    {
      if(ccw(cent, cs[0], cs[i]))
      {
        angles1.insert(std::pair<double, unsigned int>((cs[i]-cent).normalized().dot(ref),i));
        //std::cout << "angle1 " << i << ": " << acos((cs[i]-cent).normalized().dot(ref))*180/3.14159 << std::endl;
      }
      else
      {
        angles2.insert(std::pair<double, unsigned int>((cs[i]-cent).normalized().dot(ref),i));
        //std::cout << "angle2 " << i << ": " << acos((cs[i]-cent).normalized().dot(ref))*180/3.14159 << std::endl;
      }
    }
    std::vector<unsigned int> sorted;
    sorted.push_back(0);
    for(std::map<double, unsigned int>::reverse_iterator it = angles1.rbegin(); it != angles1.rend(); it++)
      sorted.push_back(it->second);
    for(std::map<double, unsigned int>::const_iterator it = angles2.begin(); it != angles2.end(); it++)
      sorted.push_back(it->second);
    std::vector<Eigen::Vector3f> inters_temp(sorted.size());
    for(unsigned int i=0; i<sorted.size(); i++)
    {
      inters_temp[i] = intersections[sorted[i]];
    }
    intersections.swap(inters_temp);
    //for(unsigned int i=0; i<intersections.size(); i++)
    //  std::cout << "inter: " << intersections[i](0) << "," << intersections[i](1) << "," << intersections[i](2) << std::endl;
  }
  if(intersections.size() == 4)
  {
    Eigen::Vector2f c1 = (poly->pose_.inverse() * intersections[0]).head(2);
    Eigen::Vector2f c2 = (poly->pose_.inverse() * intersections[1]).head(2);
    Eigen::Vector2f c3 = (poly->pose_.inverse() * intersections[2]).head(2);
    Eigen::Vector2f c4 = (poly->pose_.inverse() * intersections[3]).head(2);
    if(ccw(c1,c2,c3))
    {
      if(ccw(c1,c3,c4)) ;
      else
      {
        if(ccw(c1,c2,c4)) std::swap(intersections[3],intersections[2]);
        else std::swap(intersections[0],intersections[3]);
      }
    }
    else
    {
      if(ccw(c1,c3,c4))
      {
        if(ccw(c1,c2,c4)) std::swap(intersections[1],intersections[2]);
        else std::swap(intersections[0],intersections[1]);
      }
      else std::swap(intersections[0],intersections[2]);
    }
  }
}



