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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_semantics
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
 * ToDo:
 * add comments in doxygen style
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
//internal include
#include "cob_3d_mapping_semantics/semantic_extraction.h"

bool
SemanticExtraction::isHorizontal (PolygonPtr p_ptr)
{
  //check components of normal vector with threshold values
  if ((p_ptr->normal (2) >= norm_z_max_ || p_ptr->normal (2) <= norm_z_min_) && (p_ptr->normal (0) < norm_x_max_
      && p_ptr->normal (0) > norm_x_min_) && (p_ptr->normal (1) < norm_y_max_ && p_ptr->normal (1) > norm_y_min_))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void
SemanticExtraction::isHeightOk (PolygonPtr p_ptr)
{
  poly_height_.resize(0);
  //calcPolyCentroid (p_ptr);
  computeCentroidPCL(p_ptr);

  for(unsigned int i=0;i<p_ptr->poly_points.size();i++)
  {
    //check centroid's z component with threshold values for height
    if (centroid_[i].z > height_min_ && centroid_[i].z < height_max_)
    {
      poly_height_.push_back(true);
    }

    else
    {
      poly_height_.push_back(false);
    }
  }
  std::cout << "\n\t**poly_height_ = " << poly_height_.size() << std::endl;
}


void
SemanticExtraction::isSizeOk (PolygonPtr p_ptr)
{
  poly_size_.resize(0);
  calcPolyArea (p_ptr);
  //TODO: parameters

  for(unsigned int i=0;i<p_ptr->poly_points.size();i++)
   {
      if (area_[i] >= area_min_ && area_[i] <= area_max_)
        poly_size_.push_back(true);
      else if (area_[i] < area_min_)
      {
        std::cout << "\tSize is small "<< std::endl;
        poly_size_.push_back(false);
      }
      //TODO: really 31?

      else if (area_[i] > area_max_)
      {
        std::cout << "\tSize is large "<< std::endl;
        poly_size_.push_back(false);
      }
    }
  std::cout << "\n\t***poly_size_ = " << poly_size_.size() << std::endl;

}

// http://paulbourke.net/geometry/polyarea/
// works for all polygons except self-intersecting polygons

void
SemanticExtraction::calcPolyArea (PolygonPtr p_ptr)
{
  double xi, xi_1, yi, yi_1;

  area_.resize (p_ptr->poly_points.size ());
  double sum;
  for (unsigned int i = 0; i < p_ptr->poly_points.size (); i++)
  {
    sum = 0;
    area_[i] = 0;
    for (unsigned int j = 0; j < p_ptr->poly_points[i].size (); j++)
    {
      xi = p_ptr->poly_points[i][j][0];
      yi = p_ptr->poly_points[i][j][1];
      if (j == (p_ptr->poly_points[i].size ()) - 1)
      {
        xi_1 = p_ptr->poly_points[i][0][0];
        yi_1 = p_ptr->poly_points[i][0][1];
      }
      else
      {
        xi_1 = p_ptr->poly_points[i][j + 1][0];
        yi_1 = p_ptr->poly_points[i][j + 1][1];
      }
      sum = sum + (xi * yi_1 - xi_1 * yi);
      /*
       std::cout << " ---------------------------------------" << std::endl;
       std::cout << " isSizeOk: xi-->" << xi << std::endl;
       std::cout << " \t: xi_1-->" << xi_1 << std::endl;
       std::cout << " \nisSizeOk: yi-->" << yi << std::endl;
       std::cout << " \t: yi_1-->" << yi_1 << std::endl;
       std::cout << " isSizeOk: sum-->" << sum << std::endl;
       std::cout << " ++++++++++++++++++++++++++++++++++++++++" <<std::endl;
       */

    }
    area_[i] = fabs (sum / 2);
    std::cout << "\n\t*** Area of polygon ( " << i << " ) = " << area_[i] << std::endl;
  }

}

void
SemanticExtraction::calcPolyCentroid (PolygonPtr p_ptr)
{
  //static int ctr = 0;
  double x0, y0, z0, xi, yi, zi, xi_1, yi_1, zi_1;

  double sum1, sum2, sum3, area;

  pcl::PointXYZ c_tri;
  pcl::PointXYZ cross_p;
  double tri_area;
  double mag_cross_p;

  centroid_.resize (p_ptr->poly_points.size ());
  for (unsigned int i = 0; i < p_ptr->poly_points.size (); i++)
  {
    /*
    std::ofstream poly_file; //file to store the polygon points and its centroid
    std::stringstream ss;
    ss << "/home/goa-wq/polygon_semantics/poly_" << ctr << ".txt";
    ctr++;
    ctr = ctr % 5;
    poly_file.open (ss.str ().c_str ());
    poly_file << "\n#\t*** polygon points\n\n";
    poly_file << "#\t'X'\t\t'Y'\t\t'Z'\n\n";
  */
    //initialize variables
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
    area = 0;

    pcl::PointXYZ v1, v2;
    x0 = p_ptr->poly_points[i][0][0];
    y0 = p_ptr->poly_points[i][0][1];
    z0 = p_ptr->poly_points[i][0][2];
    for (unsigned int j = 1; j < (p_ptr->poly_points[i].size () - 1); j++)
    {
      /*
      if (poly_file.is_open ())
      {
        poly_file << "\t" << p_ptr->poly_points[i][j][0];
        poly_file << "\t" << p_ptr->poly_points[i][j][1];
        poly_file << "\t\t" << p_ptr->poly_points[i][j][2] << "\n";
      }
      */
      /*
       std::cout<<"\t"<<p_ptr->poly_points[i][j][0];//<<std::endl;
       std::cout<<"\t"<<p_ptr->poly_points[i][j][1];//<<std::endl;
       std::cout<<"\t\t"<<p_ptr->poly_points[i][j][2]<<std::endl;
       */
      v1.x = 0;
      v1.y = 0;
      v1.z = 0;
      v2.x = 0;
      v2.y = 0;
      v2.z = 0;

      mag_cross_p = 0;
      tri_area = 0;
      xi = p_ptr->poly_points[i][j][0];
      yi = p_ptr->poly_points[i][j][1];
      zi = p_ptr->poly_points[i][j][2];

      xi_1 = p_ptr->poly_points[i][j + 1][0];
      yi_1 = p_ptr->poly_points[i][j + 1][1];
      zi_1 = p_ptr->poly_points[i][j + 1][2];

      v1.x = xi - x0;
      v1.y = yi - y0;
      v1.z = zi - z0;

      v2.x = xi_1 - x0;
      v2.y = yi_1 - y0;
      v2.z = zi_1 - z0;

      cross_p.x = ((v1.y * v2.z) - (v2.y * v1.z));
      cross_p.y = -((v1.x * v2.z) - (v1.z * v2.x));
      cross_p.z = ((v1.x * v2.y) - (v1.y * v2.x));

      mag_cross_p = sqrt (pow (cross_p.x, 2) + pow (cross_p.y, 2) + pow (cross_p.z, 2));
      tri_area = mag_cross_p / 2;

      c_tri.x = (x0 + xi + xi_1) / 3;
      c_tri.y = (y0 + yi + yi_1) / 3;
      c_tri.z = (z0 + zi + zi_1) / 3;

      sum1 = sum1 + (c_tri.x * tri_area);
      sum2 = sum2 + (c_tri.y * tri_area);
      sum3 = sum3 + (c_tri.z * tri_area);
      area = area + tri_area;
      /*
       std::cout << "\n\t sum1 = " << sum1;// << std::endl;
       std::cout << "\n\t sum2 = " << sum2;// << std::endl;
       std::cout << "\n\t sum3 = " << sum3 << std::endl;
       */
    }
    centroid_[i].x = (sum1 / area);
    centroid_[i].y = (sum2 / area);
    centroid_[i].z = (sum3 / area);
    p_ptr->centroid = centroid_;
    /*
    poly_file << "\n#\t*** Centroid of polygon\n";
    poly_file << "\t" << centroid_[i].x;
    poly_file << "\t" << centroid_[i].y;
    poly_file << "\t" << centroid_[i].z << "\n";

    poly_file.close ();
    */
    std::cout << "\n\t*** Centroid of polygon ( " << i << " ) " << std::endl;
    std::cout << "\tCx:" << centroid_[i].x;
    std::cout << "\tCy:" << centroid_[i].y;
    std::cout << "\tCz:" << centroid_[i].z << std::endl;
    std::cout << "\n\t*** Area of polygon ( " << i << " ) = "<< area << std::endl;

  }

}

void
SemanticExtraction::computeCentroidPCL(PolygonPtr p_ptr)
{
  centroid_.resize(p_ptr->poly_points.size ());
  for (unsigned int i = 0; i < p_ptr->poly_points.size (); i++)
    {
      pcl::PointCloud<pcl::PointXYZ> poly_cloud;
      for (unsigned int j = 0; j < p_ptr->poly_points[i].size () ; j++)
      {
        pcl::PointXYZ p;
        p.x = p_ptr->poly_points[i][j][0];
        p.y = p_ptr->poly_points[i][j][1];
        p.z = p_ptr->poly_points[i][j][2];
        poly_cloud.push_back(p);
      }
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(poly_cloud,centroid);

      centroid_[i].x = centroid(0);
      centroid_[i].y = centroid(1);
      centroid_[i].z = centroid(2);
      p_ptr->centroid = centroid_;

      std::cout<<"\n\t**ComputeCentroidPCL";
      std::cout<<"\n\t C(x) = "<<centroid(0);
      std::cout<<"\n\t C(y) = "<<centroid(1);
      std::cout<<"\n\t C(z) = "<<centroid(2);
    }
}

