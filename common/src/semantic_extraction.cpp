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

  /*
   std::cout << "\n___________~~~~~~''ROS PARAMETERS''~~~~~~___________"<< std::endl;
   std::cout << "\n\t*norm_x_min = " << norm_x_min_ << std::endl;
   std::cout << "\n\t*norm_x_max = " << norm_x_max_ << std::endl;
   std::cout << "\n\t*norm_y_min = " << norm_y_min_ << std::endl;
   std::cout << "\n\t*norm_y_max = " << norm_y_max_ << std::endl;
   std::cout << "\n\t*norm_z_min = " << norm_z_min_ << std::endl;
   std::cout << "\n\t*norm_z_max = " << norm_z_max_ << std::endl;
   std::cout << "\n___________~~~~~~''ROS PARAMETERS''~~~~~~___________"<< std::endl;
   //float norm = sqrt(pow(p_ptr->normal (0), 2) + pow(p_ptr->normal (1), 2) + pow(p_ptr->normal (2), 2));
   */
  /*
   std::cout<<"\t%Nx: "<<p_ptr->normal(0)<<"----"<<norm<<std::endl;
   std::cout<<"\t%Ny: "<<p_ptr->normal(1)<<std::endl;
   std::cout<<"\t%Nz: "<<p_ptr->normal(2)<<std::endl;
   */
  // TODO: define as parameters (declare member variable and set it from the node using ROS parameters)
  // TODO: better use && instead of three if-conditions

  /*
   if (p_ptr->normal (2) > 0.99 || p_ptr->normal (2) < -0.99)
   {
   if (p_ptr->normal (0) < 0.1 && p_ptr->normal (0) > -0.1)
   {
   if (p_ptr->normal (1) < 0.1 && p_ptr->normal (1) > -0.1)
   {
   return true;
   }
   }
   }
   */
  if ((p_ptr->normal (2) > norm_z_max_ || p_ptr->normal (2) < norm_z_min_) && (p_ptr->normal (0) < norm_x_max_
      && p_ptr->normal (0) > norm_x_min_) && (p_ptr->normal (1) < norm_y_max_ && p_ptr->normal (1) > norm_y_min_))
  {
    return true;
  }
  else
  {
    return false;
  }

}

bool
SemanticExtraction::isHeightOk (PolygonPtr p_ptr)
{
  calcPolyArea (p_ptr);
  calcPolyCentroid (p_ptr);

  if (centroid_[0].z > height_min_ && centroid_[0].z < height_max_)
    return true;

  else
    return false;
  // for (unsigned int i = 0; i < p_ptr->poly_points.size (); i++)
  //  {
  //
  // }

  /*
   int count1 = 0;
   int count2 = 0;
   for (unsigned int i = 0; i < p_ptr->poly_points.size (); i++)
   {
   for (unsigned int j = 0; j < p_ptr->poly_points[i].size (); j++)
   {
   //std::cout<<"\t%x: "<<p_ptr->poly_points[i][j][0]<<std::endl;
   //std::cout<<"\t%y: "<<p_ptr->poly_points[i][j][1]<<std::endl;
   //std::cout<<"\t%z: "<<p_ptr->poly_points[i][j][2]<<std::endl;

   // TODO: parameters
   // TODO: what is the purpose of the 2 counters? Shouldn't we better check the height of the centroid?
   if ((p_ptr->poly_points[i][j][2] < 1) && (p_ptr->poly_points[i][j][2] > 0.4))
   {
   count1++;
   //std::cout<<" count1: "<<count1<<std::endl;
   if (count1 > 10)
   {
   return true;
   break;
   }
   else
   continue;
   }
   else
   {
   count2++;
   //std::cout<<" count2: "<<count2<<std::endl;
   if (count2 > 10)
   {
   return false;
   break;
   }
   else
   continue;

   }

   }//end for inner
   }//end for outer

   */
}

bool
SemanticExtraction::isSizeOk (PolygonPtr p_ptr)
{
  //float* area;
  calcPolyArea (p_ptr);
  //TODO: parameters

  //for(int i=0;i<area_.size();i++)
  // {
  if (area_[0] >= area_min_ && area_[0] <= area_max_)
    return true;
  else if (area_[0] < area_min_)
  {
    std::cout << " Size is small " << area_[0] << std::endl;
    return false;
  }
  //TODO: really 31?
  else if (area_[0] > area_max_)
  {
    std::cout << " Size is large " << area_[0] << std::endl;
    return false;
  }
  // }


}

// http://paulbourke.net/geometry/polyarea/
// works for all polygons except self-intersecting polygons

void
SemanticExtraction::calcPolyArea (PolygonPtr p_ptr)
{
  double xi, xi_1, yi, yi_1;
  //float area[p_ptr->poly_points.size ()];

  area_.resize (p_ptr->poly_points.size ());
  double sum;
  for (unsigned int i = 0; i < p_ptr->poly_points.size (); i++)
  {

    //std::cout << " p_ptr->poly_points[i].size----:" << p_ptr->poly_points.size () << std::endl;
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
    area_[i] = abs (sum / 0.5);
    std::cout << "\n\t*** Area of polygon ( " << i << " ) = " << area_[i] << std::endl;
  }

}

void
SemanticExtraction::calcPolyCentroid (PolygonPtr p_ptr)
{

  double x0, y0, z0, xi, yi, zi, xi_1, yi_1, zi_1;

  double sum1, sum2, sum3;

  Point c_tri;
  double tri_area;
  Point cross_p;
  double mag_cross_p;
  centroid_.resize (p_ptr->poly_points.size ());

  for (unsigned int i = 0; i < p_ptr->poly_points.size (); i++)
  {
    //std::cout << " p_ptr->poly_points[i].size----:" << p_ptr->poly_points.size () << std::endl;
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;

    Point v1, v2;
    x0 = p_ptr->poly_points[i][0][0];
    y0 = p_ptr->poly_points[i][0][1];
    z0 = p_ptr->poly_points[i][0][2];
    for (unsigned int j = 1; j < (p_ptr->poly_points[i].size () - 1); j++)
    {

      v1.x = 0;
      v1.y = 0;
      v1.z = 0;
      v2.x = 0;
      v2.y = 0;
      v2.z = 0;

      mag_cross_p = 0;

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
      //std::cout << "\n\t mag_cross_p = " << mag_cross_p << std::endl;
      c_tri.x = (x0 + xi + xi_1) / 3;
      c_tri.y = (y0 + yi + yi_1) / 3;
      c_tri.z = (z0 + zi + zi_1) / 3;

      sum1 = sum1 + (c_tri.x * tri_area);
      sum2 = sum2 + (c_tri.y * tri_area);
      sum3 = sum3 + (c_tri.z * tri_area);
/*
      std::cout << "\n\t sum1 = " << sum1 << std::endl;
      std::cout << "\n\t sum2 = " << sum2 << std::endl;
      std::cout << "\n\t sum3 = " << sum3 << std::endl;
      */
    }

    if (area_[i] == 0)
    {
      centroid_[i].x = sum1;
      centroid_[i].y = sum2;
      centroid_[i].z = sum3;
    }
    else
    {
      centroid_[i].x = (sum1 / area_[i]);
      centroid_[i].y = (sum2 / area_[i]);
      centroid_[i].z = (sum3 / area_[i]);
    }
    std::cout << "\n\t*** Centroid of polygon ( " << i << " ) " << std::endl;
    std::cout << "\n\t Cx = " << centroid_[i].x << std::endl;
    std::cout << "\n\t Cy = " << centroid_[i].y << std::endl;
    std::cout << "\n\t Cz = " << centroid_[i].y << std::endl;
  }

}

/*
 int
 main ()
 {
 SemanticExtraction sen;

 sen.print ();
 std::vector<Eigen::Vector3f> ph;
 std::vector<std::vector<Eigen::Vector3f> > vv3;

 Eigen::Vector3f p;
 p (0) = 1;
 p (1) = 2;
 p (2) = 3;

 cout << "............................" << endl;

 Eigen::Vector3f q;
 q (1) = 100;
 q (2) = 200;
 q (0) = 300;

 for (int k = 0; k < 2; k++)
 ph.push_back (p);
 ph.push_back (q);

 vv3.push_back (ph);
 vv3.push_back (ph);
 cout << " vv3 size = " << vv3.size () << endl;
 cout << " v3 size = " << ph.size () << endl;
 for (vector<vector<Eigen::Vector3f> >::size_type u = 0; u < vv3.size (); u++)
 {
 cout << u << "----" << endl;
 for (vector<Eigen::Vector3f>::size_type v = 0; v < ph.size (); v++)
 {
 cout << "\t++" << v << endl;
 for (int w = 0; w < 3; w++)
 {
 cout << "\t\t%%";
 cout << vv3[u][v][w];
 cout << endl;
 }
 }
 }

 return 0;
 }
 */
