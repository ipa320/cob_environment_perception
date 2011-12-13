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
  //float norm = sqrt(pow(p_ptr->normal (0), 2) + pow(p_ptr->normal (1), 2) + pow(p_ptr->normal (2), 2));

  /*
   std::cout<<"\t%Nx: "<<p_ptr->normal(0)<<"----"<<norm<<std::endl;
   std::cout<<"\t%Ny: "<<p_ptr->normal(1)<<std::endl;
   std::cout<<"\t%Nz: "<<p_ptr->normal(2)<<std::endl;
   */
  // TODO: define as parameters (declare member variable and set it from the node using ROS parameters)
  // TODO: better use && instead of three if-conditions
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

  else
  {
    return false;
  }
}

bool
SemanticExtraction::isHeightOk (PolygonPtr p_ptr)
{
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
}

bool
SemanticExtraction::isSizeOk (PolygonPtr p_ptr)
{

  float xi, xi_1, yi, yi_1;
  for (unsigned int i = 0; i < p_ptr->poly_points.size (); i++)
  {
    //std::cout << " p_ptr->poly_points[i].size----:" << p_ptr->poly_points.size () << std::endl;
    float sum = 0;
    float area = 0;
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
    area = abs (sum / 0.5);

    std::cout << "\n\t*** Area = "<< area<< std::endl;

    //TODO: parameters
    if (area >= 1 && area <= 3)
      return true;
    else if(area<1)
    {
      std::cout << " Size is small "<< area<< std::endl;
      return false;
    }
    //TODO: really 31?
    else if(area>31)
    {
      std::cout << " Size is large "<< area<< std::endl;
      return false;
    }
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
