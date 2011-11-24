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

#include "cob_3d_mapping_semantics/semantic_extraction.h"
//#include <vector.h>
using namespace std;

void
SemanticExtraction::print()
{
  std::cout<<" Entered common folder "<<std::endl;
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
