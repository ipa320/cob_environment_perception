/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
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
/*
 * test_segmentation.cpp
 *
 *  Created on: 18.06.2012
 *      Author: josh
 */


//includes needed for testing
#include <gtest/gtest.h>


//includes needed for segmentation (things to test)
#include <cob_3d_segmentation/polygon_extraction/polygon_types.h>
#include <cob_3d_segmentation/polygon_extraction/polygon_extraction.h>



TEST(cob_3d_segmentation, polygon_extraction)
{
  cob_3d_segmentation::PolygonExtraction pe;

  std::vector<cob_3d_segmentation::PolygonPoint> outs;
  for(int i=0; i<30; i++)
  {
    cob_3d_segmentation::PolygonPoint xy;

    xy.x=i+10;
    xy.y=10;
    outs.push_back(xy);

    xy.x=11+i;
    xy.y=40;
    outs.push_back(xy);

    xy.x=10;
    xy.y=11+i;
    outs.push_back(xy);

    xy.x=40;
    xy.y=10+i;
    outs.push_back(xy);
  }

  cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint> poly;
  pe.outline(60,60, outs, poly);

  printf("%d polygons first with %d points\n", (int)poly.polys_.size(), (int)poly.polys_[0].size());
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
