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

/// binary mapping from $\delta x, \delta y$ to binary pattern
namespace Contour2D {

  struct spline2D {
    int v, x,y, bf;
    bool multiple;
    spline2D():v(0),x(0),y(0),bf(0), multiple(false) {}
  };

  /// global spline pattern (computed once, 8 for each direction + 1 for start)
  spline2D g_Splines[9][256];

  int SplineMap[]={
                   4, 5,  6,
                   3, -1, 7,
                   2, 1,  0
  };

  void generateSpline2D() {
    int mapX[]={2,1,0, 0,0, 1, 2,2};
    int mapY[]={2,2,2, 1, 0,0,0, 1};

    for(int i=0; i<256; i++) {
      bool b[8];
      int num = 0;
      for(int j=0; j<8; j++) {
        b[j]=i&(1<<j);
        if(b[j]) ++num;
      }
      spline2D s;
      s.multiple = (num>2);
      if(!i) {
        g_Splines[8][i]=s;
        continue;
      }
      int a=-1;
      for(int j=3; j>=0; j--) {
        if(b[7-j]) {
          a=7-j;
          break;}
      }
      for(int j=0; a==-1&&j<4; j++) {
        if(b[j]) {
          a=j;
          break;}
      }
      for(int j=3; j>=0; j--) {
        if(b[(a+4+j+8)%8]) {
          s.v=0;
          s.x=mapX[(a+4+j+8)%8]-1;
          s.y=mapY[(a+4+j+8)%8]-1;
          s.bf=(a+j+8)%8;
          a=-1;
          break;
        }
      }
      for(int j=0; a!=-1&&j<4; j++) {
        if(b[(a+4-j+8)%8]) {
          s.v=0;
          s.x=mapX[(a+4-j+8)%8]-1;
          s.y=mapY[(a+4-j+8)%8]-1;
          s.bf=(a-j+8)%8;
          break;
        }
      }
      g_Splines[8][i]=s;
    }

    for(int a=0; a<8; a++) {

      for(int i=0; i<256; i++) {
        bool b[8];
        int num = 0;
        for(int j=0; j<8; j++) {
          b[j]=i&(1<<j);
          if(b[j]) ++num;
        }
        spline2D s;
        s.multiple = (num>1);
        if(!i) {
          g_Splines[a][i]=s;
          continue;
        }
        bool found=false;
        for(int j=3; j>=0; j--) {
          if(b[(a+4+j+8)%8]) {
            s.v=-j;
            s.x=mapX[(a+4+j+8)%8]-1;
            s.y=mapY[(a+4+j+8)%8]-1;
            s.bf=(a+j+8)%8;
            found=true;
            break;
          }
        }
        for(int j=0; !found&&j<4; j++) {
          if(b[(a+4-j+8)%8]) {
            s.v=j;
            s.x=mapX[(a+4-j+8)%8]-1;
            s.y=mapY[(a+4-j+8)%8]-1;
            s.bf=(a-j+8)%8;
            break;
          }
        }
        g_Splines[a][i]=s;
      }
    }
  }
}
