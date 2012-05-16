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
 * ROS stack name: cob_vision
 * ROS package name: registration
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-jh
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Nov 28, 2011
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
//additional includes
#include <pcl/io/pcd_io.h>
#ifdef GICP_ENABLE
#include <pcl/io/ply_io.h>
#endif
#include <pcl/point_types.h>
#include <sys/stat.h>
   #include <ros/assert.h>
   #include <ros/console.h>
#include <iostream>
#include <fstream>

int main(int argc, char **argv) {
  if(argc<4) {
    std::cerr<<"please specify *.img *.img -> *.pcd\n";
    return 1;
  }

  sensor_msgs::Image img,depth;

  {
    FILE *fp = fopen(argv[2], "rb");
    if(!fp) {
      ROS_ERROR("couldn't open image file");
      return 0;
    }

    struct stat filestatus;
    stat(argv[2], &filestatus );

    uint8_t *up = new uint8_t[filestatus.st_size];
    fread(up,filestatus.st_size,1,fp);
//    img.deserialize(up);
    delete up;

    fclose(fp);
  }



  {
    FILE *fp = fopen(argv[1], "rb");
    if(!fp) {
      ROS_ERROR("couldn't open depth file");
      return 0;
    }

    struct stat filestatus;
    stat(argv[2], &filestatus );

    uint8_t *up = new uint8_t[filestatus.st_size];
    fread(up,filestatus.st_size,1,fp);
//    depth.deserialize(up);
    delete up;

    fclose(fp);
  }

  pcl::PointCloud<pcl::PointXYZRGB> pc;

  float focalLength = 525.0;
  float centerX = 319.5;
  float centerY = 239.5;
  float scalingFactor = 5000.0;

  depth.step/=depth.width;
  img.step/=img.width;

  ROS_ASSERT(depth.width==img.width);
  ROS_ASSERT(depth.height==img.height);

  for(int y=0; y<img.height; y++) {
    for(int x=0; x<img.width; x++) {
      pcl::PointXYZRGB p;

      p.z = *(uint16_t*)&depth.data[ (x+y*depth.width)*depth.step + 0] / scalingFactor;
      p.x = (x - centerX) * p.z / focalLength;
      p.y = (y - centerY) * p.z / focalLength;

      if(p.z==0) {
        p.z=p.y=p.x=std::numeric_limits<float>::quiet_NaN();
      }

      p.b = img.data[ (x+y*img.width)*img.step + 0];
      p.g = img.data[ (x+y*img.width)*img.step + 1];
      p.r = img.data[ (x+y*img.width)*img.step + 2];

      pc.points.push_back(p);
    }
  }

  pc.width  = img.width;
  pc.height = img.height;

  pcl::io::savePCDFileASCII(argv[3],pc);

  return 0;
}
