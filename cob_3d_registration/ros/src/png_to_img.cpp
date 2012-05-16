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
 * ROS package name: cob_3d_mapping_tools-RelWithDebInfo@cob_3d_mapping_tools
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

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>
#include <iostream>
#include <fstream>

int main(int argc, char **argv) {
  if(argc<3) {
    std::cerr<<"please specify *.png -> *.img\n";
    return 1;
  }

  cv_bridge::CvImage tmp;
  tmp.encoding="rgb8";
  tmp.image = cv::imread(argv[1],-1);



  sensor_msgs::ImageConstPtr last_img_ = tmp.toImageMsg();

std::ofstream img_stream;

img_stream << *last_img_;

img_stream.close();


//  //serialize image
//  FILE *fp = fopen(argv[2],"wb");
//  if(fp)
//  {
//    uint32_t len = last_img_->serializationLength();
//
//    uint8_t *wptr = new uint8_t[len];
//    last_img_->serialize(wptr,0);
//    fwrite(wptr, 1, len, fp);
//    delete [] wptr;
//
//    fclose(fp);
//  }
//  else {
//    ROS_ERROR("couldn't open %s", argv[2]);
//    return 1;
//  }

  return 0;
}
