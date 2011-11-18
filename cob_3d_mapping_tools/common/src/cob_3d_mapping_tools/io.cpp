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
 * ROS package name: cob_3d_mapping_tools
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
 * ToDo:
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

#include <cob_3d_mapping_tools/io.h>
#include <cob_3d_mapping_common/label_defines.h>

#include <iostream>
#include <fstream>

using namespace std;

int cob_3d_mapping_tools::PPMReader::mapRGB(const string &file_name,
				       pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
  ifstream ppmFile;
  ppmFile.open(file_name.c_str());
  if(!ppmFile.is_open())
  {
    cout << "Could not open \"" << file_name << "\"" << endl;
    return (-1);
  } 

  try
  {
    size_t width, height;
    size_t word = 0;
    char c;
    string value("");

    // Read header:
    while (word < 4)
    {
      c = ppmFile.get();
      if (!isspace(c))
      { // Store char as value
	if (c == '#')
	{
	  cout << "Comment: ";
	  getline(ppmFile, value);
	  cout << value << endl;
	  value = "";
	}
	else
	  value += c;
      }
      else
      { //process last value
	word++;
	if (word == 2) width = atoi(value.c_str());
	else if(word == 3) height = atoi(value.c_str());
	value = "";
      }
    }
    
    // Check size
    if (width != cloud.width || height != cloud.height)
    {
      cout << "Size of PPM file (" 
	   << width << " x " << height 
	   << ") does not match the size of point cloud (" 
	   << cloud.width << " x " << cloud.height << ")" << endl;
      ppmFile.close();
      return(-1);
    }

    //Read colors
    word = 0;
    size_t point = 0;
    uint8_t col[3];
    uint32_t rgb;
    while (!ppmFile.eof())
    {
      value.clear();
      ppmFile >> value;
      col[word] = atoi(value.c_str());
      word++;
      if (word == 3)
      {
	rgb = ((uint32_t)col[0] << 16 | (uint32_t)col[1] << 8 | (uint32_t) col[2]);
	if (rgb != LBL_PLANE && rgb != LBL_EDGE && rgb != LBL_CYL && rgb != LBL_SPH)
	  rgb = LBL_UNDEF;
	cloud.points[point].rgba = rgb;
	word = 0;
	point++;
      }
    }
  }
  catch(int e)
  {
    ppmFile.close();
    throw e;
  }
  return 0;
}

int cob_3d_mapping_tools::PPMWriter::writeRGB(const string &file_name, 
					 const pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
  ofstream ppmFile;
  ppmFile.open(file_name.c_str());
  if(!ppmFile.is_open()) 
  {
    cout << "Could not create \"" << file_name << "\"" << endl;
    return (-1);
  }

  try 
  {
    // Write header:
    ppmFile << "P3\n"
	    << "# Generated from PCD file\n"
	    << cloud.width << " " << cloud.height << "\n"
	    << "255\n";
    
    for (size_t i=0; i < cloud.points.size(); i++)
    {
      ppmFile << (int)cloud.points[i].r << " "
	      << (int)cloud.points[i].g << " "
	      << (int)cloud.points[i].b << "\n";
    }
    ppmFile.close();
  }
  catch(int e)
  {
    ppmFile.close();
    throw e;
  }
  return 0;
}
	 
int cob_3d_mapping_tools::PPMWriter::writeDepth(const string &file_name, 
					   const pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
  ofstream ppmFile;
  ppmFile.open(file_name.c_str());
  if(!ppmFile.is_open()) 
  {
    cout << "Could not create \"" << file_name << "\"" << endl;
    return (-1);
  }

  try
  {
    // Write header:
    ppmFile << "P3\n"
	    << "# Generated from PCD file\n"
	    << cloud.width << " " << cloud.height << "\n"
	    << "255\n";

    for (size_t i = 0; i < cloud.points.size(); i++)
    {
      if (!pcl::hasValidXYZ(cloud.points[i]))
	continue;

      if(!fixed_max_) 
	max_z_ = max (cloud.points[i].z, max_z_);
      if(!fixed_min_) 
	min_z_ = min (cloud.points[i].z, min_z_);
    }
    cout << "Max_z = " << max_z_ << " | Min_z = " << min_z_ << endl;
    double grd_position;
    uint8_t rgb[3];
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
      if (pcl::hasValidXYZ(cloud.points[i]))
      {
	grd_position = (cloud.points[i].z - min_z_) / (max_z_ - min_z_);
	cob_3d_mapping_tools::getGradientColor(grd_position, rgb);
	ppmFile << (int)rgb[0] << " " << (int)rgb[1] << " " << (int)rgb[2] << "\n" ;
      }
      else 
      {
	ppmFile << "255 255 255\n";
      }
      
    }
    ppmFile.close();
  }
  catch(int e)
  {
    ppmFile.close();
    throw e;
  }
  return 0;
}

void cob_3d_mapping_tools::PPMWriter::setMaxZ (const float &max)
{
  fixed_max_ = true;
  max_z_ = max;
}

void cob_3d_mapping_tools::PPMWriter::setMinZ (const float &min)
{
  fixed_min_ = true;
  min_z_ = min;
}


// color is proportional to position 
// position  <0;1> 
// position means position of color in color gradient
uint32_t cob_3d_mapping_tools::getGradientColor(double position, uint8_t rgb[])
{
//  if (position > 1) position = position - int(position);
  if (position > 1) position = 1;
  if (position < 0) position = 0;
  // if position > 1 then we have repetition of colors
  // it maybe useful
  int n_bars_max = 4;
  double m=n_bars_max * position;
  int n=int(m); // integer of m
  double f=m-n;  // fraction of m
  uint8_t t=int(f*255);

  switch (n)
  {
  case 0: 
    rgb[0] = 255;
    rgb[1] = t;
    rgb[2] = 0;
    break;
  case 1: 
    rgb[0] = 255 - t;
    rgb[1] = 255;
    rgb[2] = 0;
    break;
  case 2: 
    rgb[0] = 0;
    rgb[1] = 255;
    rgb[2] = t;
    break;
  case 3: 
    rgb[0] = 0;
    rgb[1] = 255 - t;
    rgb[2] = 255;
    break;
  case 4: 
    rgb[0] = t;
    rgb[1] = 0;
    rgb[2] = 255;
    break;
  case 5: 
    rgb[0] = 255;
    rgb[1] = 0;
    rgb[2] = 255 - t;
    break;
  };
  return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
};
