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
//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "pcl/io/pcd_io.h"
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <cob_env_model/features/range_image_border_extractor.h>
#include <pcl/range_image/range_image.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

//#include <cob_vision_features/SURFDetector.h>
//#include <cob_vision_features/AbstractFeatureVector.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointType;

class NormTime
{
public:
  // Constructor
  NormTime()
  {
    /// void
  }

  // Destructor
  ~NormTime()
  {
    /// void
  }

  /*
   * ausgabe darf nur x , y oder z sein
   */
  void calcDeviation(pcl::PointCloud<pcl::Normal> cloud_normals , double target , char cord_char)

  {
    int cord=0;
    int counter=0;
    double value_sum=0.0;

    switch (cord_char){
      case 'x': cord=0;
      break;
      case 'y': cord=1;
      break;
      case 'z': cord=2;
      break;
    }
    for (int i=0 ; i< cloud_normals.size();i++)
    {
      if (0.8*target< cloud_normals.points[i].normal[cord] && cloud_normals.points[i].normal[cord]< 1.2*target || -0.8*target> cloud_normals.points[i].normal[cord] && cloud_normals.points[i].normal[cord] > -1.2*target)
      {
        counter++;
        if ( cloud_normals.points[i].normal[cord] < 0)
          value_sum=value_sum - cloud_normals.points[i].normal[cord];
        else
          value_sum=value_sum + cloud_normals.points[i].normal[cord];

      }
    }
    if (counter>0){
      ROS_INFO_STREAM("Average of " << cord_char << " is " <<  (value_sum/counter));
      ROS_INFO_STREAM("Difference of " << cord_char << " is " <<  (value_sum/counter)/target );
    }
  }
  void getPointCloud (PointCloud &cloud)
  {
    cloud.height=480;
    cloud.width = 640;
    cloud.points.resize (cloud.width * cloud.height);
    double  x_val=-1 , y_val=-1 , z_val=3;
    for (int i=0; i<cloud.height;i++){
      for (int j=0 ; j<cloud.width ;j++)
      {
        if (x_val<3)
        {
          cloud.points[i*cloud.width+j].x=x_val;
          cloud.points[i*cloud.width+j].y=y_val;
          cloud.points[i*cloud.width+j].z=z_val;
          x_val += 0.01;
        }
        else  {
          cloud.points[i*cloud.width+j].x=x_val;
          cloud.points[i*cloud.width+j].y=y_val;
          cloud.points[i*cloud.width+j].z=z_val;
          z_val -= 0.01;
        }
      }
      x_val=-1;
      z_val=3;
      y_val+=0.01;
    }


  }

  void extractRangeImage (PointCloud &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals)
  {
    pcl::RangeImage range_image;
    range_image.header = cloud.header;
    range_image.width = cloud.width;
    range_image.height = cloud.height;
    range_image.is_dense = false;
    for(int i = 0; i<cloud.size(); i++)
    {
      pcl::PointWithRange p;

      {
        p.x = cloud.points[i].x;
        p.y = cloud.points[i].y;
        p.z = cloud.points[i].z;
        p.range = sqrt(cloud.points[i].x*cloud.points[i].x+cloud.points[i].y*cloud.points[i].y+cloud.points[i].z*cloud.points[i].z);
      }
      range_image.points.push_back(p);
    }

    ipa_features::RangeImageBorderExtractor border_extractor(&range_image);
    boost::timer ti;

    ipa_features::RangeImageBorderExtractor::LocalSurface **ls = border_extractor.getSurfaceStructure();
    double time = ti.elapsed();

    cloud_normals.resize(range_image.size());
    for (int i=0;i<range_image.size();i++)
    {
      cloud_normals.points[i].normal[0]=range_image.points[i].x;
      cloud_normals.points[i].normal[1]=range_image.points[i].y;
      cloud_normals.points[i].normal[2]=range_image.points[i].z;

    }
    //normalization
    for (int i=0;i<cloud_normals.size();i++)
    {
      double length=sqrt(cloud_normals.points[i].normal[0]*cloud_normals.points[i].normal[0]+cloud_normals.points[i].normal[1]*cloud_normals.points[i].normal[1]+cloud_normals.points[i].normal[2]*cloud_normals.points[i].normal[2]);
      cloud_normals.points[i].normal[0]=cloud_normals.points[i].normal[0]/length;
      cloud_normals.points[i].normal[1]=cloud_normals.points[i].normal[1]/length;
      cloud_normals.points[i].normal[2]=cloud_normals.points[i].normal[2]/length;
    }
    double all=0;
    int counter=0;
    for (int i=0;i<cloud_normals.size();i++)
    {
      if (!isnan(cloud_normals.points[i].normal[0])){
        double length=sqrt(cloud_normals.points[i].normal[0]*cloud_normals.points[i].normal[0]+cloud_normals.points[i].normal[1]*cloud_normals.points[i].normal[1]+cloud_normals.points[i].normal[2]*cloud_normals.points[i].normal[2]);
        all +=length;
        counter++;
      }
    }
    ROS_INFO_STREAM("Range image time " <<time <<" average normal length " << all/counter);
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n;
    pcl::concatenateFields (cloud, cloud_normals, cloud_n);

    pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/extractrangenormals.pcd", cloud_n);

  }




  void surfaceNormalsFast(PointCloud &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals)
  {



    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud.makeShared());

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    //pcl::PointCloud<pcl::Normal> cloud_normals ;

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    boost::timer ti;

    ne.compute (cloud_normals);
    double time = ti.elapsed();
    double all=0;
    int counter=0;
    for (int i=0;i<cloud_normals.size();i++)
    {
      if (!isnan(cloud_normals.points[i].normal[0])){
        double length=sqrt(cloud_normals.points[i].normal[0]*cloud_normals.points[i].normal[0]+cloud_normals.points[i].normal[1]*cloud_normals.points[i].normal[1]+cloud_normals.points[i].normal[2]*cloud_normals.points[i].normal[2]);
        all +=length;
        counter++;
      }
    }
    ROS_INFO_STREAM("surface fast time " <<time <<" average normal length " << all/counter);


  }

  void   IntergralImage(PointCloud &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals){
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;


    ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud.makeShared());
    boost::timer ti;
    ne.compute(cloud_normals);

    double time = ti.elapsed();
    for (int i=0;i<cloud_normals.size();i++)
    {
      double length=sqrt(cloud_normals.points[i].normal[0]*cloud_normals.points[i].normal[0]+cloud_normals.points[i].normal[1]*cloud_normals.points[i].normal[1]+cloud_normals.points[i].normal[2]*cloud_normals.points[i].normal[2]);
      cloud_normals.points[i].normal[0]=cloud_normals.points[i].normal[0]/length;
      cloud_normals.points[i].normal[1]=cloud_normals.points[i].normal[1]/length;
      cloud_normals.points[i].normal[2]=cloud_normals.points[i].normal[2]/length;
    }
    double all=0;
    int counter=0;
    for (int i=0;i<cloud_normals.size();i++)
    {
      if (!isnan(cloud_normals.points[i].normal[0])){
        double length=sqrt(cloud_normals.points[i].normal[0]*cloud_normals.points[i].normal[0]+cloud_normals.points[i].normal[1]*cloud_normals.points[i].normal[1]+cloud_normals.points[i].normal[2]*cloud_normals.points[i].normal[2]);
        all +=length;
        counter++;
      }

    }
    ROS_INFO_STREAM("Integrate time " <<time << " average normal length " << all/counter );

  }
  void surfaceNormals(PointCloud &cloud,pcl::PointCloud<pcl::Normal> &cloud_normals)
  {



    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud.makeShared());

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    //pcl::PointCloud<pcl::Normal> cloud_normals ;

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    boost::timer ti;

    // Compute the features
    ne.compute (cloud_normals);
    double time = ti.elapsed();
    /* pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n;
 pcl::concatenateFields (cloud, cloud_normals, cloud_n);

 pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/surfaceNormals.pcd", cloud_n);*/
    double all=0;
    int counter=0;
    for (int i=0;i<cloud_normals.size();i++)
    {
      if (!isnan(cloud_normals.points[i].normal[0])){
        double length=sqrt(cloud_normals.points[i].normal[0]*cloud_normals.points[i].normal[0]+cloud_normals.points[i].normal[1]*cloud_normals.points[i].normal[1]+cloud_normals.points[i].normal[2]*cloud_normals.points[i].normal[2]);
        all +=length;
        counter++;
      }
    }
    ROS_INFO_STREAM("surface time " <<time << " average normal length " << all/counter);


  }

};
int main(int argc, char** argv)
{
  // right wall
  // x = 0,906307787
  // z= 0,422618262

  //front wall
  //x=0,422618262
  //z=0,906307787

  bool kinect_cloud=true;

  std::string directory("/home/goa-hh/pcl_daten/");
  PointCloud cloud_in;
  if(kinect_cloud){
    ROS_INFO("use kinect cloud");
    pcl::io::loadPCDFile(directory+"simple_planes.pcd", cloud_in);}


  NormTime nt;
  if(!kinect_cloud){
    ROS_INFO("Use simulation cloud");
    nt.getPointCloud(cloud_in);}
  pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/cloud_in.pcd", cloud_in);
  pcl::PointCloud<pcl::Normal> cloud_normals_surface;
  pcl::PointCloud<pcl::Normal> cloud_normals_surface_fast;
  pcl::PointCloud<pcl::Normal> cloud_normals_integral;
  pcl::PointCloud<pcl::Normal> cloud_normals_range;


  nt.surfaceNormals(cloud_in,cloud_normals_surface);
  nt.surfaceNormalsFast(cloud_in,cloud_normals_surface_fast);
  nt.IntergralImage(cloud_in,cloud_normals_integral);
  nt.extractRangeImage(cloud_in,cloud_normals_range);


  // kinect cloud
  if(kinect_cloud){
    ROS_INFO("Surface  right wall");
    nt.calcDeviation(cloud_normals_surface , 0.906307787 ,'x');
    nt.calcDeviation(cloud_normals_surface , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_surface , 0.422618262 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Surface  front wall");
    nt.calcDeviation(cloud_normals_surface , 0.422618262 ,'x');
    nt.calcDeviation(cloud_normals_surface , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_surface , 0.906307787 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Surface fast right wall");
    nt.calcDeviation(cloud_normals_surface_fast , 0.906307787 ,'x');
    nt.calcDeviation(cloud_normals_surface_fast , 0 ,'y');
    nt.calcDeviation(cloud_normals_surface_fast , 0.422618262 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Surface fast front wall");
    nt.calcDeviation(cloud_normals_surface_fast , 0.422618262 ,'x');
    nt.calcDeviation(cloud_normals_surface_fast , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_surface_fast , 0.906307787 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Integral fast right wall");
    nt.calcDeviation(cloud_normals_integral , 0.906307787 ,'x');
    nt.calcDeviation(cloud_normals_integral , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_integral , 0.422618262 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Integral fast front wall");
    nt.calcDeviation(cloud_normals_integral , 0.422618262 ,'x');
    nt.calcDeviation(cloud_normals_integral , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_integral , 0.906307787 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Range  right wall");
    nt.calcDeviation(cloud_normals_range , 0.906307787 ,'x');
    nt.calcDeviation(cloud_normals_range , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_range , 0.422618262 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Range  front wall");
    nt.calcDeviation(cloud_normals_range , 0.422618262 ,'x');
    nt.calcDeviation(cloud_normals_range , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_range , 0.906307787 ,'z');
  }
  //simulation cloud
  if(!kinect_cloud){
    ROS_INFO("Surface  right wall");
    nt.calcDeviation(cloud_normals_surface , 1.0 ,'x');
    nt.calcDeviation(cloud_normals_surface , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_surface , 0.0 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Surface  front wall");
    nt.calcDeviation(cloud_normals_surface , 0.0 ,'x');
    nt.calcDeviation(cloud_normals_surface , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_surface , 1.0 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Surface fast right wall");
    nt.calcDeviation(cloud_normals_surface_fast , 1.0 ,'x');
    nt.calcDeviation(cloud_normals_surface_fast , 0 ,'y');
    nt.calcDeviation(cloud_normals_surface_fast , 0.0 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Surface fast front wall");
    nt.calcDeviation(cloud_normals_surface_fast , 0.0 ,'x');
    nt.calcDeviation(cloud_normals_surface_fast , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_surface_fast , 1.0 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Integral fast right wall");
    nt.calcDeviation(cloud_normals_integral , 1.0 ,'x');
    nt.calcDeviation(cloud_normals_integral , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_integral , 0.0 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Integral fast front wall");
    nt.calcDeviation(cloud_normals_integral , 0.0 ,'x');
    nt.calcDeviation(cloud_normals_integral , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_integral , 1.0 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Range  right wall");
    nt.calcDeviation(cloud_normals_range , 1.0 ,'x');
    nt.calcDeviation(cloud_normals_range , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_range , 0.0 ,'z');
    ROS_INFO("--------------------------------------");
    ROS_INFO("Range  front wall");
    nt.calcDeviation(cloud_normals_range , 0.0 ,'x');
    nt.calcDeviation(cloud_normals_range , 0.0 ,'y');
    nt.calcDeviation(cloud_normals_range , 1.0 ,'z');
  }

  return 0;
}
