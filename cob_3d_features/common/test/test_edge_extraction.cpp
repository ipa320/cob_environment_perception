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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: cob_env_model
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 01/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 * Only update if new robot pose available
 * Resample point cloud
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


//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
//#include <tf_conversions/tf_kdl.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv.h>
#include <highgui.h>
#include "pcl/io/pcd_io.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/normal_3d.h"
#include <pcl/features/principal_curvatures.h>
//#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PointIndices.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/range_image/range_image.h>
#include <pcl/features/integral_image_normal.h>
#include <cob_3d_features/edge_estimation_3d.h>
#include <cob_3d_features/range_image_border_extractor.h>
#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_mapping_features/segmentation.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

//#include <cob_vision_features/SURFDetector.h>
//#include <cob_vision_features/AbstractFeatureVector.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;
typedef pcl::PointXYZRGB PointT;
typedef cob_3d_features::Coords Coords;

/*
struct Coords
{
  int u;
  int v;

  Coords(int u_in, int v_in)
  {
    u = u_in;
    v=  v_in;
  }
};
*/


//####################
//#### node class ####
class ExtractFeatures
{
public:
  // Constructor
  ExtractFeatures()
  {
    /// void
  }

  // Destructor
  ~ExtractFeatures()
  {
    /// void
  }

  /**Estimates the point normals for cloud and returns cloud_n*/
  void estimatePointNormals(PointCloudT& cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_n)
  {

    boost::timer t;
    // Estimate normals
    /*pcl::IntegralImageNormalEstimation<PointT,pcl::Normal> ne;

                   pcl::PointCloud<pcl::Normal> cloud_normal;

                   ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
                   //ne.setMaxDepthChangeFactor(0.02f);
                   ne.setRectSize(10,10);
                   ne.setNormalSmoothingSize(10.0f);
                   ne.setInputCloud(cloud.makeShared());
                   ne.compute(cloud_normal);*/

    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //pcl::PCDReader reader;
    //reader.read ("/home/goa/pcl_daten/table/icp_fov/pc_aligned_1.pcd", cloud);
    //std::cout << "Input cloud has " << cloud.size() << " data points" << std::endl;

    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud.makeShared());
    //pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
    pcl::OrganizedDataIndex<pcl::PointXYZRGB>::Ptr tree (new pcl::OrganizedDataIndex<pcl::PointXYZRGB> ());
    normalEstimator.setSearchMethod(tree);
    //normalEstimator.setKSearch(50);
    normalEstimator.setRadiusSearch(0.05);
    //normalEstimator.setNumberOfThreads(8);
    pcl::PointCloud<pcl::Normal> cloud_normal;
    normalEstimator.compute(cloud_normal);

    ROS_INFO("Time elapsed for normal estimation: %f", t.elapsed());
    pcl::concatenateFields (cloud, cloud_normal, cloud_n);
    return;
  }
  void integralEstimatePointNormals(PointCloudT& cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_n)
  {

    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;


    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.2f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setDepthDependentSmoothing(true);
    ne.setInputCloud(cloud.makeShared());
    boost::timer ti;
    pcl::PointCloud<pcl::Normal> cloud_normal;

    ne.compute(cloud_normal);

    double time = ti.elapsed();
    /*for (int i=0;i<cloud_normal.size();i++)
    {
      double length=sqrt(cloud_normal.points[i].normal[0]*cloud_normal.points[i].normal[0]+cloud_normal.points[i].normal[1]*cloud_normal.points[i].normal[1]+cloud_normal.points[i].normal[2]*cloud_normal.points[i].normal[2]);
      cloud_normal.points[i].normal[0]=cloud_normal.points[i].normal[0]/length;
      cloud_normal.points[i].normal[1]=cloud_normal.points[i].normal[1]/length;
      cloud_normal.points[i].normal[2]=cloud_normal.points[i].normal[2]/length;
    }
    double all=0;
    int counter=0;
    for (int i=0;i<cloud_normal.size();i++)
    {
      if (!isnan(cloud_normal.points[i].normal[0])){
        double length=sqrt(cloud_normal.points[i].normal[0]*cloud_normal.points[i].normal[0]+cloud_normal.points[i].normal[1]*cloud_normal.points[i].normal[1]+cloud_normal.points[i].normal[2]*cloud_normal.points[i].normal[2]);
        all +=length;
        counter++;
      }

    }*/
    pcl::concatenateFields (cloud, cloud_normal, cloud_n);

    ROS_INFO("Integrate time %f",time);
  }

  /**Stores the color field of a PointCloud to cv::Mat*/
  void getColorImage(PointCloudT::Ptr& pc, cv::Mat& color_image)
  {
    unsigned char* c_ptr = 0;
    int pc_pt_idx=0;
    for (int row = 0; row < color_image.rows; row++)
    {
      c_ptr = color_image.ptr<unsigned char>(row);
      for (int col = 0; col < color_image.cols; col++, pc_pt_idx++)
      {
        memcpy(&c_ptr[3*col], &pc->points[pc_pt_idx].rgb, 3*sizeof(unsigned char));
      }
    }
  }

  /**Stores the color field of a PointCloud to cv::Mat*/
  void getRangeImage(PointCloudT::Ptr& pc, cv::Mat& range_image)
  {
    float* c_ptr = 0;
    int pc_pt_idx=0;
    for (int row = 0; row < range_image.rows; row++)
    {
      c_ptr = range_image.ptr<float>(row);
      for (int col = 0; col < range_image.cols; col++, pc_pt_idx++)
      {
        memcpy(&c_ptr[col], &pc->points[pc_pt_idx].z, sizeof(float));
      }
    }
  }

  /**Extracts edges from a color image using the canny algorithm*/
  void extractEdgesCanny(cv::Mat& color_image, cv::Mat& canny_image)
  {
    boost::timer t;
    cv::Mat grey_image;
    cvtColor( color_image, grey_image, CV_BGR2GRAY );
    cv::Mat color_canny_image;
    Canny( grey_image, canny_image, 50, 150, 3 );
    ROS_INFO("Time elapsed for canny edge: %f", t.elapsed());
  }

  /**Extracts edges from a color image using the sobel filter*/
  void extractEdgesSobel(cv::Mat& color_image, cv::Mat& sobel_image, int x, int y)
  {
    boost::timer t;
    /*cv::Mat grey_image;
    cvtColor( color_image, grey_image, CV_RGB2GRAY );
    cv::imshow("grey image", grey_image);*/
    cv::Mat s_image;
    cv::Sobel(color_image, s_image, CV_32FC1, x, y, 3);
    cv::pow(s_image,2,sobel_image);
    ROS_INFO("Time elapsed for sobel: %f", t.elapsed());
  }

  /**Extracts edges from a color image using the sobel filter*/
  void extractEdgesLaPlace(cv::Mat& color_image, cv::Mat& laplace_image)
  {
    boost::timer t;
    //cv::Mat grey_image;
    //cvtColor( color_image, grey_image, CV_RGB2GRAY );
    cv::Laplacian(color_image, laplace_image, CV_32FC1, 1);
    ROS_INFO("Time elapsed for la place: %f", t.elapsed());
  }

  /**Uses the PCL BoundaryEstimation to find edges in a point cloud. Uses an angle criterion to detect edges
   * Unfortunately marks the outline of a point cloud as edge; needs normal estimation
   */
  void extractEdges3D(PointCloudT::Ptr& cloud_in, pcl::PointCloud<pcl::InterestPoint>& cloud_out, cv::Mat& border_image)
  {

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    //pcl::PointCloud<pcl::Boundary>::Ptr edge_estimation_pts (new pcl::PointCloud<pcl::Boundary> ());
    integralEstimatePointNormals(*cloud_in, *cloud_n);

    // estimatePointNormals(*cloud_in, *cloud_n);
    boost::timer t;

    //pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree;
    pcl::OrganizedDataIndex<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::OrganizedDataIndex<pcl::PointXYZRGBNormal> ());
    //tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> > ();
    cob_3d_features::EdgeEstimation3D<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal,pcl::InterestPoint> edge_estimation;
    edge_estimation.setSearchMethod(tree);
    edge_estimation.setInputCloud(cloud_n);
    //edge_estimation.setSearchSurface (cloud);
    edge_estimation.setRadiusSearch(0.04); //parameter not used, must be set to prevent crash
    edge_estimation.setInputNormals(cloud_n);

    edge_estimation.dist_threshold_ = 0.02; //increase to get more border points

    edge_estimation.compute(cloud_out);

    //pcl::PointCloud<pcl::PointXYZRGBNormal> edge_estimation_cloud;
    //cloud_out.points.resize(cloud_n->points.size());
    //cloud_out.header = cloud_n->header;
    //int nr_p = 0;

    //std::cout << "size: " << edge_estimation_pts->size() << std::endl;

    /*for( unsigned int i = 0; i < cloud_in->points.size(); i++)
                {
                        if( cloud_out.points[i].edge_estimation_point == 1)
                        {
                                //pc_edge.points[i].edge_estimation_point++;
                                cloud_out.points[nr_p++] = cloud_in->points[i];
                        }
                }
                cloud_out.width = nr_p;
                cloud_out.height = 1;
                cloud_out.points.resize(nr_p);
                cloud_out.is_dense = true;*/

    border_image = cv::Mat(cloud_in->height, cloud_in->width, CV_32FC1);
    int pt_idx=0;
    for(unsigned int row=0; row<border_image.rows; row++)
    {
      for(unsigned int col=0; col<border_image.cols; col++, pt_idx++)
      {
        if(cloud_out.points[pt_idx].strength < 2)
          border_image.at<float>(row,col) = cloud_out.points[pt_idx].strength;
        else
          border_image.at<float>(row,col) = 0;
        /*if( cloud_out.points[pt_idx].edge_estimation_point == 1)
          border_image.at<unsigned char>(row,col) = 255;
        else
          border_image.at<unsigned char>(row,col) = 0;*/
      }
    }
    //cv::imshow("boundary image", border_image);
    //cv::waitKey();
    ROS_INFO("Time elapsed for boundary estimation: %f", t.elapsed());
  }


  void propagateWavefront(pcl::PointCloud<PointLabel>::Ptr& cloud_in)
  {
    //TODO: unlabel small cluster, change pixel step?
    boost::timer t;
    int width = cloud_in->width, height = cloud_in->height;
    std::vector<pcl::Boundary*> wave;
    std::vector<Coords> wave_coords;

    int cur_label = 3;
    /*uint8_t region_size[256];
            for (int i=0; i<256; i++)
            {
                region_size[i] = 0;
            }*/


    for(int i = 0; i < height; i++ )
    {
      for(int j = 0; j < width; j++ )
      {
        //std::cout << i << "," << j << ":" << cur_label << std::endl;
        if(cloud_in->points[i*width+j].label == 0)
        {
          cur_label++;
          PointLabel* p = &cloud_in->points[i*width+j];
          p->label = cur_label;
          //Eigen::Vector2d uv(j,i);
          Coords c(j,i,false);

          //int count = 0;
          bool is_wave = true;
          while(is_wave)
          {
            //count++;
            int pt_ctr = c.u+c.v/*uv(0)+uv(1)*/*width;
            if( c.u/*uv(0)*/ < width-1 && cloud_in->points[pt_ctr+1].label==0)
            {
              cloud_in->points[pt_ctr+1].label = cur_label;
              //wave.push_back(&cloud_in->points[pt_ctr+1]);
              wave_coords.push_back(Coords(c.u+1,c.v,false)/*Eigen::Vector2d(uv(0)+1,uv(1))*/);
            }
            if( c.u/*uv(0)*/ > 0 && cloud_in->points[pt_ctr-1].label==0)
            {
              cloud_in->points[pt_ctr-1].label = cur_label;
              //wave.push_back(&cloud_in->points[pt_ctr-1]);
              wave_coords.push_back(Coords(c.u-1,c.v,false)/*Eigen::Vector2d(uv(0)-1,uv(1))*/);
            }
            //std::cout << pt_ctr+width << ": "<< (int)(cloud_in->points[pt_ctr+width].boundary_point) << std::endl;
            if(c.v/*uv(1)*/ < height-1 && cloud_in->points[pt_ctr+width].label==0)
            {
              cloud_in->points[pt_ctr+width].label = cur_label;
              //wave.push_back(&cloud_in->points[pt_ctr+width]);
              wave_coords.push_back(Coords(c.u,c.v+1,false)/*Eigen::Vector2d(uv(0),uv(1)+1)*/);
            }
            if(c.v/*uv(1)*/ > 0 && cloud_in->points[pt_ctr-width].label==0)
            {
              cloud_in->points[pt_ctr-width].label = cur_label;
              //wave.push_back(&cloud_in->points[pt_ctr-width]);
              wave_coords.push_back(Coords(c.u,c.v-1,false)/*Eigen::Vector2d(uv(0),uv(1)-1)*/);
            }
            //p = wave.back();
            if(wave_coords.size()>0)
            {
              //wave.pop_back();
              c = wave_coords.back();
              wave_coords.pop_back();
              //std::cout << wave_coords.size() << std::endl;
            }
            else
              is_wave = false;
          }
          /*if(count <= maxSize)
                                {
                                        region_size[cloud_in->points[i*width+j].boundary_point]=1;
                                }
                                else
                                {
                                        region_size[cloud_in->points[i*width+j].boundary_point]=2;
                                }*/
        }
        /*else
                        {
                                if(region_size[cloud_in->points[i*width+j].boundary_point]==1)
                                        cloud_in->points[i*width+j].boundary_point = 0;
                        }*/
      }

    }
    std::cout << "Time elapsed for wavefront propagation: " << t.elapsed() << std::endl;
    return;
  }


  int
  searchForNeighbors (
      pcl::PointCloud<PointLabel>::Ptr& cloud_in,
      int col, int row,
      double radius,
      std::vector<int>& indices_ul,
      std::vector<int>& indices_ur,
      std::vector<int>& indices_lr,
      std::vector<int>& indices_ll,
      bool gap_l, bool gap_r, bool gap_a, bool gap_d)
  {
    //NaN test
    //TODO: use PCL function for that
    //if(cloud_in->points[index].x != cloud_in->points[index].x) return 0;

    gap_l=gap_r=gap_a=gap_d=false;
    indices_ul.clear();
    indices_ur.clear();
    indices_lr.clear();
    indices_ll.clear();

    int idx_x = col;
    int idx_y = row;

    int gap_l_ctr = 0;
    int gap_a_ctr = 0;
    for (int i=idx_x-radius; i<=idx_x; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y-radius+1; j<=idx_y; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ul.push_back(i+j*cloud_in->width);
        if(j==idx_y && cloud_in->points[i+j*cloud_in->width].label!=1) gap_l_ctr++;
        if(i==idx_x && cloud_in->points[i+j*cloud_in->width].label!=1) gap_a_ctr++;
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    if(gap_l_ctr==0) gap_l = true;
    if(gap_a_ctr==0) gap_a = true;

    int gap_r_ctr = 0;
    int gap_d_ctr = 0;
    for (int i=idx_x; i<idx_x+radius; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y-radius; j<=idx_y; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ur.push_back(i+j*cloud_in->width);
        if(j==idx_y && cloud_in->points[i+j*cloud_in->width].label!=1) gap_r_ctr++;
        if(i==idx_x && cloud_in->points[i+j*cloud_in->width].label!=1) gap_d_ctr++;
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    if(gap_r_ctr==0) gap_r = true;
    if(gap_d_ctr==0) gap_d = true;
    for (int i=idx_x; i<idx_x+radius; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y; j<=idx_y+radius; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_lr.push_back(i+j*cloud_in->width);
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    for (int i=idx_x-radius+1; i<=idx_x; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y; j<=idx_y+radius; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ll.push_back(i+j*cloud_in->width);
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    return 1;
  }

  bool isStopperInNeighbors(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<int>& indices)
  {
    for(unsigned int i=0; i< indices.size(); i++)
    {
      if(cloud_in->points[indices[i]].label == 1)
        return true;
    }
    return false;
  }


  void propagateWavefront2(pcl::PointCloud<PointLabel>::Ptr& cloud_in)
  {
    //TODO: unlabel small cluster, change pixel step?
    boost::timer t;
    int width = cloud_in->width, height = cloud_in->height;
    std::vector<pcl::Boundary*> wave;
    std::vector<Coords> wave_coords;

    int cur_label = 3;
    int px_range = 5;
    std::vector<int> indices_ul;
    std::vector<int> indices_ur;
    std::vector<int> indices_lr;
    std::vector<int> indices_ll;

    for(int i = 0; i < height; i++ )
    {
      for(int j = 0; j < width; j++ )
      {
        //std::cout << i << "," << j << ":" << cur_label << std::endl;
        if(cloud_in->points[i*width+j].label == 0)
        {
          cur_label++;
          PointLabel* p = &cloud_in->points[i*width+j];
          p->label = cur_label;
          Coords c(j,i,false);

          //int count = 0;
          bool gap_l=false, gap_r=false, gap_a=false, gap_d=false;
          bool is_wave = true;
          while(is_wave)
          {
            //count++;
            int pt_ctr = c.u+c.v*width;
            if( c.u < width-1 && cloud_in->points[pt_ctr+1].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);
              if(c.u = 320 && c.v == 240)
              {
                std::cout << c.u << "," << c.v << " --> " << pt_ctr << std::endl;
                std::cout << "UL: ";
                for(unsigned int i=0; i<indices_ul.size(); i++)
                  std::cout << indices_ul[i] << ", ";
                std::cout << std::endl;
                std::cout << "UR: ";
                for(unsigned int i=0; i<indices_ur.size(); i++)
                  std::cout << indices_ur[i] << ", ";
                std::cout << std::endl;
                std::cout << "LR: ";
                for(unsigned int i=0; i<indices_lr.size(); i++)
                  std::cout << indices_lr[i] << ", ";
                std::cout << std::endl;
                std::cout << "LL: ";
                for(unsigned int i=0; i<indices_ll.size(); i++)
                  std::cout << indices_ll[i] << ", ";
                std::cout << std::endl;
              }
              cloud_in->points[pt_ctr+1].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ul) && isStopperInNeighbors(cloud_in, indices_ll) && gap_l))
              ///if(!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_lr))
                wave_coords.push_back(Coords(c.u+1,c.v,false));
            }
            if( c.u > 0 && cloud_in->points[pt_ctr-1].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr-1].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_lr) && gap_r))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_ul)) &&
              //    (isStopperInNeighbors(cloud_in, indices_lr) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_ul) || !isStopperInNeighbors(cloud_in, indices_ll))
                wave_coords.push_back(Coords(c.u-1,c.v,false));
            }
            //std::cout << pt_ctr+width << ": "<< (int)(cloud_in->points[pt_ctr+width].boundary_point) << std::endl;
            if(c.v < height-1 && cloud_in->points[pt_ctr+width].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr+width].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_ul) && gap_a))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_lr)) &&
              //    (isStopperInNeighbors(cloud_in, indices_ul) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_lr) || !isStopperInNeighbors(cloud_in, indices_ll))
                wave_coords.push_back(Coords(c.u,c.v+1,false));
            }
            if(c.v > 0 && cloud_in->points[pt_ctr-width].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr-width].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_lr) && isStopperInNeighbors(cloud_in, indices_ll) && gap_d))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_lr)) &&
              //    (isStopperInNeighbors(cloud_in, indices_ul) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_ul))
                wave_coords.push_back(Coords(c.u,c.v-1,false));
            }
            //p = wave.back();
            if(wave_coords.size()>0)
            {
              //wave.pop_back();
              c = wave_coords.back();
              wave_coords.pop_back();
              //std::cout << wave_coords.size() << std::endl;
            }
            else
              is_wave = false;
          }
          /*if(count <= maxSize)
                                {
                                        region_size[cloud_in->points[i*width+j].boundary_point]=1;
                                }
                                else
                                {
                                        region_size[cloud_in->points[i*width+j].boundary_point]=2;
                                }*/
        }
        /*else
                        {
                                if(region_size[cloud_in->points[i*width+j].boundary_point]==1)
                                        cloud_in->points[i*width+j].boundary_point = 0;
                        }*/
      }

    }
    std::cout << "Time elapsed for wavefront propagation: " << t.elapsed() << std::endl;
    return;
  }


  int
  searchForNeighbors (
      pcl::PointCloud<PointLabel>::Ptr& cloud_in,
      int col, int row,
      double radius,
      std::vector<int>& indices_ul,
      std::vector<int>& indices_ur,
      std::vector<int>& indices_lr,
      std::vector<int>& indices_ll,
      bool gap_l, bool gap_r, bool gap_a, bool gap_d)
  {
    //NaN test
    //TODO: use PCL function for that
    //if(cloud_in->points[index].x != cloud_in->points[index].x) return 0;

    gap_l=gap_r=gap_a=gap_d=false;
    indices_ul.clear();
    indices_ur.clear();
    indices_lr.clear();
    indices_ll.clear();

    int idx_x = col;
    int idx_y = row;

    int gap_l_ctr = 0;
    int gap_a_ctr = 0;
    for (int i=idx_x-radius; i<=idx_x; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y-radius+1; j<=idx_y; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ul.push_back(i+j*cloud_in->width);
        if(j==idx_y && cloud_in->points[i+j*cloud_in->width].label!=1) gap_l_ctr++;
        if(i==idx_x && cloud_in->points[i+j*cloud_in->width].label!=1) gap_a_ctr++;
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    if(gap_l_ctr==0) gap_l = true;
    if(gap_a_ctr==0) gap_a = true;

    int gap_r_ctr = 0;
    int gap_d_ctr = 0;
    for (int i=idx_x; i<idx_x+radius; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y-radius; j<=idx_y; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ur.push_back(i+j*cloud_in->width);
        if(j==idx_y && cloud_in->points[i+j*cloud_in->width].label!=1) gap_r_ctr++;
        if(i==idx_x && cloud_in->points[i+j*cloud_in->width].label!=1) gap_d_ctr++;
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    if(gap_r_ctr==0) gap_r = true;
    if(gap_d_ctr==0) gap_d = true;
    for (int i=idx_x; i<idx_x+radius; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y; j<=idx_y+radius; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_lr.push_back(i+j*cloud_in->width);
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    for (int i=idx_x-radius+1; i<=idx_x; i++)
    {
      if(i>=cloud_in->width) break;
      if(i<0) continue;
      for (int j=idx_y; j<=idx_y+radius; j++)
      {
        if(j<0) continue;
        if(i==idx_x && j==idx_y) continue; //skip p itself
        if(j>=cloud_in->height) break;
        indices_ll.push_back(i+j*cloud_in->width);
        //std::cout << "(" << i << "," << j << ")";
      }
    }
    return 1;
  }

  bool isStopperInNeighbors(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<int>& indices)
  {
    for(unsigned int i=0; i< indices.size(); i++)
    {
      if(cloud_in->points[indices[i]].label == 1)
        return true;
    }
    return false;
  }


  void propagateWavefront2(pcl::PointCloud<PointLabel>::Ptr& cloud_in)
  {
    //TODO: unlabel small cluster, change pixel step?
    boost::timer t;
    int width = cloud_in->width, height = cloud_in->height;
    std::vector<pcl::Boundary*> wave;
    std::vector<Coords> wave_coords;

    int cur_label = 3;
    int px_range = 5;
    std::vector<int> indices_ul;
    std::vector<int> indices_ur;
    std::vector<int> indices_lr;
    std::vector<int> indices_ll;

    for(int i = 0; i < height; i++ )
    {
      for(int j = 0; j < width; j++ )
      {
        //std::cout << i << "," << j << ":" << cur_label << std::endl;
        if(cloud_in->points[i*width+j].label == 0)
        {
          cur_label++;
          PointLabel* p = &cloud_in->points[i*width+j];
          p->label = cur_label;
          Coords c(j,i);

          //int count = 0;
          bool gap_l=false, gap_r=false, gap_a=false, gap_d=false;
          bool is_wave = true;
          while(is_wave)
          {
            //count++;
            int pt_ctr = c.u+c.v*width;
            if( c.u < width-1 && cloud_in->points[pt_ctr+1].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);
              if(c.u = 320 && c.v == 240)
              {
                std::cout << c.u << "," << c.v << " --> " << pt_ctr << std::endl;
                std::cout << "UL: ";
                for(unsigned int i=0; i<indices_ul.size(); i++)
                  std::cout << indices_ul[i] << ", ";
                std::cout << std::endl;
                std::cout << "UR: ";
                for(unsigned int i=0; i<indices_ur.size(); i++)
                  std::cout << indices_ur[i] << ", ";
                std::cout << std::endl;
                std::cout << "LR: ";
                for(unsigned int i=0; i<indices_lr.size(); i++)
                  std::cout << indices_lr[i] << ", ";
                std::cout << std::endl;
                std::cout << "LL: ";
                for(unsigned int i=0; i<indices_ll.size(); i++)
                  std::cout << indices_ll[i] << ", ";
                std::cout << std::endl;
              }
              cloud_in->points[pt_ctr+1].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ul) && isStopperInNeighbors(cloud_in, indices_ll) && gap_l))
              ///if(!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_lr))
                wave_coords.push_back(Coords(c.u+1,c.v));
            }
            if( c.u > 0 && cloud_in->points[pt_ctr-1].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr-1].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_lr) && gap_r))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_ul)) &&
              //    (isStopperInNeighbors(cloud_in, indices_lr) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_ul) || !isStopperInNeighbors(cloud_in, indices_ll))
                wave_coords.push_back(Coords(c.u-1,c.v));
            }
            //std::cout << pt_ctr+width << ": "<< (int)(cloud_in->points[pt_ctr+width].boundary_point) << std::endl;
            if(c.v < height-1 && cloud_in->points[pt_ctr+width].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr+width].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_ur) && isStopperInNeighbors(cloud_in, indices_ul) && gap_a))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_lr)) &&
              //    (isStopperInNeighbors(cloud_in, indices_ul) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_lr) || !isStopperInNeighbors(cloud_in, indices_ll))
                wave_coords.push_back(Coords(c.u,c.v+1));
            }
            if(c.v > 0 && cloud_in->points[pt_ctr-width].label==0)
            {
              searchForNeighbors (cloud_in,c.u,c.v,px_range,
                  indices_ul, indices_ur,
                  indices_lr, indices_ll,
                  gap_l, gap_r, gap_a, gap_d);

              cloud_in->points[pt_ctr-width].label = cur_label;
              if(!(isStopperInNeighbors(cloud_in, indices_lr) && isStopperInNeighbors(cloud_in, indices_ll) && gap_d))
              //if(!((isStopperInNeighbors(cloud_in, indices_ur) || isStopperInNeighbors(cloud_in, indices_lr)) &&
              //    (isStopperInNeighbors(cloud_in, indices_ul) || isStopperInNeighbors(cloud_in, indices_ll))))
              //if(!isStopperInNeighbors(cloud_in, indices_ur) || !isStopperInNeighbors(cloud_in, indices_ul))
                wave_coords.push_back(Coords(c.u,c.v-1));
            }
            //p = wave.back();
            if(wave_coords.size()>0)
            {
              //wave.pop_back();
              c = wave_coords.back();
              wave_coords.pop_back();
              //std::cout << wave_coords.size() << std::endl;
            }
            else
              is_wave = false;
          }
          /*if(count <= maxSize)
                                {
                                        region_size[cloud_in->points[i*width+j].boundary_point]=1;
                                }
                                else
                                {
                                        region_size[cloud_in->points[i*width+j].boundary_point]=2;
                                }*/
        }
        /*else
                        {
                                if(region_size[cloud_in->points[i*width+j].boundary_point]==1)
                                        cloud_in->points[i*width+j].boundary_point = 0;
                        }*/
      }

    }
    std::cout << "Time elapsed for wavefront propagation: " << t.elapsed() << std::endl;
    return;
  }


  /*Calculates the point curvature of a point cloud and thresholds to mark edges.
   * Curvature seems to be a weak indicator for edges.
   */
  void extractEdgesCurvature(PointCloudT::Ptr& cloud_in, pcl::PointCloud<pcl::InterestPoint>& cloud_out, cv::Mat& curvature_image)
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    integralEstimatePointNormals(*cloud_in, *cloud_n);

    boost::timer t;
    cloud_out.points.resize(cloud_n->points.size());
    cloud_out.header = cloud_n->header;

    int nr_p = 0;
    for( unsigned int i = 0; i < cloud_n->points.size(); i++)
    {
      cloud_out.points[i].strength = cloud_n->points[i].curvature;
      //if( cloud_n->points[i].curvature > 0.05)
      //  cloud_out.points[nr_p++] = cloud_in->points[i];
    }

    curvature_image = cv::Mat(cloud_in->height, cloud_in->width, CV_32FC1);
    int pt_idx=0;
    for(unsigned int row=0; row<curvature_image.rows; row++)
    {
      for(unsigned int col=0; col<curvature_image.cols; col++, pt_idx++)
      {
        curvature_image.at<float>(row,col) = cloud_out.points[pt_idx].strength;
      }
    }

    /*cloud_out.width = nr_p;
    cloud_out.height = 1;
    cloud_out.points.resize(nr_p);
    cloud_out.is_dense = true;*/
    ROS_INFO("Time elapsed for curvature estimation : %f", t.elapsed());
  }

  /*Finds edges in a range image using the RangeImageBorderExtractor
   * Looks promising but needs some modifications.
   * Try to adjust parameters in range_image_border_extractor.h
   */
  void extractEdgesRangeImage(PointCloudT::Ptr& cloud_in, pcl::PointCloud<pcl::PointWithRange>& cloud_out, cv::Mat& border_image)
  {

    //pcl::RangeImage range_image;
    Eigen::Affine3f sensorPose =
        (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

    //cloud_out.createFromPointCloud2 (*cloud_in, pcl::deg2rad(0.089f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), sensorPose);
    pcl::RangeImage range_image;
    range_image.header = cloud_in->header;
    range_image.width = cloud_in->width;
    range_image.height = cloud_in->height;
    range_image.is_dense = false;
    for(int i = 0; i<cloud_in->size(); i++)
    {
      pcl::PointWithRange p;

      //p.range = sqrt(cloud_in->points[i].x*cloud_in->points[i].x+cloud_in->points[i].y*cloud_in->points[i].y+cloud_in->points[i].z*cloud_in->points[i].z);
      /*if(cloud_in->points[i].x!=cloud_in->points[i].x || cloud_in->points[i].y!=cloud_in->points[i].y || cloud_in->points[i].z!=cloud_in->points[i].z)
                        {
                                p.x = p.y = p.z = 0;//std::numeric_limits<float>::quiet_NaN();
                                p.range=0;//-std::numeric_limits<float>::infinity();
                        }
                        else*/
      {
        p.x = cloud_in->points[i].x;
        p.y = cloud_in->points[i].y;
        p.z = cloud_in->points[i].z;
        p.range = sqrt(cloud_in->points[i].x*cloud_in->points[i].x+cloud_in->points[i].y*cloud_in->points[i].y+cloud_in->points[i].z*cloud_in->points[i].z);
      }
      range_image.points.push_back(p);
    }
    //std::string directory("/home/goa/pcl_daten/test/");
    //pcl::io::savePCDFileASCII (directory+"/range_image.pcd", range_image);

    ipa_features::RangeImageBorderExtractor border_extractor(&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute(border_descriptions);


    pcl::PointCloud<pcl::Boundary>::Ptr boundary_pts (new pcl::PointCloud<pcl::Boundary> ());
    //boundary_pts->points.resize(cloud_in->size());
    for (int y=0; y<(int)range_image.height; ++y)
    {
      for (int x=0; x<(int)range_image.width; ++x)
      {
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        {
          cloud_out.points.push_back(range_image.points[y*range_image.width + x]);
          pcl::Boundary p;
          p.boundary_point = 1;
          boundary_pts->points.push_back(p);
        }
        else
        {
          pcl::Boundary p;
          p.boundary_point = 0;
          boundary_pts->points.push_back(p);
        }
      }
    }

    border_image = cv::Mat(range_image.height, range_image.width, CV_8UC1);
    int pt_idx=0;
    for(int row=0; row<border_image.rows; row++)
    {
      for(int col=0; col<border_image.cols; col++, pt_idx++)
      {
        if( boundary_pts->points[pt_idx].boundary_point == 1)
          border_image.at<unsigned char>(row,col) = 255;
        else
          border_image.at<unsigned char>(row,col) = 0;
      }
    }
    //cv::imshow("range border image", border_image);
    //cv::waitKey();
  }

  /**Uses the PCL BoundaryEstimation to find edges in a point cloud. Uses an angle criterion to detect edges
   * Unfortunately marks the outline of a point cloud as edge; needs normal estimation
   */
  void extractPrincipalCurvature(PointCloudT::Ptr& cloud_in, pcl::PointCloud<pcl::InterestPoint>& cloud_out, cv::Mat& curvature_image)
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curv_pts (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    integralEstimatePointNormals(*cloud_in, *cloud_n);

    boost::timer t;
    //pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree;
    pcl::OrganizedDataIndex<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::OrganizedDataIndex<pcl::PointXYZRGBNormal> ());
    //tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> > ();
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal,pcl::PrincipalCurvatures> pce;
    pce.setSearchMethod(tree);
    pce.setInputCloud(cloud_n);
    //boundary.setSearchSurface (cloud);
    pce.setRadiusSearch(0.05);
    pce.setInputNormals(cloud_n);
    pce.compute(*curv_pts);
    //pcl::PointCloud<pcl::PointXYZRGBNormal> boundary_cloud;
    cloud_out.points.resize(cloud_n->points.size());
    cloud_out.header = cloud_n->header;
    int nr_p = 0;

    for( unsigned int i = 0; i < curv_pts->height; i++)
    {
      for( unsigned int j = 0; j < curv_pts->width; j++)
      {
        std::cout << i << "," << j << ": x,y,z,pc1,pc2: " << curv_pts->points[i*curv_pts->width+j].principal_curvature_x << "," <<
            curv_pts->points[i*curv_pts->width+j].principal_curvature_y << "," <<
            curv_pts->points[i*curv_pts->width+j].principal_curvature_z << "," <<
            curv_pts->points[i*curv_pts->width+j].pc1 << "," <<
            curv_pts->points[i*curv_pts->width+j].pc2 << std::endl;
      }
    }
    cloud_out.width = nr_p;
    cloud_out.height = 1;
    cloud_out.points.resize(nr_p);
    cloud_out.is_dense = true;

    curvature_image = cv::Mat(cloud_in->height, cloud_in->width, CV_32FC1);
    int pt_idx=0;
    for(unsigned int row=0; row<curvature_image.rows; row++)
    {
      for(unsigned int col=0; col<curvature_image.cols; col++, pt_idx++)
      {
        curvature_image.at<float>(row,col) = cloud_out.points[pt_idx].strength;
      }
    }


    ROS_INFO("Time elapsed for boundary estimation: %f", t.elapsed());
  }

  /**Segments a color image using an edge image and the watershed algorithm
   *
   */
  void segmentByEdgeImage(cv::Mat& color_image, cv::Mat& edge_image, cv::Mat& markers)
  {
    /// find contours in edge image
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(edge_image, contours, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    markers = cv::Mat::zeros(edge_image.size(), CV_32S);
    std::cout << "num contours: " << contours.size() << std::endl;
    for(int idx=0; idx<contours.size(); idx++)
      cv::drawContours(markers, contours, idx, cv::Scalar(idx+1));
    std::vector<cv::Vec3b> colorTab;
    for(int i = 0; i < contours.size(); i++ )
    {
      int b = cv::theRNG().uniform(0, 255);
      int g = cv::theRNG().uniform(0, 255);
      int r = cv::theRNG().uniform(0, 255);

      colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
    }
    cv::Mat vis_markers(markers.size(), CV_8UC3);
    for(int i = 0; i < markers.rows; i++ )
    {
      for(int j = 0; j < markers.cols; j++ )
      {
        int idx = markers.at<int>(i,j);
        if( idx == 0 )
          vis_markers.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
        else
          vis_markers.at<cv::Vec3b>(i,j) = colorTab[idx - 1];
      }
    }
    cv::imshow("Contours", vis_markers);
    cv::waitKey();
    /// apply watershed algorithm
    cv::watershed(color_image, markers);
    cv::Mat wshed_image=cv::Mat(markers.size(), CV_8UC3);
    for(int i = 0; i < markers.rows; i++ )
    {
      for(int j = 0; j < markers.cols; j++ )
      {
        int idx = markers.at<int>(i,j);
        if( idx == -1 )
          wshed_image.at<cv::Vec3b>(i,j) = cv::Vec3b(255,255,255);
        else
          wshed_image.at<cv::Vec3b>(i,j) = colorTab[idx - 1];
      }
    }
    cv::imshow("wshed image", wshed_image);
    cv::waitKey();

    //cvtColor( canny_image, color_canny_image, CV_GRAY2BGR );
    /*c_ptr = 0;
                int pt_idx=0;
                for (int row = 0; row < canny_image.rows; row++)
                {
                        c_ptr = canny_image.ptr<unsigned char>(row);
                        for (int col = 0; col < canny_image.cols; col++, pt_idx++)
                        {
                                if(c_ptr[col]==255)
                                {
                                        //pc_edge.points[pt_idx].boundary_point = 1;
                                        pc_out.points[nr_p++] = pc->points[pt_idx];
                                }
                        }
                }
                //resize pc_out according to filtered points
                pc_out.width = nr_p;
                pc_out.height = 1;
                pc_out.points.resize (nr_p);
                pc_out.is_dense = true;*/
    //cv::imshow("Canny Image", canny_image);

    /*vector<cv::Vec4i> lines;
                HoughLinesP( canny_image, lines, 1, CV_PI/180, 80, 30, 10 );
                for( size_t i = 0; i < lines.size(); i++ )
                {
                        line( color_canny_image, cv::Point(lines[i][0], lines[i][1]),
                                cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );

                }*/

  }

  /**Segments a point cloud using a watershed image
   * returns clusters of points belonging together
   */
  void getClusterIndices(PointCloudT::Ptr& cloud_in, cv::Mat& wshed_image, std::vector<pcl::PointIndices>& cluster_indices)
  {
    int max_idx=0;
    for(int i = 0; i < wshed_image.rows; i++ )
    {
      for(int j = 0; j < wshed_image.cols; j++ )
      {
        if(wshed_image.at<int>(i,j)>max_idx) max_idx=wshed_image.at<int>(i,j);
      }
    }
    for(int k=0; k<=max_idx; k++)
    {
      pcl::PointIndices cluster;
      for(int i = 0; i < wshed_image.rows; i++ )
      {
        for(int j = 0; j < wshed_image.cols; j++ )
        {
          if(wshed_image.at<int>(i,j)==k)
            cluster.indices.push_back(i*cloud_in->width+j);
        }
      }
      cluster_indices.push_back(cluster);
    }
  }

  void getClusterIndices(pcl::PointCloud<PointLabel>::Ptr& cloud_in, std::vector<pcl::PointIndices>& cluster_indices, cv::Mat& seg_img)
  {
    int max_idx=0;
    int i=0;
    for(i = 0; i < cloud_in->size(); i++ )
    {
      if(cloud_in->points[i].label>max_idx) max_idx=cloud_in->points[i].label;
    }
    for(int k=0; k<=max_idx; k++)
    {
      pcl::PointIndices cluster;
      for(i = 0; i < cloud_in->size(); i++ )
      {
        if(cloud_in->points[i].label==k)
          cluster.indices.push_back(i);
      }
      cluster_indices.push_back(cluster);
    }
    seg_img = cv::Mat(cloud_in->height,cloud_in->width, CV_8UC3);
    std::vector<cv::Vec3b> colorTab;
    for(int i = 0; i < 256; i++ )
    {
      int b = cv::theRNG().uniform(0, 255);
      int g = cv::theRNG().uniform(0, 255);
      int r = cv::theRNG().uniform(0, 255);

      colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
    }
    for(int i = 0; i < seg_img.rows; i++ )
    {
      for(int j = 0; j < seg_img.cols; j++ )
      {
        int label = cloud_in->points[i*cloud_in->width+j].label;
        seg_img.at<cv::Vec3b>(i,j) = colorTab[label];
      }
    }

  }

};

/*int main(int argc, char** argv)
{
        ExtractFeatures ef;

        std::string directory("/home/goa/pcl_daten/corner/");
        pcl::PointCloud<pcl::Boundary>::Ptr cloud_in = pcl::PointCloud<pcl::Boundary>::Ptr (new pcl::PointCloud<pcl::Boundary>);
        pcl::io::loadPCDFile(directory+"contour_sim.pcd", *cloud_in);
        cv::Mat edge_img(cv::Size(cloud_in->height,cloud_in->width), CV_8UC3);
    for(unsigned int i = 0; i < cloud_in->height; i++ )
    {
        for(unsigned int j = 0; j < cloud_in->width; j++ )
        {
                std::cout << (int)(cloud_in->points[i*cloud_in->width+j].boundary_point);
                //std::cout << i << "," << j << ": " << (int)(cloud_in->points[i*cloud_in->width+cloud_in->height].boundary_point) << std::endl;
                        if(cloud_in->points[i*cloud_in->width+j].boundary_point == 1)
                                edge_img.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
                        else
                                edge_img.at<cv::Vec3b>(i,j) = cv::Vec3b(255,255,255);
        }
    }
        cv::imshow("edge",edge_img);
        cv::waitKey();
        ef.propagateWavefront(cloud_in);
        std::vector<pcl::PointIndices> cluster;
        cv::Mat seg_img;
        ef.getClusterIndices(cloud_in, cluster, seg_img);
        cv::namedWindow("seg",CV_WINDOW_NORMAL);
        cv::imshow("seg",seg_img);
        cv::waitKey();
        for(int i=0; i<cluster.size(); i++)
        {
                std::cout << i << ": ";
                for (int j=0; j<cluster[i].indices.size(); j++)
                        std::cout << cluster[i].indices[j] << ",";
                std::cout << std::endl;
        }


        return 1;
}*/

//#######################
//#### main programm ####
int main(int argc, char** argv)
{

  /// Create extract features  class instance
  ExtractFeatures ef;

  /// Load PCD file as input; better use binary PCD files, ascii files seem to generate corrupt point clouds
  std::string directory("/home/goa/pcl_daten/corner/");
  if (argc == 2)
    directory = argv[1];
  else
    std::cout << "Selected default directory \" " << directory << "\"" << std::endl;

  PointCloudT::Ptr cloud_in = PointCloudT::Ptr (new PointCloudT);
  pcl::io::loadPCDFile(directory+"pc_0.pcd", *cloud_in);

  cv::Mat range_image(cloud_in->height,cloud_in->width,CV_32FC1);
  ef.getRangeImage(cloud_in, range_image);
  cv::Mat range_image_show;
  cv::normalize(range_image, range_image_show, 0,1,cv::NORM_MINMAX);
  cv::imshow("Range Image", range_image_show);
  cv::waitKey();

  /// Extract edges on the color image
  cv::Mat color_image(cloud_in->height,cloud_in->width,CV_8UC3);
  ef.getColorImage(cloud_in, color_image);
  cv::imshow("Color Image", color_image);
  std::vector<cv::Mat> img_channels;
  img_channels.resize(3);
  //for(int i=0; i<2; i++)
  //  img_channels.push_back(cv::Mat());
  cv::split(color_image, img_channels);
  std::vector<cv::Mat> sobel_imgs;
  for(int i=0; i<3; i++)
  {
    cv::Mat sobel_image_x;
    ef.extractEdgesSobel(img_channels[i], sobel_image_x,1 ,0);
    cv::Mat sobel_image_y;
    ef.extractEdgesSobel(img_channels[i], sobel_image_y,0 ,1);
    cv::Mat sobel_image = sobel_image_x + sobel_image_y;
    sobel_imgs.push_back(sobel_image);
  }
  for(int i=1; i<3; i++)
  {
    sobel_imgs[0] += sobel_imgs[i];
  }
  cv::normalize(sobel_imgs[0], sobel_imgs[0], 0,1,cv::NORM_MINMAX);
  //cv::threshold(sobel_imgs[0],sobel_imgs[0],0.01,1,cv::THRESH_BINARY);
  //cv::normalize(sobel_imgs[0], sobel_imgs[0], 0,1,cv::NORM_MINMAX);
  cv::imshow("Sobel Image", sobel_imgs[0]);


  std::vector<cv::Mat> laplace_imgs;
  for(int i=0; i<3; i++)
  {
    cv::Mat image;
    ef.extractEdgesLaPlace(img_channels[i], image);
    laplace_imgs.push_back(image);
  }
  for(int i=1; i<3; i++)
  {
    laplace_imgs[0] += laplace_imgs[i];
  }
  double min, max;
  cv::minMaxLoc(laplace_imgs[0], &min, &max);
  //laplace_imgs[0] = laplace_imgs[0] - 0.5;
  cv::Mat laplace_image = cv::abs(laplace_imgs[0]);
  std::cout << "minmax laplace: " << min << ", " << max << std::endl;
  cv::normalize(laplace_image, laplace_image, 0,1,cv::NORM_MINMAX);
  cv::imshow("LaPlace Image", laplace_image);
  cv::threshold(laplace_image, laplace_image, 0.1, 1, cv::THRESH_TOZERO);
  cv::imshow("LaPlace Image Th1", laplace_image);
  laplace_image *= 2;
  cv::threshold(laplace_image, laplace_image, 1, 1, cv::THRESH_TRUNC);
  cv::imshow("LaPlace Image Th2", laplace_image);
  cv::waitKey();

  //pcl::PointCloud<pcl::Boundary> cloud_out;
  pcl::PointCloud<pcl::InterestPoint>::Ptr cloud_out = pcl::PointCloud<pcl::InterestPoint>::Ptr (new pcl::PointCloud<pcl::InterestPoint>);
  cv::Mat border_image;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ni (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
  //ef.estimatePointNormals(*cloud_in, *cloud_n);

  ef.integralEstimatePointNormals(*cloud_in, *cloud_n);
  pcl::io::savePCDFileASCII (directory+"/output/corner_ni.pcd", *cloud_n);
  /// Extract edges using curvature
  /*pcl::PointCloud<pcl::InterestPoint>::Ptr cloud_curv(new pcl::PointCloud<pcl::InterestPoint>);
  cv::Mat curvature_image;
  //ef.extractEdgesCurvature(cloud_in, *cloud_curv, curvature_image);
  ef.extractPrincipalCurvature(cloud_in, *cloud_curv, curvature_image);
  cv::imshow("Curvature", curvature_image);
  cv::waitKey();*/
  //pcl::io::savePCDFileASCII (directory+"/edges/edges_curvature.pcd", cloud_out);
  //return 0;
  /// Extract edges using boundary estimation

  ef.extractEdges3D(cloud_in, *cloud_out, border_image);

  cv::imshow("Edges", border_image);
  cv::waitKey();

  border_image += sobel_imgs[0];
  border_image += laplace_image;
  //cv::normalize(border_image, border_image, 0,1,cv::NORM_MINMAX);

  cv::Mat thr_border_image, bin_border_image;
  cv::threshold(border_image, thr_border_image, 0.1, 1, cv::THRESH_BINARY);
  thr_border_image.convertTo(bin_border_image, CV_8UC1, 255);

  cv::imshow("Bin Edges", bin_border_image);
  cv::imshow("Combined Edges", border_image);
  cv::Mat edge_morph;
  cv::morphologyEx(bin_border_image, edge_morph, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)), cv::Point(-1,-1), 1);
  cv::imshow("Edge morph", edge_morph);
  cv::waitKey();


  pcl::PointCloud<PointLabel>::Ptr cloud_out2 = pcl::PointCloud<PointLabel>::Ptr (new pcl::PointCloud<PointLabel>);
  cloud_out2->height = cloud_out->height;
  cloud_out2->width = cloud_out->width;
  cloud_out2->points.resize(cloud_out2->height*cloud_out2->width);
  for (int i=0; i<cloud_out->height; i++)
  {
    for (int j=0; j<cloud_out->width; j++)
    {
    if(/*edge_morph.at<unsigned char>(i,j)*/border_image.at<float>(i,j) > 0.1)
    //if(cloud_out->points[i].strength > 0.1)
      cloud_out2->points[i*cloud_out->width+j].label = 1;//(int)(cloud_out->points[i].boundary_point);
    else if(cloud_out->points[i*cloud_out->width+j].strength >= 2)
      cloud_out2->points[i*cloud_out->width+j].label = cloud_out->points[i*cloud_out->width+j].strength;
    else
      cloud_out2->points[i*cloud_out->width+j].label = 0;
    /*if(cloud_out->points[i].strength == 2)
      cloud_out2->points[i].label = 2;
    if(cloud_out->points[i].strength == 3)
      cloud_out2->points[i].label = 3;*/
    }
  }

  pcl::io::savePCDFileASCII (directory+"/output/edge_cloud.pcd", *cloud_out2);
  cob_3d_features::Segmentation seg;
  seg.propagateWavefront2(cloud_out2);

  std::vector<pcl::PointIndices> clusters;
  cv::Mat seg_img;
  seg.getClusterIndices(cloud_out2, clusters, seg_img);

  /*pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  for (int i = 0; i<clusters.size(); i++)
  {
    if(clusters[i].indices.size() > 100)
    {
      pcl::PointCloud<pcl::PointXYZRGB> cluster;
      extract.setInputCloud (cloud_in);
      extract.setIndices (boost::make_shared<const pcl::PointIndices> (clusters[i]));
      extract.setNegative (false);
      extract.filter (cluster);
      std::stringstream ss;
      ss << i;
      pcl::io::savePCDFileASCII (directory+"/output/cluster_"+ss.str()+".pcd", cluster);
    }
  }*/
  //cv::namedWindow("seg",CV_WINDOW_NORMAL);
  cv::imshow("seg2",seg_img);
  cv::waitKey();

  clusters.clear();
  ef.propagateWavefront(cloud_out2);
  ef.getClusterIndices(cloud_out2, clusters, seg_img);
  cv::imshow("seg",seg_img);
  cv::waitKey();
  //pcl::io::savePCDFileASCII (directory+"/edges/edges_boundary.pcd", cloud_out);


  cv::Mat markers;
  ef.segmentByEdgeImage(color_image, bin_border_image, markers);

  return 0;

  /// Extract edges using range image border extraction
  pcl::RangeImage range_image_out;
  ef.extractEdgesRangeImage(cloud_in, range_image_out, border_image);
  pcl::io::savePCDFileASCII (directory+"/edges/edges_range_border.pcd", range_image_out);
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.addCoordinateSystem(1.0f);
  viewer.addPointCloud<PointT>(cloud_in, "original point cloud");
  viewer.addPointCloud<pcl::PointWithRange>(range_image_out.makeShared(), "border points");
  while(!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    usleep(100000);
  }

  /// Combine two edge images
  /*cv::Mat combined_edge_image = canny_image | border_image;
        cv::imshow("combined edge image", combined_edge_image);
        cv::waitKey();

        /// Segment color image using canny edge image (can also be done with combined edge image)
        cv::Mat wshed_canny;
        ef.segmentByEdgeImage(color_image, canny_image, wshed_canny);

        /// Cluster point cloud according to color image segmentation
        std::vector<pcl::PointIndices> cluster_indices;
        ef.getClusterIndices(cloud_in, wshed_canny, cluster_indices);
        pcl::ExtractIndices<PointT> extract;
        for(unsigned int i = 0; i < cluster_indices.size(); i++)
        {
                if(cluster_indices[i].indices.size()>100)
                {
                        pcl::PointCloud<PointT> cluster;
                        extract.setInputCloud (cloud_in);
                        extract.setIndices (boost::make_shared<const pcl::PointIndices> (cluster_indices[i]));
                        extract.setNegative (false);
                        extract.filter (cluster);
                        stringstream ss; //create a stringstream
                        ss << directory << "/cluster/cluster_" << i << ".pcd";//add number to the stream
                        pcl::io::savePCDFileASCII (ss.str(), cluster);
                }
        }*/

  return 0;
}



