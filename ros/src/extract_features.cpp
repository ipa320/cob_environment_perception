/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 01/2011
 * ToDo:
 * Only update if new robot pose available
 * Resample point cloud
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


//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <tf_conversions/tf_kdl.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv.h>
#include <highgui.h>
#include "pcl/io/pcd_io.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/normal_3d.h"
#include <pcl/features/boundary.h>
//#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/PointIndices.h>
#include "pcl/filters/extract_indices.h"
#include <cob_env_model/ipa_range_image.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

//#include <cob_vision_features/SURFDetector.h>
//#include <cob_vision_features/AbstractFeatureVector.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB PointT;



//####################
//#### node class ####
class ExtractFeatures
{
public:
    // Constructor
	ExtractFeatures(const ros::NodeHandle& nh)
	  :	n_(nh)
	{
		//point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &ExtractFeatures::extractLines, this);
		point_cloud_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("point_cloud2_featured",1);
	}

    // Destructor
    ~ExtractFeatures()
    {
    	/// void
    }

	void estimatePointNormals(PointCloud& cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_n)
	{
		//pcl::PointCloud<pcl::PointXYZ> cloud;
		//pcl::PCDReader reader;
		//reader.read ("/home/goa/pcl_daten/table/icp_fov/pc_aligned_1.pcd", cloud);
		//std::cout << "Input cloud has " << cloud.size() << " data points" << std::endl;

		pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> normalEstimator;
		normalEstimator.setInputCloud(boost::make_shared<PointCloud >(cloud));
		pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
		normalEstimator.setSearchMethod(tree);
		//normalEstimator.setKSearch(50);
		normalEstimator.setRadiusSearch(0.05);
		//normalEstimator.setNumberOfThreads(8);
		pcl::PointCloud<pcl::Normal> cloud_normal;
		boost::timer t;
		normalEstimator.compute(cloud_normal);

		ROS_INFO("Time elapsed for normal estimation: %f", t.elapsed());
		pcl::concatenateFields (cloud, cloud_normal, cloud_n);
		return;
	}

	void getColorImage(PointCloud::Ptr& pc, cv::Mat& color_image)
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

    void extractEdgesCanny(cv::Mat& color_image, cv::Mat& canny_image)
	{
    	ROS_INFO("[env_model_node] Detecting features");
    	//PointCloud pc_out;
        //pc_out.points.resize (pc->points.size ());
        //pc_out.header = pc->header;

        int nr_p = 0;

        boost::timer t;

    	ROS_INFO("Time elapsed for image conversion: %f", t.elapsed());
    	t.restart();
    	cv::Mat grey_image;
    	cvtColor( color_image, grey_image, CV_RGB2GRAY );
    	cv::imshow("Color Image", color_image);
    	//cv::imshow("Grey Image", grey_image);
    	//cv::waitKey();
    	cv::Mat color_canny_image;
        Canny( grey_image, canny_image, 50, 150, 3 );
        cv::imshow("Canny Image", canny_image);
        cv::waitKey();
    	ROS_INFO("Time elapsed for canny edge: %f", t.elapsed());

	}

	void extractEdgesBoundary(PointCloud::Ptr& cloud_in, PointCloud& cloud_out, cv::Mat& border_image)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
		pcl::PointCloud<pcl::Boundary>::Ptr boundary_pts (new pcl::PointCloud<pcl::Boundary> ());
		estimatePointNormals(*cloud_in, *cloud_n);

		boost::timer t;
		pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree;
		tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> > ();
		pcl::BoundaryEstimation<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal,pcl::Boundary> boundary;
		boundary.setSearchMethod(tree);
		boundary.setInputCloud(cloud_n);
		//boundary.setSearchSurface (cloud);
		boundary.setRadiusSearch(0.05);
		boundary.setInputNormals(cloud_n);
		//boundary.angle_threshold_ = M_PI/4;
		boundary.compute(*boundary_pts);
		//pcl::PointCloud<pcl::PointXYZRGBNormal> boundary_cloud;
		cloud_out.points.resize(cloud_n->points.size());
		cloud_out.header = cloud_n->header;
		int nr_p = 0;

		for( unsigned int i = 0; i < cloud_in->points.size(); i++)
		{
			if( boundary_pts->points[i].boundary_point == 1)
			{
				//pc_edge.points[i].boundary_point++;
				cloud_out.points[nr_p++] = cloud_in->points[i];
			}
		}
		cloud_out.width = nr_p;
		cloud_out.height = 1;
		cloud_out.points.resize(nr_p);
		cloud_out.is_dense = true;

		border_image = cv::Mat(cloud_in->height, cloud_in->width, CV_8UC1);
		int pt_idx=0;
		for(unsigned int row=0; row<border_image.rows; row++)
		{
			for(unsigned int col=0; col<border_image.cols; col++, pt_idx++)
			{
				if( boundary_pts->points[pt_idx].boundary_point == 1)
					border_image.at<unsigned char>(row,col) = 255;
				else
					border_image.at<unsigned char>(row,col) = 0;
			}
		}
		cv::imshow("boundary image", border_image);
		cv::waitKey();
		ROS_INFO("Time elapsed for boundary estimation: %f", t.elapsed());
	}

	void extractEdgesCurvature(PointCloud::Ptr& cloud_in, PointCloud& cloud_out)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
		estimatePointNormals(*cloud_in, *cloud_n);

		boost::timer t;
		cloud_out.points.resize(cloud_n->points.size());
		cloud_out.header = cloud_n->header;

		int nr_p = 0;
		for( unsigned int i = 0; i < cloud_n->points.size(); i++)
		{
			if( cloud_n->points[i].curvature > 0.05)
				cloud_out.points[nr_p++] = cloud_in->points[i];
		}

		cloud_out.width = nr_p;
		cloud_out.height = 1;
		cloud_out.points.resize(nr_p);
		cloud_out.is_dense = true;
		ROS_INFO("Time elapsed for boundary estimation (curvature): %f", t.elapsed());
	}

	void extractEdgesRangeImage(PointCloud::Ptr& cloud_in, pcl::RangeImage& cloud_out, cv::Mat& border_image)
	{

		//pcl::RangeImage range_image;
		Eigen::Affine3f sensorPose =
		  (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

		cloud_out.createFromPointCloud (*cloud_in, pcl::deg2rad(0.089f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), sensorPose);

		  pcl::RangeImageBorderExtractor border_extractor(&cloud_out);
		  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
		  border_extractor.compute(border_descriptions);


		  // ----------------------------------
		  // -----Show points in 3D viewer-----
		  // ----------------------------------
		  pcl::PointCloud<pcl::PointWithRange> border_points, veil_points, shadow_points;
		  pcl::PointCloud<pcl::Boundary>::Ptr boundary_pts (new pcl::PointCloud<pcl::Boundary> ());
		  //boundary_pts->points.resize(cloud_in->size());
		  for (int y=0; y<(int)cloud_out.height; ++y)
		  {
		    for (int x=0; x<(int)cloud_out.width; ++x)
		    {
		      if (border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER] ||
		    		  border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER] )
		      {
		        border_points.points.push_back(cloud_out.points[y*cloud_out.width + x]);
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
		      //else if (border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
		      //  veil_points.points.push_back(cloud_out.points[y*cloud_out.width + x]);
		    }
		  }
		  std::string directory("/home/goa/pcl_daten/test/");
		  pcl::io::savePCDFileASCII (directory+"/edges/ri_border_pts.pcd", border_points);

			border_image = cv::Mat(cloud_out.height, cloud_out.width, CV_8UC1);
			int pt_idx=0;
			for(unsigned int row=0; row<border_image.rows; row++)
			{
				for(unsigned int col=0; col<border_image.cols; col++, pt_idx++)
				{
					if( boundary_pts->points[pt_idx].boundary_point == 1)
						border_image.at<unsigned char>(row,col) = 255;
					else
						border_image.at<unsigned char>(row,col) = 0;
				}
			}
			cv::imshow("boundary image", border_image);
			cv::waitKey();

		/*	pcl_visualization::PCLVisualizer viewer("3D Viewer");
			  viewer.addCoordinateSystem(1.0f);
			  viewer.addPointCloud(cloud_out, "original point cloud");
		  viewer.addPointCloud(border_points, "border points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "border points");
		  viewer.addPointCloud(veil_points, "veil points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "veil points");
		  viewer.addPointCloud(shadow_points, "shadow points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "shadow points");
*/
		  /*std::string directory("/home/goa/pcl_daten/kitchen_sf/");
			pcl::io::savePCDFileASCII (directory+"/edges/ri_border_pts.pcd", border_points);
			pcl::io::savePCDFileASCII (directory+"/edges/ri_shadow_pts.pcd", shadow_points);*/

	/*	  while(!viewer.wasStopped())
		  {
		    viewer.spinOnce(100);
		    usleep(100000);
		  }*/
	}

	void extractEdgesRangeImage2(PointCloud::Ptr& cloud_in, IPARangeImage& cloud_out, cv::Mat& border_image)
	{

		//pcl::RangeImage range_image;
		Eigen::Affine3f sensorPose =
		  (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

		//cloud_out.createFromPointCloud2 (*cloud_in, pcl::deg2rad(0.089f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), sensorPose);
		cloud_out.header = cloud_in->header;
		cloud_out.width = cloud_in->width;
		cloud_out.height = cloud_in->height;
		cloud_out.is_dense = false;
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
			cloud_out.points.push_back(p);
		}
		std::string directory("/home/goa/pcl_daten/test/");
		//pcl::io::savePCDFileASCII (directory+"/range_image2.pcd", cloud_out);

		  pcl::RangeImageBorderExtractor border_extractor(&cloud_out);
		  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
		  border_extractor.compute(border_descriptions);


		  // ----------------------------------
		  // -----Show points in 3D viewer-----
		  // ----------------------------------
		  pcl::PointCloud<pcl::PointWithRange> border_points, veil_points, shadow_points;
		  pcl::PointCloud<pcl::Boundary>::Ptr boundary_pts (new pcl::PointCloud<pcl::Boundary> ());
		  //boundary_pts->points.resize(cloud_in->size());
		  for (int y=0; y<(int)cloud_out.height; ++y)
		  {
		    for (int x=0; x<(int)cloud_out.width; ++x)
		    {
		      if (border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER] ||
		    		  border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER] ||
		    		  border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
		      {
		        border_points.points.push_back(cloud_out.points[y*cloud_out.width + x]);
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
		      //else if (border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
		      //  veil_points.points.push_back(cloud_out.points[y*cloud_out.width + x]);
		    }
		  }
		  //std::string directory("/home/goa/pcl_daten/kitchen_kinect/");
		  pcl::io::savePCDFileASCII (directory+"/edges/ri_border_pts.pcd", border_points);

			border_image = cv::Mat(cloud_out.height, cloud_out.width, CV_8UC1);
			int pt_idx=0;
			for(unsigned int row=0; row<border_image.rows; row++)
			{
				for(unsigned int col=0; col<border_image.cols; col++, pt_idx++)
				{
					if( boundary_pts->points[pt_idx].boundary_point == 1)
						border_image.at<unsigned char>(row,col) = 255;
					else
						border_image.at<unsigned char>(row,col) = 0;
				}
			}
			cv::imshow("boundary image", border_image);
			cv::waitKey();

		/*	pcl_visualization::PCLVisualizer viewer("3D Viewer");
			  viewer.addCoordinateSystem(1.0f);
			  viewer.addPointCloud(cloud_out, "original point cloud");
		  viewer.addPointCloud(border_points, "border points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "border points");
		  viewer.addPointCloud(veil_points, "veil points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "veil points");
		  viewer.addPointCloud(shadow_points, "shadow points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "shadow points");*/

		  /*std::string directory("	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	for(unsigned int i = 0; i < cluster_indices.size(); i++)
	{
		if(cluster_indices[i].indices.size()>100)
		{
			pcl::PointCloud<pcl::PointXYZRGB> cluster;
			extract.setInputCloud (cloud_in);
			extract.setIndices (boost::make_shared<const pcl::PointIndices> (cluster_indices[i]));
			extract.setNegative (false);
			extract.filter (cluster);
			stringstream ss;//create a stringstream
			ss << directory << "/cluster/cluster_" << i << ".pcd";//add number to the stream
			pcl::io::savePCDFileASCII (ss.str(), cluster);
		}
	}/home/goa/pcl_daten/kitchen_sf/");
			pcl::io::savePCDFileASCII (directory+"/edges/ri_border_pts.pcd", border_points);
			pcl::io::savePCDFileASCII (directory+"/edges/ri_shadow_pts.pcd", shadow_points);*/

		 /* while(!viewer.wasStopped())
		  {
		    viewer.spinOnce(100);
		    usleep(100000);
		  }*/
	}

	void segmentByEdgeImage(cv::Mat& color_image, cv::Mat& edge_image, cv::Mat& markers)
	{
		cv::Mat edge_morph;
		cv::morphologyEx(edge_image, edge_morph, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)), cv::Point(-1,-1), 1);
		cv::imshow("Edge morph", edge_morph);
		vector<vector<cv::Point> > contours;
		cv::findContours(edge_image, contours, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
		markers = cv::Mat::zeros(edge_image.size(), CV_32S);
		std::cout << "num contours: " << contours.size() << std::endl;
		for(int idx=0; idx<contours.size(); idx++)
		cv::drawContours(markers, contours, idx, cv::Scalar(idx+1));
		vector<cv::Vec3b> colorTab;
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
		//cv::imshow("Color Canny Image", color_canny_image);
		//cv::waitKey();

	}

	void getClusterIndices(PointCloud::Ptr& cloud_in, cv::Mat& wshed_image, std::vector<pcl::PointIndices>& cluster_indices)
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

	void extractEdgesByAngle(PointCloud::Ptr &pc_in, PointCloud &pc_out)
	{
		//pointer to indices of points to be removed
		pcl::PointIndices::Ptr points_to_remove (new pcl::PointIndices ());

		double upper_angle_deg_ = 170;
		//upper angle threshold in radians
		double upper_angle_thresh = upper_angle_deg_/180*M_PI;

		//Lower angle threshold in radians
		double lower_angle_thresh = (180-upper_angle_deg_)/180*M_PI;
		for (unsigned int i = 0; i < pc_in->points.size(); i++)
		{
			if (!pcl::hasValidXYZ(pc_in->points[i]))  continue;
			if(i< pc_in->width || i%pc_in->width==0 || i%pc_in->width==3 || i>pc_in->width*(pc_in->height-1)) continue; //skip border points
			Eigen::Vector3f v_m(pc_in->points[i].x,pc_in->points[i].y,pc_in->points[i].z);
			Eigen::Vector3f v_m_n = v_m.normalized();
			int index = i-pc_in->width-1;
			Eigen::Vector3f vd_ul(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			vd_ul.normalize();

			double angle = std::acos(v_m_n.dot(vd_ul));
			if(angle > upper_angle_thresh || angle < lower_angle_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}
			index = i-pc_in->width;
			Eigen::Vector3f vd_u(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			vd_u.normalize();
			angle = std::acos(v_m_n.dot(vd_u));
			if(angle > upper_angle_thresh || angle < lower_angle_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}
			index = i-pc_in->width+1;
			Eigen::Vector3f vd_ur(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			vd_ur.normalize();
			angle = std::acos(v_m_n.dot(vd_ur));
			if(angle > upper_angle_thresh || angle < lower_angle_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}
			index = i-1;
			Eigen::Vector3f vd_l(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			vd_l.normalize();
			angle = std::acos(v_m_n.dot(vd_l));
			if(angle > upper_angle_thresh || angle < lower_angle_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}
			index = i+1;
			Eigen::Vector3f vd_r(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			vd_r.normalize();
			angle = std::acos(v_m_n.dot(vd_r));
			if(angle > upper_angle_thresh || angle < lower_angle_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}
			index = i+pc_in->width-1;
			Eigen::Vector3f vd_ll(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			vd_ll.normalize();
			angle = std::acos(v_m_n.dot(vd_ll));
			if(angle > upper_angle_thresh || angle < lower_angle_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}
			index = i+pc_in->width;
			Eigen::Vector3f vd_lo(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			vd_lo.normalize();
			angle = std::acos(v_m_n.dot(vd_lo));
			if(angle > upper_angle_thresh || angle < lower_angle_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}
			index = i+pc_in->width+1;
			Eigen::Vector3f vd_lr(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			vd_lr.normalize();
			angle = std::acos(v_m_n.dot(vd_lr));
			if(angle > upper_angle_thresh || angle < lower_angle_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}
		}
		pcl::ExtractIndices<pcl::PointXYZRGB> extractIndices;
		extractIndices.setInputCloud(pc_in);
		extractIndices.setIndices(points_to_remove);
		extractIndices.setNegative(false);
		extractIndices.filter(pc_out);
	}


	void extractEdgesByRange(PointCloud::Ptr &pc_in, PointCloud &pc_out)
	{
		//pointer to indices of points to be removed
		pcl::PointIndices::Ptr points_to_remove (new pcl::PointIndices ());

		//double upper_angle_deg_ = 170;
		//upper angle threshold in radians
		//double upper_angle_thresh = upper_angle_deg_/180*M_PI;
		double range_thresh = 0.05;

		//Lower angle threshold in radians
		//double lower_angle_thresh = (180-upper_angle_deg_)/180*M_PI;
		for (unsigned int i = 0; i < pc_in->points.size(); i++)
		{
			if (!pcl::hasValidXYZ(pc_in->points[i]))  continue;
			if(i< pc_in->width || i%pc_in->width==0 || i%pc_in->width==3 || i>pc_in->width*(pc_in->height-1)) continue; //skip border points
			double range = sqrt(pc_in->points[i].x*pc_in->points[i].x+pc_in->points[i].y*pc_in->points[i].y+pc_in->points[i].z*pc_in->points[i].z);
			//Eigen::Vector3f v_m(pc_in->points[i].x,pc_in->points[i].y,pc_in->points[i].z);
			//Eigen::Vector3f v_m_n = v_m.normalized();
			int index = i-pc_in->width-1;
			double range_n = sqrt(pc_in->points[index].x*pc_in->points[index].x+pc_in->points[index].y*pc_in->points[index].y+pc_in->points[index].z*pc_in->points[index].z);
			//Eigen::Vector3f vd_ul(v_m(0)-pc_in->points[index].x, v_m(1)-pc_in->points[index].y, v_m(2)-pc_in->points[index].z);
			//vd_ul.normalize();

			if(std::fabs(range-range_n)>range_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}

			index = i-pc_in->width;
			range_n = sqrt(pc_in->points[index].x*pc_in->points[index].x+pc_in->points[index].y*pc_in->points[index].y+pc_in->points[index].z*pc_in->points[index].z);
			if(std::fabs(range-range_n)>range_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}

			index = i-pc_in->width+1;
			range_n = sqrt(pc_in->points[index].x*pc_in->points[index].x+pc_in->points[index].y*pc_in->points[index].y+pc_in->points[index].z*pc_in->points[index].z);
			if(std::fabs(range-range_n)>range_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}

			index = i-1;
			range_n = sqrt(pc_in->points[index].x*pc_in->points[index].x+pc_in->points[index].y*pc_in->points[index].y+pc_in->points[index].z*pc_in->points[index].z);
			if(std::fabs(range-range_n)>range_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}

			index = i+1;
			range_n = sqrt(pc_in->points[index].x*pc_in->points[index].x+pc_in->points[index].y*pc_in->points[index].y+pc_in->points[index].z*pc_in->points[index].z);
			if(std::fabs(range-range_n)>range_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}

			index = i+pc_in->width-1;
			range_n = sqrt(pc_in->points[index].x*pc_in->points[index].x+pc_in->points[index].y*pc_in->points[index].y+pc_in->points[index].z*pc_in->points[index].z);
			if(std::fabs(range-range_n)>range_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}

			index = i+pc_in->width;
			range_n = sqrt(pc_in->points[index].x*pc_in->points[index].x+pc_in->points[index].y*pc_in->points[index].y+pc_in->points[index].z*pc_in->points[index].z);
			if(std::fabs(range-range_n)>range_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}

			index = i+pc_in->width+1;
			range_n = sqrt(pc_in->points[index].x*pc_in->points[index].x+pc_in->points[index].y*pc_in->points[index].y+pc_in->points[index].z*pc_in->points[index].z);
			if(std::fabs(range-range_n)>range_thresh)
			{
				points_to_remove->indices.push_back(i);
				continue;
			}

		}
		pcl::ExtractIndices<pcl::PointXYZRGB> extractIndices;
		extractIndices.setInputCloud(pc_in);
		extractIndices.setIndices(points_to_remove);
		extractIndices.setNegative(false);
		extractIndices.filter(pc_out);
	}


    ros::NodeHandle n_;

protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
};


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "extract_features");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	/// Create camera node class instance
	ExtractFeatures ef(nh);

	 pcl::PointCloud<pcl::Boundary> pc_edge;

	std::string directory("/home/goa/pcl_daten/test/");
	PointCloud::Ptr cloud_in = PointCloud::Ptr (new PointCloud);
	sensor_msgs::PointCloud2 cloud_blob;
	pcl::io::loadPCDFile(directory+"frame_0.000000000_bin.pcd", *cloud_in);

	PointCloud::Ptr cloud_out = PointCloud::Ptr (new PointCloud);
	ef.extractEdgesByRange(cloud_in, *cloud_out);
	pcl::visualization::CloudViewer viewer("3D Viewer");
	viewer.showCloud(cloud_out);

	 while(!viewer.wasStopped())
	  {
		 usleep(100000);
	  }
	//pcl::fromROSMsg (cloud_blob, *cloud_in);
	//pcl::io::savePCDFile (directory+"frame_0.000000000_bin.pcd", *cloud_in, true);
	std::cout << "cloud_in has " << cloud_in->size() << " points." << std::endl;

	cv::Mat color_image(cloud_in->height,cloud_in->width,CV_8UC3);
	ef.getColorImage(cloud_in, color_image);
	cv::Mat canny_image;
	ef.extractEdgesCanny(color_image, canny_image);
	//pcl::io::savePCDFileASCII (directory+"/edges/edges_canny.pcd", *cloud_out);

	/*cloud_out = PointCloud::Ptr (new PointCloud);
	cv::Mat border_image;
	ef.extractEdgesBoundary(cloud_in, *cloud_out, border_image);
	pcl::io::savePCDFileASCII (directory+"/edges/edges_boundary.pcd", *cloud_out);
	cv::imshow("Boundary image (angle)", border_image);
	cv::waitKey();*/
	//pcl::io::savePCDFileASCII (directory+"/edges/edge_map.pcd", pc_edge);

	/*cv::Mat complete_edge_image = max(canny_image, border_image);
	cv::imshow("Edge image", complete_edge_image);
	cv::waitKey();*/

	/*PointCloud cloud_out_marked;
	cloud_out_marked.header = cloud_in->header;
	cloud_out_marked.points.resize(cloud_in->points.size());
	int nr_p = 0;
	for (int i=0; i<cloud_in->size(); i++)
	{
		if(pc_edge.points[i].boundary_point >= 2)
			cloud_out_marked.points[nr_p++] = cloud_in->points[i];
	}
	cloud_out_marked.width = nr_p;
	cloud_out_marked.height = 1;
	cloud_out_marked.points.resize(nr_p);
	cloud_out_marked.is_dense = true;
	pcl::io::savePCDFileASCII (directory+"/edges/edge_map.pcd", cloud_out_marked);*/

	cloud_out = PointCloud::Ptr (new PointCloud);
	ef.extractEdgesCurvature(cloud_in, *cloud_out);
	pcl::io::savePCDFileASCII (directory+"/edges/edges_curvature.pcd", *cloud_out);

	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n_out;
	ef.estimatePointNormals(*cloud_in, cloud_n_out);
	pcl::io::savePCDFileASCII (directory+"/frame_normals.pcd", cloud_n_out);

	cv::Mat border_image;
	IPARangeImage range_image;
	ef.extractEdgesRangeImage2(cloud_in, range_image, border_image);
	cv::Mat combined_edge_image = canny_image | border_image;
	cv::imshow("combined edge image", combined_edge_image);
	cv::waitKey();

	cv::Mat wshed_canny;
	ef.segmentByEdgeImage(color_image, canny_image, wshed_canny);
	//cv::imshow("watershed canny", water_shed_canny);
	cv::Mat wshed_combined;
	ef.segmentByEdgeImage(color_image, combined_edge_image, wshed_combined);
	//cv::imshow("watershed combined", water_shed_combined);
	//cv::waitKey();
	std::vector<pcl::PointIndices> cluster_indices;
	ef.getClusterIndices(cloud_in, wshed_canny, cluster_indices);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	for(unsigned int i = 0; i < cluster_indices.size(); i++)
	{
		if(cluster_indices[i].indices.size()>100)
		{
			pcl::PointCloud<pcl::PointXYZRGB> cluster;
			extract.setInputCloud (cloud_in);
			extract.setIndices (boost::make_shared<const pcl::PointIndices> (cluster_indices[i]));
			extract.setNegative (false);
			extract.filter (cluster);
			stringstream ss;//create a stringstream
			ss << directory << "/cluster/cluster_" << i << ".pcd";//add number to the stream
			pcl::io::savePCDFileASCII (ss.str(), cluster);
		}
	}
	//pcl::io::savePCDFileASCII (directory+"/range_image.pcd", range_image);
	//ros::spin();

	return 0;
}



