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
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
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
#include <tf_conversions/tf_kdl.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv.h>
#include <highgui.h>
#include "pcl/io/pcd_io.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/normal_3d.h"
#include <cob_env_model/features/boundary.h>
#include <pcl/features/principal_curvatures.h>
//#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PointIndices.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/range_image/range_image.h>
#include <cob_env_model/features/range_image_border_extractor.h>
#include <pcl/features/integral_image_normal.h>
#include <cob_env_model/point_types.h>
#include "pcl/filters/extract_indices.h"
// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

//#include <cob_vision_features/SURFDetector.h>
//#include <cob_vision_features/AbstractFeatureVector.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;
typedef pcl::PointXYZRGB PointT;

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
		normalEstimator.setInputCloud(boost::make_shared<PointCloudT >(cloud));
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

/**Extracts edges from a color image using the canny algorithm*/
    void extractEdgesCanny(cv::Mat& color_image, cv::Mat& canny_image)
{
        boost::timer t;
     cv::Mat grey_image;
     cvtColor( color_image, grey_image, CV_RGB2GRAY );
        cv::Mat opening_image;
        cv::Mat element=cv::Mat::ones(3,3, CV_32F);
        double m[9][9]={{0,0,0,1,1,1,0,0,0},{0,0,0,1,1,1,0,0,0},{0,0,0,1,1,1,0,0,0},{1,1,1,1,1,1,1,1,1},{1,1,1,1,1,1,1,1,1},{1,1,1,1,1,1,1,1,1},{0,0,0,1,1,1,0,0,0},{0,0,0,1,1,1,0,0,0},{0,0,0,1,1,1,0,0,0}};
        cv::Mat edges=cv::Mat(9,9,CV_64F,m).inv();
        cv::Point anchor(-1,-1);
        int borderType=cv::BORDER_CONSTANT;
        cv::Scalar borderValue=cv::morphologyDefaultBorderValue();
        cv::erode(grey_image, grey_image,element,anchor,1,borderType,borderValue);
        //cv::dilate(grey_image, grey_image,element,anchor,4,borderType,borderValue);

     cv::Mat color_canny_image;
        Canny( grey_image, canny_image, 25, 150, 3 );
     ROS_INFO("Time elapsed for canny edge: %f", t.elapsed());
}

    /**Uses the PCL BoundaryEstimation to find edges in a point cloud. Uses an angle criterion to detect edges
     * Unfortunately marks the outline of a point cloud as edge; needs normal estimation
     */
	void extractEdgesBoundary(PointCloudT::Ptr& cloud_in, pcl::PointCloud<pcl::Boundary>& cloud_out, cv::Mat& border_image)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
		//pcl::PointCloud<pcl::Boundary>::Ptr boundary_pts (new pcl::PointCloud<pcl::Boundary> ());
		estimatePointNormals(*cloud_in, *cloud_n);

		boost::timer t;
		//pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree;
		pcl::OrganizedDataIndex<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::OrganizedDataIndex<pcl::PointXYZRGBNormal> ());
		//tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> > ();
		ipa_features::BoundaryEstimation<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal,pcl::Boundary> boundary;
		boundary.setSearchMethod(tree);
		boundary.setInputCloud(cloud_n);
		//boundary.setSearchSurface (cloud);
		boundary.setRadiusSearch(0.03);
		boundary.setInputNormals(cloud_n);
		boundary.dist_threshold_ = 0.02; //increase to get more border points
		boundary.compute(cloud_out);
		//pcl::PointCloud<pcl::PointXYZRGBNormal> boundary_cloud;
		//cloud_out.points.resize(cloud_n->points.size());
		//cloud_out.header = cloud_n->header;
		//int nr_p = 0;

		//std::cout << "size: " << boundary_pts->size() << std::endl;

		/*for( unsigned int i = 0; i < cloud_in->points.size(); i++)
		{
			if( cloud_out.points[i].boundary_point == 1)
			{
				//pc_edge.points[i].boundary_point++;
				cloud_out.points[nr_p++] = cloud_in->points[i];
			}
		}
		cloud_out.width = nr_p;
		cloud_out.height = 1;
		cloud_out.points.resize(nr_p);
		cloud_out.is_dense = true;*/

		border_image = cv::Mat(cloud_in->height, cloud_in->width, CV_8UC1);
		int pt_idx=0;
		for(unsigned int row=0; row<border_image.rows; row++)
		{
			for(unsigned int col=0; col<border_image.cols; col++, pt_idx++)
			{
				if( cloud_out.points[pt_idx].boundary_point == 1)
					border_image.at<unsigned char>(row,col) = 255;
				else
					border_image.at<unsigned char>(row,col) = 0;
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

	    int cur_label = 2;
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
	        		Coords c(j,i);

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
	        				wave_coords.push_back(Coords(c.u+1,c.v)/*Eigen::Vector2d(uv(0)+1,uv(1))*/);
	        			}
	        			if( c.u/*uv(0)*/ > 0 && cloud_in->points[pt_ctr-1].label==0)
	        			{
	        				cloud_in->points[pt_ctr-1].label = cur_label;
	        				//wave.push_back(&cloud_in->points[pt_ctr-1]);
	        				wave_coords.push_back(Coords(c.u-1,c.v)/*Eigen::Vector2d(uv(0)-1,uv(1))*/);
	        			}
	        			//std::cout << pt_ctr+width << ": "<< (int)(cloud_in->points[pt_ctr+width].boundary_point) << std::endl;
	        			if(c.v/*uv(1)*/ < height-1 && cloud_in->points[pt_ctr+width].label==0)
	        			{
	        				cloud_in->points[pt_ctr+width].label = cur_label;
	        				//wave.push_back(&cloud_in->points[pt_ctr+width]);
	        				wave_coords.push_back(Coords(c.u,c.v+1)/*Eigen::Vector2d(uv(0),uv(1)+1)*/);
	        			}
	        			if(c.v/*uv(1)*/ > 0 && cloud_in->points[pt_ctr-width].label==0)
	        			{
	        				cloud_in->points[pt_ctr-width].label = cur_label;
	        				//wave.push_back(&cloud_in->points[pt_ctr-width]);
	        				wave_coords.push_back(Coords(c.u,c.v-1)/*Eigen::Vector2d(uv(0),uv(1)-1)*/);
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
	void extractEdgesCurvature(PointCloudT::Ptr& cloud_in, PointCloudT& cloud_out)
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
			cv::imshow("range border image", border_image);
			cv::waitKey();
	}

    /**Uses the PCL BoundaryEstimation to find edges in a point cloud. Uses an angle criterion to detect edges
     * Unfortunately marks the outline of a point cloud as edge; needs normal estimation
     */
	void extractPrincipalCurvature(PointCloudT::Ptr& cloud_in, PointCloudT& cloud_out)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curv_pts (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
		estimatePointNormals(*cloud_in, *cloud_n);

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


		ROS_INFO("Time elapsed for boundary estimation: %f", t.elapsed());
	}

	/**Segments a color image using an edge image and the watershed algorithm
	 *
	 */
	void segmentByEdgeImage(cv::Mat& color_image, cv::Mat& edge_image, cv::Mat& markers,vector<vector<cv::Point> >& big_contours)
	    {
	        /// apply closing to connect edge segments, not working very well
	        cv::Mat edge_morph;

	        //cv::morphologyEx(edge_image, edge_morph, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)), cv::Point(-1,-1), 1);
	        //cv::morphologyEx(edge_image, edge_morph, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)), cv::Point(-1,-1), 1);
	       // cv::imshow("Edge morph", edge_morph);
	        /// find contours in edge image
	        vector<vector<cv::Point> > contours;
	        //vector<vector<cv::Point> > big_contours;


	        // all contours
	        cv::findContours(edge_image, contours, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
	        //only extrem contours
	        //cv::findContours(edge_morph, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	        markers = cv::Mat::zeros(edge_image.size(), CV_32S);


	        for(int i=0; i<contours.size();i++){
	                  if (contours[i].size()>50){
	                    vector<cv::Point> inside;
	                    contours[i].swap(inside);
	                    big_contours.push_back(inside) ;
	                  }
	                }
	        ExtractFeatures ef;
	        int contours_size=big_contours.size();
	        ef.vis_markers(big_contours, markers);
	        ef.watershed(color_image,markers, contours_size);


//	        for(int idx=0; idx<big_contours.size(); idx++)
//
//	        cv::drawContours(markers, big_contours, idx, cv::Scalar(idx+1));
//	        vector<cv::Vec3b> colorTab;
//	        for(int i = 0; i < big_contours.size(); i++ )
//	        {
//	            int b = cv::theRNG().uniform(0, 255);
//	            int g = cv::theRNG().uniform(0, 255);
//	            int r = cv::theRNG().uniform(0, 255);
//
//	            colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
//	        }
//	        cv::Mat vis_markers(markers.size(), CV_8UC3);
//	        for(int i = 0; i < markers.rows; i++ )
//	        {
//	            for(int j = 0; j < markers.cols; j++ )
//	            {
//	                int idx = markers.at<int>(i,j);
//	                if( idx == 0 )
//	                    vis_markers.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
//	                else
//	                    vis_markers.at<cv::Vec3b>(i,j) = colorTab[idx - 1];
//	            }
//	        }
//	        cv::imshow("Contours", vis_markers);
//	        cv::waitKey();
	        /// apply watershed algorithm
//	        cv::watershed(color_image, markers);
//
//	        cv::Mat wshed_image=cv::Mat(markers.size(), CV_8UC3);
//	        for(int i = 0; i < markers.rows; i++ )
//	        {
//	            for(int j = 0; j < markers.cols; j++ )
//	            {
//	                int idx = markers.at<int>(i,j);
//	                if( idx == -1 )
//	                    wshed_image.at<cv::Vec3b>(i,j) = cv::Vec3b(255,255,255);
//	                else
//	                    wshed_image.at<cv::Vec3b>(i,j) = colorTab[idx - 1];
//	            }
//	        }
//	        cv::imshow("wshed image", wshed_image);
//	        cv::waitKey();
	    //    markers=vis_markers;

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



void watershed(cv::Mat &color_image , cv::Mat &combined_wshed_image , int &contours_size ){

    cv::watershed(color_image, combined_wshed_image);

	vector<cv::Vec3b> colorTab;

	for(int i = 0; i < contours_size; i++ )
	{
	int b = cv::theRNG().uniform(0, 255);
	int g = cv::theRNG().uniform(0, 255);
	int r = cv::theRNG().uniform(0, 255);

	colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
	}
	cv::Mat wshed_image=cv::Mat(combined_wshed_image.size(), CV_8UC3);
	for(int i = 0; i < combined_wshed_image.rows; i++ )
	{
	for(int j = 0; j < combined_wshed_image.cols; j++ )
	{
	int idx = combined_wshed_image.at<int>(i,j);
	if( idx == -1 )
	wshed_image.at<cv::Vec3b>(i,j) = cv::Vec3b(255,255,255);
	else
	wshed_image.at<cv::Vec3b>(i,j) = colorTab[idx - 1];
	}



	}
	cv::imshow("wshed image", wshed_image);




	cv::waitKey();

}

void vis_markers(vector<vector<cv::Point> > big_contours ,cv::Mat &markers ){


    for(int idx=0; idx<big_contours.size(); idx++)

    cv::drawContours(markers, big_contours, idx, cv::Scalar(idx+1));
    vector<cv::Vec3b> colorTab;
    for(int i = 0; i < big_contours.size(); i++ )
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
}

void add2contours(vector<vector<cv::Point> > &input_contour1 ,vector<vector<cv::Point> > &input_contour2 , vector<vector<cv::Point> > &output_contour)
{
	for(int i=0 ; i<input_contour1.size();i++){

	output_contour.push_back(input_contour1[i]);
	}
	for(int i=0 ; i<input_contour2.size();i++){
	output_contour.push_back(input_contour2[i]);
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

/// Create extract features class instance
ExtractFeatures ef;

/// Load PCD file as input; better use binary PCD files, ascii files seem to generate corrupt point clouds
std::string directory("/home/goa-hh/pcl_daten/test/");
PointCloud::Ptr cloud_in = PointCloud::Ptr (new PointCloud);
int key=0;
std::cout <<"Bitte wähle den Speicherslot 1,2 oder 3"<<std::endl;
std:cin >>key;
switch(key){
case 1 : pcl::io::loadPCDFile(directory+"karton2_bin.pcd", *cloud_in);break;
case 2 : pcl::io::loadPCDFile(directory+"eingang_bin.pcd", *cloud_in);break;
case 3 : pcl::io::loadPCDFile(directory+"schreibtisch_bin.pcd", *cloud_in);break;
}
/// Extract edges on the color image
cv::Mat color_image(cloud_in->height,cloud_in->width,CV_8UC3);
ef.getColorImage(cloud_in, color_image);
cv::imshow("Color Image", color_image);
cv::Mat canny_image;
ef.extractEdgesCanny(color_image, canny_image);
        cv::imshow("Canny Image", canny_image);
        cv::waitKey();

PointCloud cloud_out;
cv::Mat border_image;

    /// Extract edges using curvature
/*ef.extractEdgesCurvature(cloud_in, cloud_out);
pcl::io::savePCDFileASCII (directory+"/edges/edges_curvature.pcd", cloud_out);*/

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
	ef.estimatePointNormals(*cloud_in, *cloud_n);
	pcl::io::savePCDFileASCII (directory+"/corner_n.pcd", *cloud_n);
    /// Extract edges using curvature
/*ef.extractEdgesCurvature(cloud_in, cloud_out);
pcl::io::savePCDFileASCII (directory+"/edges/edges_curvature.pcd", cloud_out);*/



//Extract edges using boundary estimation
// ef.extractEdgesBoundary(cloud_in, cloud_out, border_image);
// pcl::io::savePCDFileASCII (directory+"/edges/edges_boundary.pcd", cloud_out);

// /// Extract edges using range image border extraction
pcl::RangeImage range_image_out;
ef.extractEdgesRangeImage(cloud_in, range_image_out, border_image);


cv::Mat element=cv::Mat::ones(3,3, CV_32F);
cv::Mat border_image2;
cv::Point anchor(-1,-1);
int borderType=cv::BORDER_CONSTANT;
cv::Scalar borderValue=cv::morphologyDefaultBorderValue();
//cv::dilate(border_image, border_image,element,anchor,1,borderType,borderValue);
cv::erode(border_image, border_image,element,anchor,1,borderType,borderValue);

        // cv::dilate(border_image, border_image,element,anchor,1,borderType,borderValue);
          //cv::erode(border_image, border_image,element,anchor,1,borderType,borderValue);


        cv::imshow("border_image", border_image);


pcl::io::savePCDFileASCII (directory+"/edges/edges_range_border.pcd", range_image_out);
pcl::visualization::PCLVisualizer viewer("3D Viewer");
viewer.addCoordinateSystem(1.0f);
viewer.addPointCloud<pcl::PointXYZRGB>(cloud_in, "original point cloud");
viewer.addPointCloud<pcl::PointWithRange>(range_image_out.makeShared(), "border points");
while(!viewer.wasStopped())
{
viewer.spinOnce(100);
usleep(100000);
}

/// Combine two edge images
cv::Mat combined_edge_image = canny_image | border_image;
cv::imshow("combined edge image", combined_edge_image);
cv::waitKey();

/// Segment color image using canny edge image (can also be done with combined edge image)
cv::Mat wshed_canny;
cv::Mat wshed_range_image;
cv::Mat wshed_combined_image;

vector<vector<cv::Point> > big_contours_canny;
vector<vector<cv::Point> > big_contours_range;
vector<vector<cv::Point> > big_contours_combined;

ef.segmentByEdgeImage(color_image, canny_image, wshed_canny,big_contours_canny );
ef.segmentByEdgeImage(color_image, border_image, wshed_range_image,big_contours_range);
ef.segmentByEdgeImage(color_image, combined_edge_image, wshed_combined_image,big_contours_combined );


cv::Mat combined_wshed_image=wshed_canny | wshed_range_image;
cv::Mat all_image = canny_image | border_image;
int contours_size=big_contours_canny.size()+big_contours_range.size();

vector<vector<cv::Point> > all_contours;
cv::Mat markers2 = cv::Mat::zeros(all_image.size(), CV_32S);

ef.add2contours(big_contours_canny , big_contours_range , all_contours);
ef.vis_markers(all_contours,markers2);
ef.watershed(color_image , markers2 , contours_size);
//cv::watershed(color_image, combined_wshed_image);
//
//
//vector<cv::Vec3b> colorTab;
//
//for(int i = 0; i < big_contours_canny.size()+big_contours_range.size(); i++ )
//{
//int b = cv::theRNG().uniform(0, 255);
//int g = cv::theRNG().uniform(0, 255);
//int r = cv::theRNG().uniform(0, 255);
//
//colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
//}
//cv::Mat wshed_image=cv::Mat(combined_wshed_image.size(), CV_8UC3);
//for(int i = 0; i < combined_wshed_image.rows; i++ )
//{
//for(int j = 0; j < combined_wshed_image.cols; j++ )
//{
//int idx = combined_wshed_image.at<int>(i,j);
//if( idx == -1 )
//wshed_image.at<cv::Vec3b>(i,j) = cv::Vec3b(255,255,255);
//else
//wshed_image.at<cv::Vec3b>(i,j) = colorTab[idx - 1];
//}
//
//
//
//}
//cv::imshow("wshed image", wshed_image);
//
//
//
//
//cv::waitKey();

/// Cluster point cloud according to color image segmentation
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
stringstream ss; //create a stringstream
//ss << directory << "/cluster/cluster_" << i << ".pcd";//add number to the stream
//pcl::io::savePCDFileASCII (ss.str(), cluster);
}
}

return 0;
}




