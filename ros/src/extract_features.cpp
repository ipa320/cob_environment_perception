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
#include <pcl/range_image/range_image.h>
#include <pcl_visualization/cloud_viewer.h>
#include <pcl/features/range_image_border_extractor.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

//#include <cob_vision_features/SURFDetector.h>
//#include <cob_vision_features/AbstractFeatureVector.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


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

    /*void pointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
	{
    	ROS_INFO("[env_model_node] Detecting features");

    	cv::Mat color_image(pc->height,pc->width,CV_8UC3);
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
    	ipa_Features::AbstractLocalFeatureDetector* feature_detector = new ipa_Features::SURFDetector();
    	((SURFDetector*)feature_detector)->Init(500);
    	ipa_Features::AbstractFeatureVectorPtr feature_vector = AbstractFeatureVectorPtr(new AbstractFeatureVector());
		feature_detector->DetectFeatures(color_image, *feature_vector);

		pc_pt_idx=0;
    	for (int row = 0; row < color_image.rows; row++)
    	{
    		c_ptr = color_image.ptr<unsigned char>(row);
    		for (int col = 0; col < color_image.cols; col++, pc_pt_idx++)
    		{
				AbstractFeatureVector::iterator It;
				for (It=feature_vector->begin(); It!=feature_vector->end(); It++)
				{
					if((*It)->Get<double>(M_U) == col && (*It)->Get<double>(M_V) == row)
					{
						//pc->points[pc_pt_idx].isFeature = 1;
						feature_vector->erase(It);
						break;
					}
				}
    		}
    	}
    	point_cloud_pub_.publish(pc);
	}*/

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
		normalEstimator.setRadiusSearch(0.02);
		//normalEstimator.setNumberOfThreads(8);
		pcl::PointCloud<pcl::Normal> cloud_normal;
		boost::timer t;
		normalEstimator.compute(cloud_normal);

		ROS_INFO("Time elapsed for normal estimation: %f", t.elapsed());
		pcl::concatenateFields (cloud, cloud_normal, cloud_n);
		return;
	}

    void extractEdgesCanny(PointCloud::Ptr& pc, PointCloud& pc_out, cv::Mat& canny_image)
	{
    	ROS_INFO("[env_model_node] Detecting features");
    	//PointCloud pc_out;
        pc_out.points.resize (pc->points.size ());
        pc_out.header = pc->header;
        /*pc_edge.points.resize (pc->points.size ());
        pc_edge.header = pc->header;
        pc_edge.width = pc->width;
        pc_edge.height = pc->height;
        pc_edge.is_dense = true;*/
        int nr_p = 0;

        boost::timer t;
    	cv::Mat color_image(pc->height,pc->width,CV_8UC3);
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
    	t.restart();
    	cv::Mat canny_morph;
    	cv::morphologyEx(canny_image, canny_morph, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)), cv::Point(-1,-1), 1);
    	cv::imshow("Canny morph", canny_morph);
    	vector<vector<cv::Point> > contours;
    	cv::findContours(canny_morph, contours, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    	cv::Mat markers = cv::Mat::zeros(color_image.size(), CV_32S);
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
    	cv::Mat wshed(markers.size(), CV_8UC3);
    	for(int i = 0; i < markers.rows; i++ )
    	{
    		for(int j = 0; j < markers.cols; j++ )
    		{
    			int idx = markers.at<int>(i,j);
    			if( idx == -1 )
    				wshed.at<cv::Vec3b>(i,j) = cv::Vec3b(255,255,255);
    			else
    				wshed.at<cv::Vec3b>(i,j) = colorTab[idx - 1];
    		}
    	}
    	imshow( "watershed transform", wshed );
    	cv::waitKey();
        //cvtColor( canny_image, color_canny_image, CV_GRAY2BGR );
        c_ptr = 0;
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
        pc_out.is_dense = true;
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
		border_image = cv::Mat(cloud_in->height, cloud_in->width, CV_8UC1);
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
			if( cloud_n->points[i].curvature > 0.2)
				cloud_out.points[nr_p++] = cloud_in->points[i];
		}

		cloud_out.width = nr_p;
		cloud_out.height = 1;
		cloud_out.points.resize(nr_p);
		cloud_out.is_dense = true;
		ROS_INFO("Time elapsed for boundary estimation (curvature): %f", t.elapsed());
	}

	void extractEdgesRangeImage(PointCloud::Ptr& cloud_in, pcl::RangeImage& cloud_out)
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
		  for (int y=0; y<(int)cloud_out.height; ++y)
		  {
		    for (int x=0; x<(int)cloud_out.width; ++x)
		    {
		      if (border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
		        border_points.points.push_back(cloud_out.points[y*cloud_out.width + x]);
		      if (border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
		        veil_points.points.push_back(cloud_out.points[y*cloud_out.width + x]);
		      if (border_descriptions.points[y*cloud_out.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
		        shadow_points.points.push_back(cloud_out.points[y*cloud_out.width + x]);
		    }
		  }
			pcl_visualization::PCLVisualizer viewer("3D Viewer");
			  viewer.addCoordinateSystem(1.0f);
			  viewer.addPointCloud(cloud_out, "original point cloud");
		  viewer.addPointCloud(border_points, "border points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "border points");
		  viewer.addPointCloud(veil_points, "veil points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "veil points");
		  viewer.addPointCloud(shadow_points, "shadow points");
		  viewer.setPointCloudRenderingProperties(pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 1, "shadow points");

		  /*std::string directory("/home/goa/pcl_daten/kitchen_sf/");
			pcl::io::savePCDFileASCII (directory+"/edges/ri_border_pts.pcd", border_points);
			pcl::io::savePCDFileASCII (directory+"/edges/ri_shadow_pts.pcd", shadow_points);*/

		  while(!viewer.wasStopped())
		  {
		    viewer.spinOnce(100);
		    usleep(100000);
		  }



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

	std::string directory("/home/goa/pcl_daten/kitchen_kinect/");
	PointCloud::Ptr cloud_in = PointCloud::Ptr (new PointCloud);
	pcl::io::loadPCDFile(directory+"shelves_close.pcd", *cloud_in);
	std::cout << "cloud_in has " << cloud_in->size() << " points." << std::endl;
	PointCloud::Ptr cloud_out = PointCloud::Ptr (new PointCloud);
	cv::Mat canny_image;
	ef.extractEdgesCanny(cloud_in, *cloud_out, canny_image);
	pcl::io::savePCDFileASCII (directory+"/edges/edges_canny.pcd", *cloud_out);

	/*cloud_out = PointCloud::Ptr (new PointCloud);
	cv::Mat border_image;
	ef.extractEdgesBoundary(cloud_in, *cloud_out, border_image);
	pcl::io::savePCDFileASCII (directory+"/edges/edges_boundary.pcd", *cloud_out);
	//pcl::io::savePCDFileASCII (directory+"/edges/edge_map.pcd", pc_edge);

	cv::Mat complete_edge_image = max(canny_image, border_image);
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

	/*cloud_out = PointCloud::Ptr (new PointCloud);
	ef.extractEdgesCurvature(cloud_in, *cloud_out);
	pcl::io::savePCDFileASCII (directory+"/edges/edges_curvature.pcd", *cloud_out);*/

	/*pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n_out;
	ef.estimatePointNormals(*cloud_in, cloud_n_out);
	pcl::io::savePCDFileASCII (directory+"/shelves_close_n.pcd", cloud_n_out);*/

	pcl::RangeImage range_image;
	ef.extractEdgesRangeImage(cloud_in, range_image);
	pcl::io::savePCDFileASCII (directory+"/range_image.pcd", range_image);
	//ros::spin();

	return 0;
}



