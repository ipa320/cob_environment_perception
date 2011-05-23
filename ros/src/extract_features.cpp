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

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

#include <cob_vision_features/SURFDetector.h>
#include <cob_vision_features/AbstractFeatureVector.h>


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
		normalEstimator.setRadiusSearch(0.03);
		//normalEstimator.setNumberOfThreads(8);
		pcl::PointCloud<pcl::Normal> cloud_normal;
		boost::timer t;
		normalEstimator.compute(cloud_normal);

		ROS_INFO("Time elapsed for normal estimation: %f", t.elapsed());
		pcl::concatenateFields (cloud, cloud_normal, cloud_n);
		return;
	}

    void extractEdgesCanny(PointCloud::Ptr& pc, PointCloud& pc_out, pcl::PointCloud<pcl::Boundary>& pc_edge)
	{
    	ROS_INFO("[env_model_node] Detecting features");
    	//PointCloud pc_out;
        pc_out.points.resize (pc->points.size ());
        pc_out.header = pc->header;
        pc_edge.points.resize (pc->points.size ());
        pc_edge.header = pc->header;
        pc_edge.width = pc->width;
        pc_edge.height = pc->height;
        pc_edge.is_dense = true;
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
    	cv::Mat canny_image, color_canny_image;
        Canny( grey_image, canny_image, 50, 200, 3 );
    	ROS_INFO("Time elapsed for canny edge: %f", t.elapsed());
    	t.restart();
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
    				pc_edge.points[pt_idx].boundary_point = 1;
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

	void extractEdgesBoundary(PointCloud::Ptr& cloud_in, PointCloud& cloud_out, pcl::PointCloud<pcl::Boundary>& pc_edge)
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
				pc_edge.points[i].boundary_point++;
				cloud_out.points[nr_p++] = cloud_in->points[i];
			}
		}
		cloud_out.width = nr_p;
		cloud_out.height = 1;
		cloud_out.points.resize(nr_p);
		cloud_out.is_dense = true;
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

	std::string directory("/home/goa/pcl_daten/kitchen_sf/");
	PointCloud::Ptr cloud_in = PointCloud::Ptr (new PointCloud);
	pcl::io::loadPCDFile(directory+"1303291800.731377681.pcd", *cloud_in);
	std::cout << "cloud_in has " << cloud_in->size() << " points." << std::endl;
	PointCloud::Ptr cloud_out = PointCloud::Ptr (new PointCloud);
	ef.extractEdgesCanny(cloud_in, *cloud_out, pc_edge);
	pcl::io::savePCDFileASCII (directory+"/edges/edges_canny.pcd", *cloud_out);

	cloud_out = PointCloud::Ptr (new PointCloud);
	ef.extractEdgesBoundary(cloud_in, *cloud_out, pc_edge);
	pcl::io::savePCDFileASCII (directory+"/edges/edges_boundary.pcd", *cloud_out);
	pcl::io::savePCDFileASCII (directory+"/edges/edge_map.pcd", pc_edge);

	PointCloud cloud_out_marked;
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
	pcl::io::savePCDFileASCII (directory+"/edges/edge_map.pcd", cloud_out_marked);

	/*cloud_out = PointCloud::Ptr (new PointCloud);
	ef.extractEdgesCurvature(cloud_in, *cloud_out);
	pcl::io::savePCDFileASCII (directory+"/edges/edges_curvature.pcd", *cloud_out);*/

	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n_out;
	ef.estimatePointNormals(*cloud_in, cloud_n_out);
	pcl::io::savePCDFileASCII (directory+"/normals.pcd", cloud_n_out);
	//ros::spin();

	return 0;
}


