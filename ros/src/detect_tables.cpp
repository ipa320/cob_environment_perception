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
 * Date of creation: 02/2011
 * ToDo:
 * Nodelet code
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


//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### nodelet class ####
class DetectTables : public pcl_ros::PCLNodelet
{
public:
    // Constructor
	DetectTables()
	{
	}

    // Destructor
    ~DetectTables()
    {
    	/// void
    }


    void onInit()
    {
    	PCLNodelet::onInit();
    	n_ = getNodeHandle();

		point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &DetectTables::pointCloudSubCallback, this);
		table_marker_pub_ = n_.advertise<visualization_msgs::Marker>("table_marker",100);
		convex_hull_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("table_hull",1);
		object_cluster_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("object_cluster",1);
    }


    //input should be point cloud that is amplitude filetered, statistical outlier filtered, voxel filtered and the floor cut, coordinate system should be /map
	void pointCloudSubCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
	{
		std::fstream filestr;
		filestr.open("/home/goa/pcl_daten/table/table_detection/meas.csv", std::fstream::in | std::fstream::out | std::fstream::app);
		boost::timer t;
		//static int ctr = 0;
		pcl::KdTree<pcl::PointXYZ>::Ptr clusters_tree;
		clusters_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;

		// Table clustering parameters
		cluster.setClusterTolerance (0.06);
		cluster.setMinClusterSize (300);
		cluster.setSearchMethod (clusters_tree);

		// Cluster potential table points
		std::vector<pcl::PointIndices> table_clusters;
		cluster.setInputCloud (cloud);
		cluster.extract (table_clusters);

		//ROS_INFO ("Number of table clusters found: %d", (int)table_clusters.size ());

		pcl::ExtractIndices<pcl::PointXYZ> extract;

		for(unsigned int i = 0; i < table_clusters.size(); ++i)
		{
			pcl::PointCloud<pcl::PointXYZ> table_cluster;
			extract.setInputCloud (cloud);
			extract.setIndices (boost::make_shared<const pcl::PointIndices> (table_clusters[i]));
			extract.setNegative (false);
			extract.filter (table_cluster);
			/*std::stringstream ss1;
			ss1 << "/home/goa/pcl_daten/table_detection/cluster_" << i << ".pcd";
			pcl::io::savePCDFileASCII (ss1.str(), table_cluster);*/

			pcl::PointCloud<pcl::PointXYZ>::ConstPtr table_cluster_ptr = boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> > (table_cluster);

			if (table_cluster_ptr->points.size() < (unsigned int)300)
			{
				//ROS_INFO("Table cluster only has %d points, skipping cluster", (int)table_cluster_ptr->points.size());
				continue;
			}

			pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimator;
			normalEstimator.setInputCloud(table_cluster_ptr);
			pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
			normalEstimator.setSearchMethod(tree);
			normalEstimator.setKSearch(30);
			//normalEstimator.setNumberOfThreads(4);
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal> ());
			normalEstimator.compute(*cloud_normals);

			pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

			// Create the segmentation object for the planar model and set all the parameters
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
			seg.setNormalDistanceWeight (0.1);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (100);
			seg.setDistanceThreshold (0.03);
			seg.setInputCloud (table_cluster_ptr);
			seg.setInputNormals (cloud_normals);
			// Obtain the plane inliers and coefficients
			pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
			pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ());
			seg.segment (*inliers_plane, *coefficients_plane);
			//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

			if (coefficients_plane->values.size () <=3)
			{
				//ROS_INFO("Failed to detect table in scan, skipping cluster");
				continue;
			}
			if ( inliers_plane->indices.size() < (unsigned int)150)
			{
				//ROS_INFO("Plane detection has %d inliers, below min threshold of %d, skipping cluster", (int)inliers_plane->indices.size(), 150);
				continue;
			}
			if(fabs(coefficients_plane->values[0]) > 0.1 || fabs(coefficients_plane->values[1]) > 0.1 || fabs(coefficients_plane->values[2]) < 0.9)
			{
				//ROS_INFO("Detected plane not perpendicular to z axis, skipping cluster");
				continue;
			}


			pcl::PointCloud<pcl::PointXYZ> dominant_plane;
			pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
			extractIndices.setInputCloud(table_cluster_ptr);
			extractIndices.setIndices(inliers_plane);
			extractIndices.filter(dominant_plane);
			//extractIndices.setNegative(true);
			//extractIndices.filter(cloud);
			//ROS_INFO("Plane has %d inliers", (int)inliers_plane->indices.size());
			//ROS_INFO("Saved plane to %s", ss.str());

			  // Project the model inliers
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::ProjectInliers<pcl::PointXYZ> proj;
			proj.setModelType (pcl::SACMODEL_PLANE);
			proj.setInputCloud (table_cluster_ptr);
			proj.setModelCoefficients (coefficients_plane);
			proj.filter (*cloud_projected);

			/*std::stringstream ss;
			ss << "/home/goa/pcl_daten/table_detection/plane_" << ctr << ".pcd";
			pcl::io::savePCDFileASCII (ss.str(), *cloud_projected);
			ctr++;*/

			// Create a Convex Hull representation of the projected inliers
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ> ());
			std::vector< pcl::Vertices > hull_polygon;
			pcl::ConvexHull<pcl::PointXYZ> chull;
			chull.setInputCloud (cloud_projected);
			chull.reconstruct (*cloud_hull, hull_polygon);

			pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
			// Consider only objects in a given layer above the table
			prism.setHeightLimits (-0.5, -0.03);
			// ---[ Get the objects on top of the table
			pcl::PointIndices cloud_object_indices;
			prism.setInputCloud (table_cluster_ptr);
			prism.setInputPlanarHull (cloud_hull);
			prism.segment (cloud_object_indices);

			pcl::PointCloud<pcl::PointXYZ> cloud_objects;
			pcl::ExtractIndices<pcl::PointXYZ> extract_object_indices;
			extract_object_indices.setInputCloud (table_cluster_ptr);
			extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
			extract_object_indices.filter (cloud_objects);
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_objects_ptr = cloud_objects.makeShared();

			pcl::KdTree<pcl::PointXYZ>::Ptr obj_clusters_tree;
			obj_clusters_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();

			pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_obj;

			// Table clustering parameters
			cluster_obj.setClusterTolerance (0.06);
			cluster_obj.setMinClusterSize (100);
			cluster_obj.setSearchMethod (clusters_tree);

			// Cluster potential table points
			std::vector<pcl::PointIndices> object_clusters;
			cluster_obj.setInputCloud (cloud_objects.makeShared());
			cluster_obj.extract (object_clusters);

			for(unsigned int i = 0; i < object_clusters.size(); ++i)
			{
				pcl::PointCloud<pcl::PointXYZ> object;
				extract.setInputCloud (cloud_objects.makeShared());
				extract.setIndices (boost::make_shared<const pcl::PointIndices> (object_clusters[i]));
				extract.setNegative (false);
				extract.filter (object);
				object_cluster_pub_.publish(object);
			}


			/*std::stringstream ss;
			ss << "/home/goa/pcl_daten/table_detection/hull_" << ctr << ".pcd";
			pcl::io::savePCDFileASCII (ss.str(), *cloud_hull);
			ctr++;*/

			//ROS_INFO ("Convex hull has: %zu data points.", cloud_hull->points.size ());

			visualization_msgs::Marker marker;
			marker.action = visualization_msgs::Marker::ADD;
			marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
			marker.lifetime = ros::Duration();
			marker.header.frame_id = cloud->header.frame_id;

			convex_hull_pub_.publish(cloud_hull);

			//create the marker in the table reference frame
			//the caller is responsible for setting the pose of the marker to match

			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 1;

			geometry_msgs::Point pt1, pt2, pt3;
			pt1.x = cloud_hull->points[0].x;
			pt1.y = cloud_hull->points[0].y;
			pt1.z = cloud_hull->points[0].z;

			for(unsigned int i = 1; i < cloud_hull->points.size()-1; ++i)
			{
				pt2.x = cloud_hull->points[i].x;
				pt2.y = cloud_hull->points[i].y;
				pt2.z = cloud_hull->points[i].z;

				pt3.x = cloud_hull->points[i+1].x;
				pt3.y = cloud_hull->points[i+1].y;
				pt3.z = cloud_hull->points[i+1].z;

				marker.points.push_back(pt1);
				marker.points.push_back(pt2);
				marker.points.push_back(pt3);
			}

			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;

			table_marker_pub_.publish(marker);

		}
		filestr << t.elapsed()<<std::endl;
		filestr.close();
		return;
	}

    ros::NodeHandle n_;


protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher table_marker_pub_;
    ros::Publisher convex_hull_pub_;
    ros::Publisher object_cluster_pub_;

    TransformListener tf_listener_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, DetectTables, DetectTables, nodelet::Nodelet)
