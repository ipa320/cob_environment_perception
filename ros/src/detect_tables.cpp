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
#include <pcl/io/pcd_io.h>
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
#include "pcl/surface/concave_hull.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include "pcl/filters/voxel_grid.h"


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### nodelet class ####
class DetectTables : public pcl_ros::PCLNodelet
{
public:
	typedef pcl::PointXYZRGB Point;
    // Constructor
	DetectTables()
	{
		ctr_=0;
		min_cluster_size_=300;
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
		chull_pub_ = n_.advertise<pcl::PointCloud<Point> >("chull",1);
		object_cluster_pub_ = n_.advertise<pcl::PointCloud<Point> >("object_cluster",1);
		polygon_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("polygons",1);
		polygon_array_pub_ = n_.advertise<cob_env_model::PolygonArray>("polygon_array",1);
    }


    //input should be point cloud that is amplitude filetered, statistical outlier filtered, voxel filtered and the floor cut, coordinate system should be /map
	void pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
	{
		std::cout << "Detect table callback" << std::endl;
		std::fstream filestr;
		filestr.open("/home/goa/pcl_daten/table/table_detection/meas.csv", std::fstream::in | std::fstream::out | std::fstream::app);
		boost::timer t;
		pcl::VoxelGrid<Point> voxel;
    	voxel.setInputCloud(pc_in);
    	voxel.setLeafSize(0.02,0.02,0.02);
    	voxel.setFilterFieldName("z");
    	voxel.setFilterLimits(0.2,3);
    	pcl::PointCloud<Point>::Ptr cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    	voxel.filter(*cloud);
		//static int ctr = 0;
		pcl::KdTree<Point>::Ptr clusters_tree;
		clusters_tree = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
		pcl::EuclideanClusterExtraction<Point> cluster;

		// Table clustering parameters
		cluster.setClusterTolerance (0.06);
		cluster.setMinClusterSize (min_cluster_size_);
		cluster.setSearchMethod (clusters_tree);

		// Cluster potential table points
		std::vector<pcl::PointIndices> table_clusters;
		cluster.setInputCloud (cloud);
		cluster.extract (table_clusters);

		ROS_INFO ("Number of table clusters found: %d", (int)table_clusters.size ());

		pcl::ExtractIndices<Point> extract;

		for(unsigned int i = 0; i < table_clusters.size(); ++i)
		{
			// extract cluster points
			pcl::PointCloud<Point> table_cluster;
			extract.setInputCloud (cloud);
			extract.setIndices (boost::make_shared<const pcl::PointIndices> (table_clusters[i]));
			extract.setNegative (false);
			extract.filter (table_cluster);

			pcl::PointCloud<Point>::Ptr table_cluster_ptr = table_cluster.makeShared();
			pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
			while(table_cluster_ptr->size()>min_cluster_size_)
			{
				pcl::NormalEstimation<Point,pcl::Normal> normalEstimator;
				normalEstimator.setInputCloud(table_cluster_ptr);
				pcl::KdTreeFLANN<Point>::Ptr tree (new pcl::KdTreeFLANN<Point> ());
				normalEstimator.setSearchMethod(tree);
				normalEstimator.setKSearch(30);
				//normalEstimator.setNumberOfThreads(4);
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal> ());
				normalEstimator.compute(*cloud_normals);

				pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;

				// Create the segmentation object for the planar model and set all the parameters
				seg.setOptimizeCoefficients (true);
				seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
				//seg.setAxis(Eigen::Vector3f(0,0,1));
				seg.setNormalDistanceWeight (0.05);
				seg.setMethodType (pcl::SAC_RANSAC);
				seg.setMaxIterations (100);
				seg.setDistanceThreshold (0.03);
				seg.setInputCloud (table_cluster_ptr);
				seg.setInputNormals (cloud_normals);
				// Obtain the plane inliers and coefficients
				pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ());
				seg.segment (*inliers_plane, *coefficients_plane);

				float r=0, g=0, b=0;
				if (coefficients_plane->values.size () <=3)
				{
					//ROS_INFO("Failed to detect table in scan, skipping cluster");
					break;
				}
				if ( inliers_plane->indices.size() < (unsigned int)150)
				{
					//ROS_INFO("Plane detection has %d inliers, below min threshold of %d, skipping cluster", (int)inliers_plane->indices.size(), 150);
					break;
				}
				if(fabs(coefficients_plane->values[0]) < 0.1 && fabs(coefficients_plane->values[1]) < 0.1 && fabs(coefficients_plane->values[2]) > 0.9)
				{
					//ROS_INFO("Detected plane perpendicular to z axis");
					std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;
					g=1;

				}
				else if(/*fabs(coefficients_plane->values[0]) < 0.9 && fabs(coefficients_plane->values[1]) < 0.9 &&*/ fabs(coefficients_plane->values[2]) < 0.15)
				{
					//ROS_INFO("Detected plane parallel to z axis");
					b=1;
				}
				else
				{
					ROS_INFO("Detected plane not parallel to z axis or perpendicular to z axis");
					std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;
					break;
				}


                                ROS_INFO("Plane has %d inliers", (int)inliers_plane->indices.size());
				pcl::PointCloud<Point> dominant_plane;
				pcl::ExtractIndices<Point> extractIndices;
				extractIndices.setInputCloud(table_cluster_ptr);
				extractIndices.setIndices(inliers_plane);
				extractIndices.filter(dominant_plane);
				//ROS_INFO("Saved plane to %s", ss.str());



				  // Project the model inliers
				pcl::PointCloud<Point>::Ptr cloud_projected (new pcl::PointCloud<Point> ());
				pcl::ProjectInliers<Point> proj;
				proj.setModelType (pcl::SACMODEL_PLANE);
				proj.setInputCloud (dominant_plane.makeShared()/*table_cluster_ptr*/);
				//proj.setIndices(inliers_plane);
				proj.setModelCoefficients (coefficients_plane);
				proj.filter (*cloud_projected);

				std::stringstream ss1;
				ss1 << "/home/goa/pcl_daten/kitchen_kinect/planes/cluster_" << ctr_ << ".pcd";
			//	pcl::io::savePCDFileASCII (ss1.str(), table_cluster);

				std::stringstream ss;
				ss << "/home/goa/pcl_daten/kitchen_kinect/planes/plane_" << ctr_ << ".pcd";
			//	pcl::io::savePCDFileASCII (ss.str(), dominant_plane);


				// Create a Convex Hull representation of the projected inliers
				pcl::PointCloud<Point>::Ptr cloud_hull (new pcl::PointCloud<Point> ());
				std::vector< pcl::Vertices > hull_polygons;
				pcl::ConcaveHull<Point> chull;
				chull.setInputCloud (cloud_projected);
				chull.setAlpha (0.2);
				chull.reconstruct (*cloud_hull, hull_polygons);
				cob_env_model::PolygonArray p;
				p.polygons.resize(hull_polygons.size());
				p.header = pc_in->header;
				p.normal.x = coefficients_plane->values[0];
				p.normal.y = coefficients_plane->values[1];
				p.normal.z = coefficients_plane->values[2];
				p.d.data = coefficients_plane->values[3];
				for(int j=0; j<hull_polygons.size(); j++)
				{
					p.polygons[j].points.resize(hull_polygons[j].vertices.size());
					for(int k=0; k<hull_polygons[j].vertices.size(); k++)
					{
						int idx = hull_polygons[j].vertices[k];
						p.polygons[j].points[k].x = cloud_hull->points[idx].x;
						p.polygons[j].points[k].y = cloud_hull->points[idx].y;
						p.polygons[j].points[k].z = cloud_hull->points[idx].z;
					}
					polygon_array_pub_.publish(p);
				}
				chull_pub_.publish(cloud_hull);
				std::stringstream ss2;
				ss2 << "/home/goa/pcl_daten/kitchen_kinect2/planes/hull_" << ctr_ << ".pcd";
				pcl::io::savePCDFileASCII (ss2.str(), *cloud_hull);
				std::stringstream ss3;
				ss3 << "/home/goa/pcl_daten/kitchen_kinect2/planes/plane_pr_" << ctr_ << ".pcd";
			//	pcl::io::savePCDFileASCII (ss3.str(), *cloud_projected);

				ctr_++;

				//publishMarker(*cloud_hull, cloud->header.frame_id);
				publishPolygons(*cloud_hull, cloud->header.frame_id, cloud->header.stamp);
				publishMarker2(*cloud_hull, cloud->header.frame_id, cloud->header.stamp, r, g, b);

				/*pcl::ExtractPolygonalPrismData<Point> prism;
				// Consider only objects in a given layer above the table
				prism.setHeightLimits (-0.5, -0.03);
				// ---[ Get the objects on top of the table
				pcl::PointIndices cloud_object_indices;
				prism.setInputCloud (table_cluster_ptr);
				prism.setInputPlanarHull (cloud_hull);
				prism.segment (cloud_object_indices);

				pcl::PointCloud<Point> cloud_objects;
				pcl::ExtractIndices<Point> extract_object_indices;
				extract_object_indices.setInputCloud (table_cluster_ptr);
				extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
				extract_object_indices.filter (cloud_objects);
				pcl::PointCloud<Point>::ConstPtr cloud_objects_ptr = cloud_objects.makeShared();

				pcl::KdTree<Point>::Ptr obj_clusters_tree;
				obj_clusters_tree = boost::make_shared<pcl::KdTreeFLANN<Point> > ();

				pcl::EuclideanClusterExtraction<Point> cluster_obj;

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
					pcl::PointCloud<Point> object;
					extract.setInputCloud (cloud_objects.makeShared());
					extract.setIndices (boost::make_shared<const pcl::PointIndices> (object_clusters[i]));
					extract.setNegative (false);
					extract.filter (object);
					object_cluster_pub_.publish(object);
				}*/


				/*std::stringstream ss;
				ss << "/home/goa/pcl_daten/table_detection/hull_" << ctr << ".pcd";
				pcl::io::savePCDFileASCII (ss.str(), *cloud_hull);
				ctr++;*/

				//ROS_INFO ("Convex hull has: %zu data points.", cloud_hull->points.size ());

				extractIndices.setNegative(true);
				extractIndices.filter(*table_cluster_ptr);
			}
		}


		filestr << t.elapsed()<<std::endl;
		filestr.close();
		return;
	}

	void publishPolygons(pcl::PointCloud<Point>& cloud_hull, std::string& frame_id, ros::Time stamp)
	{
		geometry_msgs::PolygonStamped polygon;
		polygon.header.frame_id=frame_id;
		polygon.header.stamp = stamp;
		polygon.polygon.points.resize(cloud_hull.points.size());
		for(unsigned int i = 0; i < cloud_hull.points.size(); i++)
		{
			polygon.polygon.points[i].x = cloud_hull.points[i].x;
			polygon.polygon.points[i].y = cloud_hull.points[i].y;
			polygon.polygon.points[i].z = cloud_hull.points[i].z;
		}
		polygon_pub_.publish(polygon);
	}

	void publishMarker(pcl::PointCloud<Point>& cloud_hull, std::string& frame_id)
	{
		visualization_msgs::Marker marker;
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
		marker.lifetime = ros::Duration();
		marker.header.frame_id = frame_id;
		marker.id = ctr_;


		//create the marker in the table reference frame
		//the caller is responsible for setting the pose of the marker to match

		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 1;

		geometry_msgs::Point pt1, pt2, pt3;
		pt1.x = cloud_hull.points[0].x;
		pt1.y = cloud_hull.points[0].y;
		pt1.z = cloud_hull.points[0].z;

		for(unsigned int i = 1; i < cloud_hull.points.size()-1; ++i)
		{
			pt2.x = cloud_hull.points[i].x;
			pt2.y = cloud_hull.points[i].y;
			pt2.z = cloud_hull.points[i].z;

			pt3.x = cloud_hull.points[i+1].x;
			pt3.y = cloud_hull.points[i+1].y;
			pt3.z = cloud_hull.points[i+1].z;

			marker.points.push_back(pt1);
			marker.points.push_back(pt2);
			marker.points.push_back(pt3);
		}

		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 1;
		marker.color.a = 1.0;

		table_marker_pub_.publish(marker);
	}

	void publishMarker2(pcl::PointCloud<Point>& cloud_hull, std::string& frame_id, ros::Time stamp, float r, float g, float b)
	{
		visualization_msgs::Marker marker;
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.lifetime = ros::Duration();
		marker.header.frame_id = frame_id;
		marker.header.stamp = stamp;
		marker.id = ctr_;


		//create the marker in the table reference frame
		//the caller is responsible for setting the pose of the marker to match

		marker.scale.x = 0.02;
		marker.scale.y = 0.02;
		marker.scale.z = 1;

		geometry_msgs::Point pt;

		for(unsigned int i = 0; i < cloud_hull.points.size(); ++i)
		{
			pt.x = cloud_hull.points[i].x;
			pt.y = cloud_hull.points[i].y;
			pt.z = cloud_hull.points[i].z;

			marker.points.push_back(pt);
		}
		pt.x = cloud_hull.points[0].x;
		pt.y = cloud_hull.points[0].y;
		pt.z = cloud_hull.points[0].z;

		//marker.points.push_back(pt);

		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;

		table_marker_pub_.publish(marker);
	}

    ros::NodeHandle n_;


protected:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher table_marker_pub_;
    ros::Publisher chull_pub_;
    ros::Publisher object_cluster_pub_;
    ros::Publisher polygon_array_pub_;
    ros::Publisher polygon_pub_;

    TransformListener tf_listener_;
    int ctr_;
    int min_cluster_size_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, DetectTables, DetectTables, nodelet::Nodelet)
