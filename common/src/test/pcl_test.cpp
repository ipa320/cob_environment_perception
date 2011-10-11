
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
 * ROS package name: pcl_test
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: 
 *
 * Date of creation: 09/2010
 * ToDo:
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
#include <sstream>


#include <pcl/ros/register_point_struct.h>
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"

#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/statistical_outlier_removal.h"

#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"

#include "pcl/registration/icp.h"

#include "pcl/surface/mls.h"

#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/filters/passthrough.h"

#include <pcl/features/boundary.h>

#include <sensor_msgs/PointCloud2.h>

#include <boost/timer.hpp>




//####################
//#### node class ####
class PCLTest
{
	public:

		// Constructor
		PCLTest()
		{

		}


		// Destructor
		~PCLTest()
		{
			/// void
		}


		void PlaneSegmentation()
		{
			//pcl::io::savePCDFileASCII ("cloud_unfiltered.pcd", *cloud);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ()), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PCDReader reader;
			reader.read ("/home/goa/pcl_daten/table/icp_fov/map_17.pcd", *cloud);

			std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
			int ctr=0;
			pcl::PassThrough<pcl::PointXYZ> vox_filter;
			vox_filter.setInputCloud(cloud);
			//vox_filter.setLeafSize(0.03, 0.03, 0.03);
			vox_filter.setFilterFieldName ("z");
			vox_filter.setFilterLimits (-0.5, 0.3);
			vox_filter.filter(*cloud);
			/*for (std::vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ> >::iterator i = cloud->points.begin(); i != cloud->points.end(); ++i)
				if(i->x==0 && i->y==0 && i->z==0) {cloud->points.erase(i--);ctr++;}
			cloud->height = cloud->points.size();
			cloud->width = 1;
			std::cerr << ctr << " points removed due to zero coordinates" << std::endl;*/
			pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimator;
			normalEstimator.setInputCloud(cloud);
			pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
			normalEstimator.setSearchMethod(tree);
			normalEstimator.setKSearch(30);
			//normalEstimator.setNumberOfThreads(4);
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal> ());
			normalEstimator.compute(*cloud_normals);

			pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;


			pcl::ModelCoefficients coefficients;
			pcl::PointIndices inliers;

			//doesn't seem to be working
			//Eigen3::Vector3f axis(1,0,0);
			//seg.setAxis(axis);
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
			seg.setNormalDistanceWeight (0.1);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (100);
			seg.setDistanceThreshold (0.03);
			seg.setInputNormals (cloud_normals);
			seg.setInputCloud(cloud);
			seg.segment (inliers, coefficients);

			if (inliers.indices.size () == 0)
			{
			ROS_ERROR ("Could not estimate a planar model for the given dataset.");
			return;
			}

			std::cerr << "Plane coefficients: " << coefficients << std::endl;

			std::cerr << "Model inliers: " << inliers.indices.size () << std::endl;

			//pcl::io::savePCDFileASCII ("cloud_seg.pcd", cloud);
			pcl::PointCloud<pcl::PointXYZ> dominant_plane;
			pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
			extractIndices.setInputCloud(cloud);
			extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
			extractIndices.filter(dominant_plane);
			//extractIndices.setNegative(true);
			//extractIndices.filter(cloud);
			pcl::io::savePCDFileASCII ("/home/goa/pcl_daten/table/icp_fov/dom_plane_17.pcd", dominant_plane);
			/*pcl::io::savePCDFileASCII ("cloud_seg_wo_dom.pcd", cloud);

			int plane_ctr = 2;
			while(inliers.indices.size()!=0 && plane_ctr<10)
			{
				std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
				seg.setInputCloud(cloud.make_shared());
				seg.segment (inliers, coefficients);
				if (inliers.indices.size () == 0)
				{
				ROS_ERROR ("Could not estimate a planar model for the given dataset.");
				return;
				}

				std::cerr << "Model coefficients: " << coefficients.values[0] << " " << coefficients.values[1] << " "
												  << coefficients.values[2] << " " << coefficients.values[3] << std::endl;

				std::cerr << "Model inliers: " << inliers.indices.size () << std::endl;

				extractIndices.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
				extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
				extractIndices.setNegative(false);
				extractIndices.filter(dominant_plane);
				extractIndices.setNegative(true);
				extractIndices.filter(cloud);
				char file_name[30];
				sprintf(file_name,"dom_plane_%d.pcd",plane_ctr);
				pcl::io::savePCDFileASCII (file_name, dominant_plane);
				//pcl::io::savePCDFileASCII ("cloud_seg_wo_dom_2.pcd", cloud);
				plane_ctr++;
			}*/

			return;

		}

		void EstimatePointNormals()
		{
			pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::PCDReader reader;
			reader.read ("/home/goa/pcl_daten/table/icp_fov/pc_aligned_1.pcd", cloud);
			std::cout << "Input cloud has " << cloud.size() << " data points" << std::endl;

			pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> normalEstimator;
			normalEstimator.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
			pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
			normalEstimator.setSearchMethod(tree);
			//normalEstimator.setKSearch(50);
			normalEstimator.setRadiusSearch(0.03);
			normalEstimator.setNumberOfThreads(4);
			pcl::PointCloud<pcl::Normal> cloud_normal;
			boost::timer t;
			normalEstimator.compute(cloud_normal);
			/*for (size_t i = 0; i < cloud.size (); i++)
			{
				cloud_out.points[i].x = cloud.points[i].x;
				cloud_out.points[i].y = cloud.points[i].y;
				cloud_out.points[i].z = cloud.points[i].z;
			}*/
			std::cout << "Normals calculated" << std::endl;
			std::cout << "Normal cloud has " << cloud_normal.size() << " data points" << std::endl;
			ROS_INFO("\tTime: %f", t.elapsed());
			pcl::PointCloud<pcl::PointNormal> cloud_out;
			pcl::concatenateFields (cloud, cloud_normal, cloud_out);
			pcl::io::savePCDFileASCII ("/home/goa/pcl_daten/table/icp_fov/pc_aligned_1.pcd", cloud_out);
			return;


		}

		void PlaneSegmentationNormals()
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PCDReader reader;
			reader.read ("/home/goa/pcl_daten/table/icp_fov/map_17.pcd", *cloud);
			std::cout << "Input cloud has " << cloud->size() << " data points" << std::endl;

			int ctr=0;
			/*for (std::vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ> >::iterator i = cloud->points.begin(); i != cloud->points.end(); ++i)
				if(i->x==0 && i->y==0 && i->z==0) {cloud->points.erase(i--);ctr++;}
			cloud->height = cloud->points.size();
			cloud->width = 1;
			std::cerr << ctr << " points removed due to zero coordinates" << std::endl;*/

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

			ROS_INFO ("Number of table clusters found: %d", (int)table_clusters.size ());

			pcl::ExtractIndices<pcl::PointXYZ> extract;

			for(unsigned int i = 0; i < table_clusters.size(); ++i)
			{
				pcl::PointCloud<pcl::PointXYZ> table_cluster;
				extract.setInputCloud (cloud);
				extract.setIndices (boost::make_shared<const pcl::PointIndices> (table_clusters[i]));
				extract.setNegative (false);
				extract.filter (table_cluster);
				std::stringstream ss1;
				ss1 << "cluster_" << i << ".pcd";
				//pcl::io::savePCDFileASCII (ss1.str(), table_cluster);
				pcl::PointCloud<pcl::PointXYZ>::ConstPtr table_cluster_ptr = boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> > (table_cluster);

				if (table_cluster_ptr->points.size() < (unsigned int)1000)
				{
					ROS_INFO("Table cluster only has %d points, skipping cluster", (int)table_cluster_ptr->points.size());
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
				std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

				if (coefficients_plane->values.size () <=3)
				{
					ROS_INFO("Failed to detect table in scan, skipping cluster");
					continue;
				}
				if ( inliers_plane->indices.size() < (unsigned int)1000)
				{
					ROS_INFO("Plane detection has %d inliers, below min threshold of %d, skipping cluster", (int)inliers_plane->indices.size(), 1000);
					continue;
				}
				if(fabs(coefficients_plane->values[0]) > 0.1 || fabs(coefficients_plane->values[1]) > 0.1 || fabs(coefficients_plane->values[2]) < 0.9)
				{
					ROS_INFO("Detected plane not perpendicular to z axis, skipping cluster");
					continue;
				}


				pcl::PointCloud<pcl::PointXYZ> dominant_plane;
				pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
				extractIndices.setInputCloud(table_cluster_ptr);
				extractIndices.setIndices(inliers_plane);
				extractIndices.filter(dominant_plane);
				//extractIndices.setNegative(true);
				//extractIndices.filter(cloud);
				ROS_INFO("Plane has %d inliers", (int)inliers_plane->indices.size());
				//ROS_INFO("Saved plane to %s", ss.str());

				  // Project the model inliers
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
				pcl::ProjectInliers<pcl::PointXYZ> proj;
				proj.setModelType (pcl::SACMODEL_PLANE);
				proj.setInputCloud (table_cluster_ptr);
				proj.setModelCoefficients (coefficients_plane);
				proj.filter (*cloud_projected);

				std::stringstream ss;
				ss << "plane_" << i << ".pcd";
				//pcl::io::savePCDFileASCII (ss.str(), *cloud_projected);

				// Create a Convex Hull representation of the projected inliers
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ> ());
				pcl::ConvexHull<pcl::PointXYZ> chull;
				chull.setInputCloud (cloud_projected);
				chull.reconstruct (*cloud_hull);

				ROS_INFO ("Convex hull has: %zu data points.", cloud_hull->points.size ());

				pcl::PCDWriter writer;
				//writer.write ("table_hull.pcd", *cloud_hull, false);

			}



			return;


		}

		void VoxelFilter(int idx, double leaf_size_x, double leaf_size_y, double leaf_size_z)
		{
			pcl::PointCloud<pcl::PointXYZRGB> cloud;
			pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
			pcl::PCDReader reader;
			reader.read ("/home/goa/tmp/office_rgb.pcd", cloud);
			std::cout << "Input cloud has " << cloud.size() << " data points" << std::endl;
			pcl::VoxelGrid<pcl::PointXYZRGB> vox_filter;
			vox_filter.setInputCloud(cloud.makeShared());
			vox_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
			boost::timer t;
			vox_filter.filter(cloud_out);
			ROS_INFO("[env_model_node] Pointcloud downsampled.");
			ROS_INFO("\tTime: %f", t.elapsed());
			ROS_INFO ("PointCloud after filtering: %d data points (%s).", cloud_out.width * cloud_out.height, pcl::getFieldsList (cloud_out).c_str ());
                        std::stringstream ss;
                        ss << "/home/goa/tmp/voxel_" << idx << ".pcd";
			pcl::io::savePCDFileASCII (ss.str(), cloud_out);
			return;
		}

		void RegisterICP()
		{
			/*pcl::PointCloud<pcl::PointXYZ> cloud_source; // the pc to register
			pcl::PointCloud<pcl::PointXYZ> cloud_target; // the map
			pcl::PCDReader reader;
			reader.read ("common/files/cob3-2/pcd_kitchen/kitchen_01_world.pcd", cloud_target);
			reader.read ("common/files/cob3-2/pcd_kitchen/kitchen_02_world.pcd", cloud_source);

			pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
			icp.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_source));
			icp.setInputTarget(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_target));
			icp.setMaximumIterations(2);
			icp.setMaxCorrespondenceDistance(0.1);
			pcl::PointCloud<pcl::PointXYZ> cloud_source_aligned;
			boost::timer t;
			//icp.align(cloud_source_aligned);
			ROS_INFO("\tTime: %f", t.elapsed());
			std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
			pcl::io::savePCDFileASCII ("common/files/cob3-2/pcd_kitchen/kitchen_02_aligned_world.pcd", cloud_source_aligned);*/
		}

		void TransformCam2World(pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Matrix4d& trafo)
		{
			for (int i=0; i<cloud.size(); i++)
			{
				Eigen::Vector4d p(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z,1);
				Eigen::Vector4d p_trans = trafo*p;
				cloud.points[i].x = p_trans(0);
				cloud.points[i].y = p_trans(1);
				cloud.points[i].z = p_trans(2);
			}
		}

		void ResamplePointCloud()
		{
			  // Load input file into a PointCloud<T> with an appropriate type
			  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

			pcl::PCDReader reader;
			reader.read ("/home/goa/pcl_daten/shelf/raw_kinect/shelf_0.pcd", *cloud);
			boost::timer t;
			  // Create a KD-Tree
			pcl::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
			  tree->setInputCloud (cloud);

			  // Output has the same type as the input one, it will be only smoothed
			  pcl::PointCloud<pcl::PointXYZ> mls_points;

			  // Init object (second point type is for the normals, even if unused)
			  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

			  // Optionally, a pointer to a cloud can be provided, to be set by MLS
			  pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
			  mls.setOutputNormals (mls_normals);

			  // Set parameters
			  mls.setInputCloud (cloud);
			  mls.setPolynomialFit (false);
			  mls.setSearchMethod (tree);
			  mls.setSearchRadius (0.03);

			  // Reconstruct
			  mls.reconstruct (mls_points);

			  // Concatenate fields for saving
			  pcl::PointCloud<pcl::PointNormal> mls_cloud;
			  pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);
				ROS_INFO("[env_model_node] Pointcloud resampled (MLS).");
				ROS_INFO("\tTime: %f", t.elapsed());

			  pcl::io::savePCDFileASCII ("/home/goa/pcl_daten/shelf/raw_kinect/shelf_0_smoothed_nopol.pcd", mls_cloud);

		}

		void StatisticalOutlierRemoval()
		{
			  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ()), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

			  // Fill in the cloud data
			  pcl::PCDReader reader;
			  reader.read<pcl::PointXYZ> ("common/files/cob3-1/kitchen/kitchen_0.pcd", *cloud);

			  std::cerr << "Cloud before filtering: " << std::endl;
			  std::cerr << *cloud << std::endl;

			  // Create the filtering object
			  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			  sor.setInputCloud (cloud);
			  sor.setMeanK (50);
			  sor.setStddevMulThresh (1.0);
			  sor.filter (*cloud_filtered);

			  std::cerr << "Cloud after filtering: " << std::endl;
			  std::cerr << *cloud_filtered << std::endl;

			  pcl::PCDWriter writer;
			  writer.write<pcl::PointXYZ> ("common/files/cob3-1/kitchen/kitchen_0_filtered.pcd", *cloud_filtered, false);

		}

		void ConvexHull()
		{
			pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::io::loadPCDFile("/home/goa/pcl_daten/table/feature_map/map_28_f0.pcd", cloud);
			  // Project the model inliers
			   // Create a set of planar coefficients with X=Y=0,Z=1
			   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			   coefficients->values.resize (4);
			   coefficients->values[0] = coefficients->values[1] = 0;
			   coefficients->values[2] = 1.0;
			   coefficients->values[3] = -cloud.points[0].z;

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::ProjectInliers<pcl::PointXYZ> proj;
			proj.setModelType (pcl::SACMODEL_PLANE);
			proj.setInputCloud (cloud.makeShared());
			proj.setModelCoefficients (coefficients);
			proj.filter (*cloud_projected);
			pcl::ConvexHull<pcl::PointXYZ> chull;
			pcl::PointCloud<pcl::PointXYZ> hull;
			chull.setInputCloud (cloud_projected);
			chull.reconstruct (hull);
			std::stringstream ss1;
			ss1 << "/home/goa/pcl_daten/table/feature_map/hull_new_28.pcd";
			pcl::io::savePCDFileASCII (ss1.str(), hull);
		}


		void ExtractEdge()
		{
			  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal> ());
			  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointNormal> ());

			  // Fill in the cloud data
			  pcl::PCDReader reader;
			  reader.read<pcl::PointNormal> ("/home/goa/pcl_daten/shelf/raw_kinect/shelf_0_voxel_normals.pcd", *cloud);

				cloud_out->points.resize(cloud->points.size());
				cloud_out->header = cloud->header;

			  std::cerr << "Cloud before filtering: " << std::endl;
			  std::cerr << *cloud << std::endl;

				int nr_p = 0;
				for( unsigned int i = 0; i < cloud->points.size(); i++)
				{
					if( cloud->points[i].curvature > 0.1)
						cloud_out->points[nr_p++] = cloud->points[i];
				}

				cloud_out->width = nr_p;
				cloud_out->height = 1;
				cloud_out->points.resize(nr_p);
				cloud_out->is_dense = true;

			  std::cerr << "Cloud after filtering: " << std::endl;
			  std::cerr << *cloud_out << std::endl;

			  pcl::PCDWriter writer;
			  writer.write<pcl::PointNormal> ("/home/goa/pcl_daten/shelf/raw_kinect/shelf_0_edge.pcd", *cloud_out, false);

		}

		void BoundaryEstimation()
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal> ());
			pcl::PointCloud<pcl::Boundary>::Ptr cloud_out (new pcl::PointCloud<pcl::Boundary> ());
			pcl::PCDReader reader;
			reader.read<pcl::PointNormal> ("/home/goa/pcl_daten/shelf/raw_kinect/shelf_0_voxel_normals.pcd", *cloud);

			boost::timer t;
			pcl::KdTree<pcl::PointNormal>::Ptr tree;
			tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointNormal> > ();
			pcl::BoundaryEstimation<pcl::PointNormal,pcl::PointNormal,pcl::Boundary> boundary;
			boundary.setSearchMethod(tree);
			boundary.setInputCloud(cloud);
			//boundary.setSearchSurface (cloud);
			boundary.setRadiusSearch(0.05);
			boundary.setInputNormals(cloud);
			//boundary.angle_threshold_ = M_PI/4;
			boundary.compute(*cloud_out);
			pcl::PointCloud<pcl::PointNormal> boundary_cloud;
			boundary_cloud.points.resize(cloud->points.size());
			boundary_cloud.header = cloud->header;
			int nr_p = 0;
			pcl::PointNormal min_pt, max_pt;
			pcl::getMinMax3D (*cloud, min_pt, max_pt);
			for( unsigned int i = 0; i < cloud_out->points.size(); i++)
			{
				if( cloud_out->points[i].boundary_point == 1)
				{
					//if(cloud->points[i].y>min_pt.y+0.05)
						boundary_cloud.points[nr_p++] = cloud->points[i];
				}
			}
			std::cout << "nr_p: " << nr_p << ", width: " << cloud->width << std::endl;
			boundary_cloud.width = nr_p;
			boundary_cloud.height = 1;
			boundary_cloud.points.resize(nr_p);
			boundary_cloud.is_dense = true;
			std::cout << "Time elapsed for boundary estimation: " << t.elapsed() << std::endl;
			pcl::PCDWriter writer;
			writer.write<pcl::PointNormal> ("/home/goa/pcl_daten/shelf/raw_kinect/shelf_0_voxel_boundary_2.pcd", boundary_cloud, false);
		}


		void ExtractBoundary()
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::Boundary>::Ptr cloud_bound (new pcl::PointCloud<pcl::Boundary> ());
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZ> ("/home/goa/pcl_daten/table/boundary/boundary_1.pcd", *cloud);
			reader.read<pcl::Boundary> ("/home/goa/pcl_daten/table/boundary/boundary_1.pcd", *cloud_bound);

			pcl::PCDWriter writer;
			writer.write<pcl::Boundary> ("/home/goa/pcl_daten/table/boundary/boundary_pts_1.pcd", *cloud_bound, false);
			writer.write<pcl::PointXYZ> ("/home/goa/pcl_daten/table/boundary/boundary_XYZ_1.pcd", *cloud, false);

			std::cerr << *cloud << std::endl;
			std::cerr << *cloud_bound << std::endl;

			boost::timer t;

			cloud_out->points.resize(cloud->points.size());
			cloud_out->header = cloud->header;
			int nr_p = 0;
			for( unsigned int i = 0; i < cloud->points.size(); i++)
			{
				if( cloud_bound->points[i].boundary_point == 1)
					cloud_out->points[nr_p++] = cloud->points[i];
			}
			cloud_out->width = nr_p;
			cloud_out->height = 1;
			cloud_out->points.resize(nr_p);
			cloud_out->is_dense = true;
			std::cout << "Time elapsed for boundary estimation: " << t.elapsed() << std::endl;

			writer.write<pcl::PointXYZ> ("/home/goa/pcl_daten/table/boundary/boundary_cut_1.pcd", *cloud_out, false);
		}

		void ConcatNormals()
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointNormal> ());
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal> ());
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZ> ("/home/goa/pcl_daten/table/icp_fov/pc_aligned_1.pcd", *cloud);
			reader.read<pcl::Normal> ("/home/goa/pcl_daten/table/normals/normals_1.pcd", *cloud_normal);

			pcl::concatenateFields(*cloud, *cloud_normal, *cloud_out);
			pcl::PCDWriter writer;
			writer.write<pcl::PointNormal> ("/home/goa/pcl_daten/table/normals/cloud_normals_1.pcd", *cloud_out, false);
		}

};



//#######################
//#### main programm ####




int main(int argc, char** argv)
{


	PCLTest pclTest;

	//pclTest.PlaneSegmentation();
	//pclTest.PlaneSegmentationNormals();
    pclTest.VoxelFilter(0, 0.01, 0.01, 0.01);
    pclTest.VoxelFilter(1, 0.03, 0.03, 0.03);
    pclTest.VoxelFilter(2, 0.05, 0.05, 0.05);

    pclTest.VoxelFilter(3, 0.03, 0.01, 0.01);
    pclTest.VoxelFilter(4, 0.05, 0.01, 0.01);
    //pclTest.EstimatePointNormals();
	//pclTest.RegisterICP();
	//pclTest.ResamplePointCloud();
	//pclTest.StatisticalOutlierRemoval();
	//pclTest.ConvexHull();
    //pclTest.ExtractEdge();
	//pclTest.BoundaryEstimation();
	//pclTest.ExtractBoundary();
   // pclTest.ConcatNormals();

    std::cout << "done" << std::endl;


    return 0;
}

