#pragma once

#include <pcl/features/fpfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/esf.h>
#include <pcl/features/cvfh.h>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include "cob_3d_features/organized_normal_estimation_omp.h"
#include "cob_3d_features/organized_normal_estimation.h"
#include "cob_3d_mapping_common/point_types.h"

void removeNaNs(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
	std::vector<int> indicies;
	cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::removeNaNFromPointCloud(cloud, *cloud_out, indicies);
}

template<class Feature>
void _compute(typename pcl::PointCloud<Feature>::Ptr &features, const double radius, typename pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2) {
	BOOST_STATIC_ASSERT(sizeof(Feature)==-1);
}

template<>
void _compute<pcl::FPFHSignature33>(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &features, const double radius, pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2) {
  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (p_n2);
  fpfh.setInputNormals (p_n2);
  
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
  fpfh.setSearchMethod (tree);
  fpfh.setIndices(indicies);
  
  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (radius);

  // Compute the features
  fpfh.compute (*features);
}

template<>
void _compute<pcl::SHOT352>(pcl::PointCloud<pcl::SHOT352>::Ptr &features, const double radius, pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2) {
  pcl::SHOTEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::SHOT352> descr_est;
  descr_est.setRadiusSearch (radius);

  pcl::PointCloud<pcl::PointNormal>::Ptr keypoints(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud (*p_n2, *indicies, *keypoints);
  
  descr_est.setInputCloud (keypoints);
  descr_est.setInputNormals (p_n2);
  descr_est.setSearchSurface (p_n2);
  descr_est.compute (*features);
}

template<>
void _compute<pcl::VFHSignature308>(pcl::PointCloud<pcl::VFHSignature308>::Ptr &features, const double radius, pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2) {
  pcl::CVFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::VFHSignature308> descr_est;
  descr_est.setRadiusSearch (radius); //not used...

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
  descr_est.setSearchMethod (tree);
  descr_est.setIndices(indicies);
  
  descr_est.setInputCloud (p_n2);
  descr_est.setInputNormals (p_n2);
  
  descr_est.compute (*features);
}

template<>
void _compute<pcl::ESFSignature640>(pcl::PointCloud<pcl::ESFSignature640>::Ptr &features, const double radius, pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2) {
  pcl::ESFEstimation<pcl::PointNormal, pcl::ESFSignature640> descr_est;
  descr_est.setRadiusSearch (radius);
  
  descr_est.setInputCloud (p_n2);
  
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
  descr_est.setSearchMethod (tree);
  descr_est.setIndices(indicies);
  
  descr_est.compute (*features);
}

template<class Feature>
void computeFeatures(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const double radius, typename pcl::PointCloud<Feature>::Ptr &features, const std::vector<Eigen::Vector3d> &keypoints) {
	if(!cloud) {
		std::cout<<"no cloud given !!"<<std::endl;
		return;
	}
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  
  PrecisionStopWatch sw;
  sw.precisionStart();
  
  pcl::PointCloud<PointLabel>::Ptr l (new pcl::PointCloud<PointLabel> ());
  cob_3d_features::OrganizedNormalEstimationOMP<pcl::PointXYZ, pcl::Normal, PointLabel> one;
    one.setInputCloud(cloud);
    one.setOutputLabels(l);
    one.setPixelSearchRadius(8,2,2);
    //one.setSkipDistantPointThreshold(12);
    one.compute(*normals);
    
  std::cout<<"normal estimation "<<sw.precisionStop()<<"s"<<std::endl;
		
  pcl::PointCloud<pcl::PointNormal> p_n;
  pcl::PointCloud<pcl::PointNormal>::Ptr p_n2(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  concatenateFields (*cloud, *normals, p_n);
  {
	std::vector<int> indicies;
	pcl::removeNaNFromPointCloud(p_n, *p_n2, indicies);
	
	pcl::VoxelGrid<pcl::PointNormal> sor;
	sor.setInputCloud (p_n2);
	sor.setLeafSize (0.03f, 0.03f, 0.03f);
	sor.filter (*p_n2);
	
	indicies.clear();
	pcl::removeNaNFromPointCloud(*cloud, *cloud2, indicies);
	
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud (cloud2);
	sor2.setLeafSize (0.03f, 0.03f, 0.03f);
	sor2.filter (*cloud2);
	
	assert(cloud2->size()==p_n2->size());
  }
  
  /*for(size_t i=0; i<cloud->size(); i++)
	if((*cloud)[i].x!=(*cloud)[i].x) {
		(*cloud)[i].x=(*cloud)[i].y=(*cloud)[i].z=-1000;
	}*/

  sw.precisionStart();

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
  tree2->setInputCloud (cloud2);
  pcl::IndicesPtr indicies(new std::vector<int>);
  for(size_t i=0; i<keypoints.size(); i++) {
	  pcl::PointXYZ pt;
	  pt.x = keypoints[i](0);
	  pt.y = keypoints[i](1);
	  pt.z = keypoints[i](2);
	  std::vector< int > k_indices;
	  std::vector< float > k_sqr_distances;
	  tree2->nearestKSearch(pt, 1, k_indices, k_sqr_distances);
	  assert(k_indices.size()>0);
	  
	  indicies->push_back(k_indices[0]);
  }
  std::cout<<"indicies "<<indicies->size()<<std::endl;
	

  // Output datasets
  features.reset(new pcl::PointCloud<Feature> ());

  _compute<Feature>(features, radius, indicies, p_n2);
  
  std::cout<<"feature estimation "<<sw.precisionStop()<<"s"<<std::endl;
}
