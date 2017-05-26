#pragma once

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/esf.h>
#include <pcl/features/cvfh.h>

//#include <pcl/kdtree/impl/kdtree_flann.hpp>
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
  pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
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

  //pcl::PointCloud<pcl::PointNormal>::Ptr keypoints(new pcl::PointCloud<pcl::PointNormal>);
  //pcl::copyPointCloud (*p_n2, *indicies, *keypoints);
  
  descr_est.setIndices(indicies);
  descr_est.setInputCloud (p_n2);
  descr_est.setInputNormals (p_n2);
  descr_est.setSearchSurface (p_n2);
  descr_est.compute (*features);
}

template<>
void _compute<pcl::VFHSignature308>(pcl::PointCloud<pcl::VFHSignature308>::Ptr &features, const double radius, pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2) {
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal> ());
  tree2->setInputCloud(p_n2);

  for(size_t i=0; i<indicies->size(); i++) {
	  std::vector< int > k_indices;
	  std::vector< float > k_sqr_distances;
	  tree2->radiusSearch((*p_n2)[(*indicies)[i]], radius, k_indices, k_sqr_distances);
	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	  for(size_t j=0; j<k_indices.size(); j++) cloud->push_back( (*p_n2)[k_indices[j]] );

	  pcl::CVFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::VFHSignature308> descr_est;
	  descr_est.setRadiusSearch (radius); //not used...
	  descr_est.setKSearch(0);

	  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
	  descr_est.setSearchMethod (tree);
	  //descr_est.setIndices(indicies);
	  
	  descr_est.setInputCloud (cloud);
	  descr_est.setInputNormals (cloud);
	  
	  pcl::PointCloud<pcl::VFHSignature308> f;
	  descr_est.compute (f);
	  if(f.size()>0) features->push_back(f[0]);
  }
}

template<>
void _compute<pcl::ESFSignature640>(pcl::PointCloud<pcl::ESFSignature640>::Ptr &features, const double radius, pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2) {
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal> ());
  tree2->setInputCloud(p_n2);

  for(size_t i=0; i<indicies->size(); i++) {
	  std::vector< int > k_indices;
	  std::vector< float > k_sqr_distances;
	  tree2->radiusSearch((*p_n2)[(*indicies)[i]], radius, k_indices, k_sqr_distances);
	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	  for(size_t j=0; j<k_indices.size(); j++) cloud->push_back( (*p_n2)[k_indices[j]] );

	  pcl::ESFEstimation<pcl::PointNormal, pcl::ESFSignature640> descr_est;
	  descr_est.setRadiusSearch (radius);
	  descr_est.setKSearch(0);
	  
	  descr_est.setInputCloud (cloud);
	  
	  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
	  descr_est.setSearchMethod (tree);
	  descr_est.setSearchSurface (cloud);
	  
	  pcl::PointCloud<pcl::ESFSignature640> f;
	  descr_est.compute (f);
	  if(f.size()>0) features->push_back(f[0]);
  }
}

double computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::vector<Eigen::Vector3d> &keypoints,
	pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2) {
	if(!cloud) {
		std::cout<<"no cloud given !!"<<std::endl;
		return -1;
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
    
   const double r = sw.precisionStop();
    
  std::cout<<"normal estimation "<<r<<"s"<<std::endl;
		
  pcl::PointCloud<pcl::PointNormal> p_n;
  p_n2.reset(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  concatenateFields (*cloud, *normals, p_n);
  {
	  const float leafSize = 0.02f;
	std::vector<int> indicies;
	pcl::removeNaNFromPointCloud(p_n, *p_n2, indicies);
	
	std::cout<<"size "<<p_n2->size()<<" -> ";
	pcl::VoxelGrid<pcl::PointNormal> sor;
	sor.setInputCloud (p_n2);
	sor.setLeafSize (leafSize, leafSize, leafSize);
	sor.filter (*p_n2);
	std::cout<<p_n2->size()<<std::endl;
	
	indicies.clear();
	pcl::removeNaNFromPointCloud(*cloud, *cloud2, indicies);
	
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud (cloud2);
	sor2.setLeafSize (leafSize, leafSize, leafSize);
	sor2.filter (*cloud2);
	
	assert(cloud2->size()==p_n2->size());
	
	for(size_t i=0; i<p_n2->size(); i++) {
		if( (*p_n2)[i].normal_x!=(*p_n2)[i].normal_x ) {
			p_n2->erase(p_n2->begin()+i);
			cloud2->erase(cloud2->begin()+i);
			--i;
		}
	}
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
  tree2->setInputCloud (cloud2);
  indicies.reset(new std::vector<int>);
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
  
  return r;
}

template<class Feature>
double computeFeatures(pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2, const double radius, typename pcl::PointCloud<Feature>::Ptr &features) {
	if(!p_n2) {
		std::cout<<"no cloud given !!"<<std::endl;
		return -1;
	}

  PrecisionStopWatch sw;
  sw.precisionStart();	

  // Output datasets
  features.reset(new pcl::PointCloud<Feature> ());
  _compute<Feature>(features, radius, indicies, p_n2);
  
  const double r = sw.precisionStop();
  
  std::cout<<"feature estimation "<<r<<"s"<<std::endl;
  
  return r;
}


template<class Feature>
void serialize_feature(const Feature &ft, std::vector<float> &res) {
	BOOST_STATIC_ASSERT(sizeof(Feature)==-1);
}

template<>
void serialize_feature<pcl::FPFHSignature33>(const pcl::FPFHSignature33 &ft, std::vector<float> &res) {
	const int SIZE=33;
	res.resize(SIZE);
	for(int i=0; i<SIZE; i++)
		res[i] = ft.histogram[i];
}

template<>
void serialize_feature<pcl::ESFSignature640>(const pcl::ESFSignature640 &ft, std::vector<float> &res) {
	const int SIZE=640;
	res.resize(SIZE);
	for(int i=0; i<SIZE; i++)
		res[i] = ft.histogram[i];
}

template<>
void serialize_feature<pcl::VFHSignature308>(const pcl::VFHSignature308 &ft, std::vector<float> &res) {
	const int SIZE=308;
	res.resize(SIZE);
	for(int i=0; i<SIZE; i++)
		res[i] = ft.histogram[i];
}

template<>
void serialize_feature<pcl::SHOT352>(const pcl::SHOT352 &ft, std::vector<float> &res) {
	const int SIZE=352;
	res.resize(SIZE);
	for(int i=0; i<SIZE; i++)
		res[i] = ft.descriptor[i];
}


////////////////////////////////// KEYPOINTS //////////////////////////////////////
#include <pcl/keypoints/keypoint.h>
//#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>

boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > createKeypointDetector(int keypoint_type) {
  boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector;

  if (keypoint_type == 1)
  {
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
    sift3D->setScales (0.01f, 3, 2);
    sift3D->setMinimumContrast (0.0);
    keypoint_detector.reset (sift3D);
  }
  else
  {
    /*pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
    harris3D->setNonMaxSupression (true);
    harris3D->setRadius (0.01f);
    harris3D->setRadiusSearch (0.01f);
    keypoint_detector.reset (harris3D);*/
    switch (keypoint_type)
    {
      /*case 2:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
      break;

      case 3:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::TOMASI);
      break;

      case 4:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::NOBLE);
      break;

      case 5:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::LOWE);
      break;

      case 6:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::CURVATURE);
      break;*/
      default:
        pcl::console::print_error("unknown key point detection method %d\n expecting values between 1 and 6", keypoint_type);
        exit (1);
        break;
    }

  }
  
  return keypoint_detector;
}

double detectKeypoints(int keypoint_type, typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints)
{
  std::cout << "keypoint detection..." << std::flush;
  
  PrecisionStopWatch sw;
  sw.precisionStart();
  
  boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector = createKeypointDetector(keypoint_type);
  keypoint_detector->setInputCloud(input);
  keypoint_detector->compute(*keypoints);
  
  const double r = sw.precisionStop();
   
  std::cout << "OK. keypoints found: " << keypoints->points.size() << " and took "<< r << std::endl;
  return r;
}
