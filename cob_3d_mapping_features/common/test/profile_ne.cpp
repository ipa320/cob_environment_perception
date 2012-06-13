#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
//#include "cob_3d_mapping_features/fast_edge_estimation_3d_omp.h"
#include "cob_3d_mapping_features/organized_normal_estimation_omp.h"
#include "cob_3d_mapping_features/organized_normal_estimation.h"

using namespace pcl;

void determinePlaneNormal(PointCloud<PointXYZ>::Ptr& p, Eigen::Vector3f& normal)
{
  SACSegmentation<PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (50);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (p);
  ModelCoefficients coefficients_plane;
  pcl::PointIndices inliers_plane;
  seg.segment (inliers_plane, coefficients_plane);
  normal(0) = coefficients_plane.values[0];
  normal(1) = coefficients_plane.values[1];
  normal(2) = coefficients_plane.values[2];
}

int main(int argc, char** argv)
{
  PointCloud<PointXYZ>::Ptr p(new PointCloud<PointXYZ>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<InterestPoint>::Ptr ip(new PointCloud<InterestPoint>);
  pcl::PointCloud<pcl::PointNormal> p_n;
  PrecisionStopWatch t;
  std::string file_ = argv[1];
  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);

  Eigen::Vector3f normal;
  determinePlaneNormal(p, normal);
  std::cout << normal << std::endl;

  cob_3d_mapping_features::OrganizedNormalEstimationOMP<PointXYZ,Normal,PointLabel> ne;
  ne.setPixelSearchRadius(8,2,2);
  ne.setInputCloud(p);
  PointCloud<PointLabel>::Ptr labels(new PointCloud<PointLabel>);
  ne.setOutputLabels(labels);
  t.precisionStart();
  ne.compute(*n);
  std::cout << t.precisionStop() << "s\t for organized normal estimation" << std::endl;
  concatenateFields (*p, *n, p_n);
  io::savePCDFileASCII ("/home/goa/pcl_daten/normal_estimation/normals_organized.pcd", p_n);

  double good_thr=0.97;
  unsigned int ctr=0, nan_ctr=0;
  double d_sum=0;
  for(unsigned int i=0; i<p->size(); i++)
  {
    if ( pcl_isnan(n->points[i].normal[0]) ) {nan_ctr++;continue;}
    double d = normal.dot(n->points[i].getNormalVector3fMap());
    d_sum += fabs(1-fabs(d));
    if(fabs(d)>good_thr) ctr++;
  }
  std::cout << "Average error: " << d_sum/p->size() << std::endl;
  std::cout << "Ratio of good normals: " << (double)ctr/p->size() << std::endl;
  std::cout << "Invalid normals: " << nan_ctr << std::endl;

  IntegralImageNormalEstimation<PointXYZ,Normal> ne2;
  ne2.setNormalEstimationMethod (ne2.COVARIANCE_MATRIX);
  ne2.setMaxDepthChangeFactor(0.02f);
  ne2.setNormalSmoothingSize(10.0f);
  ne2.setDepthDependentSmoothing(true);
  ne2.setInputCloud(p);
  t.precisionStart();
  ne2.compute(*n);
  std::cout << t.precisionStop() << "s\t for integral image normal estimation" << std::endl;
  concatenateFields (*p, *n, p_n);
  io::savePCDFileASCII ("/home/goa/pcl_daten/normal_estimation/normals_integral.pcd", p_n);

  ctr=0;
  nan_ctr=0;
  d_sum=0;
  for(unsigned int i=0; i<p->size(); i++)
  {
    if ( pcl_isnan(n->points[i].normal[0]) ) {nan_ctr++;continue;}
    double d = normal.dot(n->points[i].getNormalVector3fMap());
    d_sum += fabs(1-fabs(d));
    if(fabs(d)>good_thr) ctr++;
  }
  std::cout << "Average error: " << d_sum/p->size() << std::endl;
  std::cout << "Ratio of good normals: " << (double)ctr/p->size() << std::endl;
  std::cout << "Invalid normals: " << nan_ctr << std::endl;

  NormalEstimationOMP<PointXYZ, Normal> ne3;
  ne3.setInputCloud(p);
  ne3.setNumberOfThreads(4);
  ne3.setKSearch(256);
  //ne3.setRadiusSearch(0.01);
  t.precisionStart();
  ne3.compute(*n);
  std::cout << t.precisionStop() << "s\t for vanilla normal estimation" << std::endl;
  concatenateFields (*p, *n, p_n);
  io::savePCDFileASCII ("/home/goa/pcl_daten/normal_estimation/normals_vanilla.pcd", p_n);

  ctr=0;
  nan_ctr=0;
  d_sum=0;
  for(unsigned int i=0; i<p->size(); i++)
  {
    if ( pcl_isnan(n->points[i].normal[0]) ) {nan_ctr++;continue;}
    double d = normal.dot(n->points[i].getNormalVector3fMap());
    d_sum += fabs(1-fabs(d));
    if(fabs(d)>good_thr) ctr++;
  }
  std::cout << "Average error: " << d_sum/p->size() << std::endl;
  std::cout << "Ratio of good normals: " << (double)ctr/p->size() << std::endl;
  std::cout << "Invalid normals: " << nan_ctr << std::endl;

  return 0;
}
