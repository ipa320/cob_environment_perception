#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_features/organized_normal_estimation.h"
#include "cob_3d_mapping_features/organized_curvature_estimation.h"
#include "cob_3d_mapping_features/organized_normal_estimation_omp.h"
#include "cob_3d_mapping_features/organized_curvature_estimation_omp.h"

using namespace pcl;

int main(int argc, char** argv)
{
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<PointLabel>::Ptr l(new PointCloud<PointLabel>);
  PointCloud<PrincipalCurvatures>::Ptr pc(new PointCloud<PrincipalCurvatures>);

  PrecisionStopWatch t;
  std::string file_ = argv[1];
  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);

  t.precisionStart();
  cob_3d_mapping_features::OrganizedNormalEstimationOMP<PointXYZRGB, Normal, PointLabel>one;
  one.setInputCloud(p);
  one.setOutputLabels(l);
  one.setPixelSearchRadius(8,1,2); //radius,pixel,circle
  one.setSkipDistantPointThreshold(12);
  one.compute(*n);
  std::cout << t.precisionStop() << "s\t for organized curvature estimation" << std::endl;

  t.precisionStart();
  cob_3d_mapping_features::OrganizedCurvatureEstimationOMP<PointXYZRGB,Normal,PointLabel,PrincipalCurvatures>oce;
  oce.setInputCloud(p);
  oce.setInputNormals(n);
  oce.setOutputLabels(l);
  oce.setPixelSearchRadius(8,1,2);
  oce.setSkipDistantPointThreshold(12);
  oce.setEdgeClassThreshold(6);
  oce.compute(*pc);
  std::cout << t.precisionStop() << "s\t for organized curvature estimation" << std::endl;


  return 0;
}
