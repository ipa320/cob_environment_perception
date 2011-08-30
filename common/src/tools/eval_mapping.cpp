/*
 * eval_mapping.cpp
 *
 *  Created on: 29.08.2011
 *      Author: goa
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

class EvalMapping
{
public:
  EvalMapping() {}
  ~EvalMapping() {}

  void loadGroundTruth(std::string file_path)
  {

  }

  void loadMap(std::string file_path)
  {

  }

  void compareCoeffs(Eigen::Vector4f& coeffs_gt, Eigen::Vector4f& coeffs_gt, double error)
  {

  }

  void calculateCoverage(pcl::PointCloud<Point>& plane_gt, pcl::PointCloud<Point>& hull)
  {

  }

  void associatePlanes(std::vector<Eigen::Vector4f>& coeffs_gt, std::vector<Eigen::Vector4f>& coeffs_gt, std::vector<std::pair<int,int> >& associations)
  {

  }
};

int main(int argc, char** argv)
{
  EvalMapping em;
  std::string file_path_gt = "/home/goa/pcl_daten/kitchen_ground_truth/";
  std::string file_path = "/media/GOADaten/Daten/20110825_sim_kitchen/kitchen_sim_empty_n0/map/";
  //load all files in directory
  //associate
  //evaluate
  return 0;
}
