/*
 * eval_mapping.cpp
 *
 *  Created on: 29.08.2011
 *      Author: goa
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include <fstream>

class EvalMapping
{
public:
  typedef pcl::PointXYZ Point;

  EvalMapping()
  {
    file_list_.push_back(std::string("/kitchen_f"));
    file_list_.push_back(std::string("/kitchen_lf"));
    file_list_.push_back(std::string("/kitchen_ll"));
    file_list_.push_back(std::string("/kitchen_lr"));
    file_list_.push_back(std::string("/kitchen_mf"));
    file_list_.push_back(std::string("/kitchen_mt"));
    file_list_.push_back(std::string("/kitchen_rf"));
    file_list_.push_back(std::string("/kitchen_rl"));
    file_list_.push_back(std::string("/kitchen_rr"));
    file_list_.push_back(std::string("/kitchen_wb"));
    file_list_.push_back(std::string("/kitchen_wr"));
  }

  ~EvalMapping() {}

  void loadGroundTruth(std::string file_path, std::vector<pcl::PointCloud<Point> >& pcs_gt, std::vector<Eigen::Vector4f>& coeffs_gt)
  {
    for(int i=0; i<file_list_.size(); i++)
    {
      pcl::PointCloud<Point> pc;
      Eigen::Vector4f coeffs;
      std::ifstream plane_file;
      std::stringstream ss;
      ss << file_path << file_list_[i] << ".pcd";
      pcl::io::loadPCDFile(ss.str(), pc);
      pcs_gt.push_back(pc);
      ss.str("");
      ss.clear();
      ss << file_path << file_list_[i] << ".pl";
      plane_file.open (ss.str().c_str());
      plane_file >> coeffs(0);
      plane_file >> coeffs(1);
      plane_file >> coeffs(2);
      plane_file >> coeffs(3);
      plane_file.close();
      coeffs_gt.push_back(coeffs);
    }
  }

  void loadMap(std::string file_path, int num_planes, int idx, std::vector<pcl::PointCloud<Point> >& pcs, std::vector<Eigen::Vector4f>& v_coeffs)
  {
    for(int i=0; i<num_planes; i++)
    {
      pcl::PointCloud<Point> pc;
      Eigen::Vector4f coeffs;
      std::ifstream plane_file;
      std::stringstream ss;
      ss << file_path << idx << "_polygon_" << i << "_0.pcd";
      pcl::io::loadPCDFile(ss.str(), pc);
      pcs.push_back(pc);
      ss.str("");
      ss.clear();
      ss << file_path << idx << "_polygon_" << i << ".pl";
      plane_file.open (ss.str().c_str());
      plane_file >> coeffs(0);
      plane_file >> coeffs(1);
      plane_file >> coeffs(2);
      plane_file >> coeffs(3);
      plane_file.close();
      v_coeffs.push_back(coeffs);
    }
  }

  void compareCoeffs(Eigen::Vector4f& coeffs_gt, Eigen::Vector4f& coeffs, double error)
  {

  }

  void calculateCoverage(pcl::PointCloud<Point>& plane_gt, pcl::PointCloud<Point>& hull)
  {

  }

  void associatePlanes(std::vector<pcl::PointCloud<Point> >& pcs_gt, std::vector<pcl::PointCloud<Point> >& pcs, std::vector<Eigen::Vector4f>& coeffs_gt, std::vector<Eigen::Vector4f>& coeffs, std::vector<std::pair<int,int> >& associations)
  {
    double thresh = 0.1;
    for(unsigned int i=0; i<coeffs_gt.size(); i++)
    {
      double assoc_min=10;
      std::pair<int,int> assoc;
      for(unsigned int j=0; j<coeffs.size(); j++)
      {
        if(fabs((coeffs_gt[i] - coeffs[i]).norm()) < thresh && fabs((coeffs_gt[i] - coeffs[i]).norm()) < assoc_min)
        {
          assoc.first = i;
          assoc.second = j;
        }
      }
      associations.push_back(assoc);
    }
  }

  void evaluate(std::vector<pcl::PointCloud<Point> >& pcs_gt, std::vector<pcl::PointCloud<Point> >& pcs, std::vector<Eigen::Vector4f>& coeffs_gt, std::vector<Eigen::Vector4f>& coeffs, std::vector<std::pair<int,int> >& associations)
  {

  }

  std::vector<std::string> file_list_;
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
