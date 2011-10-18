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
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

class EvalMapping
{
public:
  typedef pcl::PointXYZ Point;

  EvalMapping()
  {
    file_list_.push_back(std::string("/kitchen_f"));
    file_list_.push_back(std::string("/kitchen_front"));
    //file_list_.push_back(std::string("/kitchen_ll"));
    file_list_.push_back(std::string("/kitchen_lr"));
    file_list_.push_back(std::string("/kitchen_mt"));
    file_list_.push_back(std::string("/kitchen_rl"));
    //file_list_.push_back(std::string("/kitchen_rr"));
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
    std::cout << pcs_gt.size() << " gt planes loaded" << std::endl;
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
      /*if(i==6)
      {
        ss.str("");
        ss.clear();
        ss << file_path << idx << "_polygon_" << i << "_1.pcd";
        pcl::io::loadPCDFile(ss.str(), pc);
        pcs.push_back(pc);
        v_coeffs.push_back(coeffs);
      }*/
    }
    std::cout << pcs.size() << " map planes loaded" << std::endl;
  }

  void compareCoeffs(Eigen::Vector4f& coeffs_gt, Eigen::Vector4f& coeffs, double error)
  {

  }

  void calculateCoverage(pcl::PointCloud<Point>& plane_gt, pcl::PointCloud<Point>& hull)
  {

  }

  void associatePlanes(std::vector<pcl::PointCloud<Point> >& pcs_gt, std::vector<pcl::PointCloud<Point> >& pcs, std::vector<Eigen::Vector4f>& coeffs_gt, std::vector<Eigen::Vector4f>& coeffs, std::vector<std::pair<int,int> >& associations)
  {
    double thresh = 0.5;
    for(unsigned int i=0; i<coeffs.size(); i++)
    {
      std::cout << i << std::endl;
      std::cout << coeffs[i](0) << "," << coeffs[i](1) << "," << coeffs[i](2) << "," << coeffs[i](3) << std::endl;
      double assoc_min=10;
      std::pair<int,int> assoc;
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(pcs[i], centroid);
      for(unsigned int j=0; j<coeffs_gt.size(); j++)
      {
        std::cout << j << ": " << file_list_[j] << std::endl;
        std::cout << coeffs_gt[j](0) << "," << coeffs_gt[j](1) << "," << coeffs_gt[j](2) << "," << coeffs_gt[j](3) << std::endl;
        Eigen::Vector4f centroid_gt;
        pcl::compute3DCentroid(pcs_gt[j], centroid_gt);
        double dist = fabs((centroid-centroid_gt).norm());
        //calculate hull
        /*pcl::PointCloud<Point> cloud_hull;
        pcl::ConvexHull<Point> chull;
        chull.setInputCloud (pcs_gt[j].makeShared());
        chull.reconstruct (cloud_hull);
        int sum =0;
        for(unsigned int k=0; k<pcs[i].size(); k++)
        {
          bool isIn = pcl::isPointIn2DPolygon(pcs[i].points[k], cloud_hull);
          if(!isIn) sum++;
        }*/
        std::cout << "dist: " << dist << std::endl;
        //std::cout << coeffs_gt[i] - coeffs[j] << std::endl;
        //std::cout << (coeffs_gt[i] - coeffs[j]).norm() << std::endl;
        if(fabs((coeffs_gt[j] - coeffs[i]).norm()) < thresh && fabs((coeffs_gt[j] - coeffs[i]).norm()) < assoc_min &&i!=1 && i!=11/*&& dist < 0.8*/)
        {
          assoc.first = i;
          assoc.second = j;
          assoc_min = fabs((coeffs_gt[j] - coeffs[i]).norm());
        }
      }
      std::cout << "assoc min: " << assoc_min << std::endl;
      if(assoc_min==10) continue;
      /*if(i==13)
      {
        assoc.first = i;
        assoc.second = 0;
      }*/
      associations.push_back(assoc);
      std::cout << "map plane " << assoc.first << " associated with gt plane " << assoc.second << std::endl << std::endl;
    }
  }

  void
  evaluate(std::vector<pcl::PointCloud<Point> >& pcs_gt,
           std::vector<pcl::PointCloud<Point> >& pcs,
           std::vector<Eigen::Vector4f>& coeffs_gt,
           std::vector<Eigen::Vector4f>& coeffs,
           std::vector<std::pair<int,int> >& associations)
  {
    std::vector<double> coeff_errors;
    std::vector<double> angle_errors;
    std::vector<double> dist_errors;
    std::vector<double> perc_fps;
    std::vector<double> mean_pt_errors;
    std::vector<double> rms_errors;
    double rms_whole = 0;
    for(unsigned int i=0; i<associations.size(); i++)
    {
      double coeff_error = (coeffs_gt[associations[i].second]-coeffs[associations[i].first]).norm();
      coeff_errors.push_back(coeff_error);
      std::cout << "coeff error " << i << ": " << coeff_error << std::endl;
      Eigen::Vector3f normal_gt = coeffs_gt[associations[i].second].head(3);
      Eigen::Vector3f normal = coeffs[associations[i].first].head(3);
      normal_gt.normalize();
      normal.normalize();
      double angle_error = acos(normal_gt.dot(normal));
      double dist_error = coeffs_gt[associations[i].second](3) - coeffs[associations[i].first](3);
      angle_errors.push_back(angle_error);
      dist_errors.push_back(dist_error);
      std::cout << "angle and dist error: " << angle_error << "," << dist_error << std::endl;

      //false positives (overlap)
      //TODO: rather calculate distance of outliers
      int sum_fp=0;
      for(unsigned int j=0; j<pcs[associations[i].first].size(); j++)
      {
        Point p = pcs[associations[i].first].points[j];

        //calculate hull
        pcl::PointCloud<Point> cloud_hull;
        pcl::ConvexHull<Point> chull;
        chull.setInputCloud (pcs_gt[associations[i].second].makeShared());
        chull.reconstruct (cloud_hull);
        bool isIn = pcl::isPointIn2DPolygon(p, cloud_hull);
        if(!isIn) sum_fp++;
      }
      double perc_fp = sum_fp/pcs[associations[i].first].size();
      perc_fps.push_back(perc_fp);
      std::cout << "perc_fp: " << perc_fp << std::endl;

      //distance errors
      double std_dev_qu = 0;
      double err_sum=0;
      for(unsigned int j=0; j<pcs[associations[i].first].size(); j++)
      {
        Point p = pcs[associations[i].first].points[j];
        double dist = pcl::pointToPlaneDistance(p, coeffs_gt[associations[i].second]);
        std_dev_qu += dist*dist;
        err_sum += dist;
      }
      std_dev_qu /= pcs[associations[i].first].size();
      double rms = sqrt(std_dev_qu);
      rms_whole += rms*rms;
      double err_mean = err_sum/pcs[associations[i].first].size();
      mean_pt_errors.push_back(err_mean);
      rms_errors.push_back(rms);
      std::cout << "err_mean " << i << ": " << err_mean << std::endl;
      std::cout << "rms " << i << ": " << rms << std::endl;

    }
    rms_whole = sqrt(rms_whole/associations.size());
    std::cout << "rms_whole: " << rms_whole << std::endl;

    std::ofstream eval_file;
    std::stringstream ss;
    ss << file_path_ << "/../eval.txt";
    eval_file.open(ss.str().c_str());
    eval_file << "coeff_errors\n";
    for(unsigned int i=0; i<coeff_errors.size(); i++)
      eval_file << coeff_errors[i] << " ";
    eval_file << "\nangle_errors\n";
    for(unsigned int i=0; i<angle_errors.size(); i++)
      eval_file << angle_errors[i] << " ";
    eval_file << "\ndist_errors\n";
    for(unsigned int i=0; i<dist_errors.size(); i++)
      eval_file << dist_errors[i] << " ";
    eval_file << "\nperc_fps\n";
    for(unsigned int i=0; i<perc_fps.size(); i++)
      eval_file << perc_fps[i] << " ";
    eval_file << "\nmean_pt_errors\n";
    for(unsigned int i=0; i<mean_pt_errors.size(); i++)
      eval_file << mean_pt_errors[i] << " ";
    eval_file << "\nrms_errors\n";
    for(unsigned int i=0; i<rms_errors.size(); i++)
      eval_file << rms_errors[i] << " ";
    eval_file << "\nwhole_rms\n";
    eval_file <<rms_whole;
    eval_file.close();
  }

  std::vector<std::string> file_list_;
  std::string file_path_;
};

int main(int argc, char** argv)
{
  typedef pcl::PointXYZ Point;
  EvalMapping em;
  std::string file_path_gt = "/media/GOADaten/Daten/kitchen_ground_truth/";
  std::string file_path = "/media/GOADaten/Daten/20110823_bagfiles/kitchen_empty/map/";
  em.file_path_ = file_path;
  //load all files in directory
  std::vector<pcl::PointCloud<Point> > pcs;
  std::vector<Eigen::Vector4f> coeffs;
  em.loadMap(file_path, 14, 39, pcs, coeffs);
  std::vector<pcl::PointCloud<Point> > pcs_gt;
  std::vector<Eigen::Vector4f> coeffs_gt;
  em.loadGroundTruth(file_path_gt, pcs_gt, coeffs_gt);
  //associate
  std::vector<std::pair<int,int> > associations;
  em.associatePlanes(pcs_gt, pcs, coeffs_gt, coeffs, associations);
  //evaluate
  em.evaluate(pcs_gt, pcs, coeffs_gt, coeffs, associations);
  return 0;
}

// for kitchen_2tables_objects: num_planes=12, idx=20
// for kitchen_empty: num_planes=15, idx=40
// for kitchen_sim_objects2: num_planes=7, idx=84
// for kitchen_sim_empty_n02: num_planes=22, idx=104
// for kitchen_sim_empty_n005: num_planes=7, idx=41
// for kitchen_sim_empty_n0: num_planes=7, idx=108
