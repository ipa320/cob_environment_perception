#pragma once

#include <sstream>
#include <fstream>


// ROS includes
#include <tf_conversions/tf_kdl.h>
#include <cob_env_model/point_types.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/field_of_view_segmentation.hpp>
#include <pcl/filters/extract_indices.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model_msgs/GetFieldOfView.h>
#include "cob_env_model_msgs/TriggerMappingAction.h"
#include <cob_env_model_msgs/SetReferenceMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <ros/ros.h>
#include <Eigen/Core>

using namespace tf;

#define POINT_MAP_FITNESS_TEST 0

class PointMap {
public:
  typedef pcl::PointXYZRGB Point;

  PointMap(int *pctr):
  first_(true), use_reference_map_(false), fitness_(0.), compution_time_(0.), pctr_(pctr), save_icp_map_(false), use_reuse_(true)
  {
    old_icp_transform_ << 1, 0, 0, 0,
                          0, 1, 0, 0,
                          0, 0, 1, 0,
                          0, 0, 0, 1;
  }

  void transform(const pcl::PointCloud<Point>::Ptr& pc_in, const pcl::PointCloud<Point>::Ptr& pc, const StampedTransform &transform);
  bool compute(const pcl::PointCloud<Point>::Ptr& pc_in, const pcl::PointCloud<Point>::Ptr& pc, const StampedTransform &transform, const cob_env_model_msgs::GetFieldOfView *get_fov_srv);

  void clearMap();
  bool setReferenceMap(cob_env_model_msgs::SetReferenceMap::Request &req);
  void shiftCloud(const pcl::PointCloud<Point>::Ptr& pc);

  pcl::PointCloud<Point> *getRefMap() {return &ref_map_;}
  pcl::PointCloud<Point> *getMap() {return &map_;}

  void setUseReferenceMap(const bool b) {use_reference_map_=b;}
  void setICP_maxIterations(const int v) {icp_max_iterations_=v;}
  void setICP_maxFirstCorrDist(const double v) {icp_max_corr_dist_on_first_=v;}
  void setICP_maxCorrDist(const double v) {icp_max_corr_dist_=v;}
  void setICP_trfEpsilon(const double v) {icp_trf_epsilon_=v;}
  void setSaveICPMap(const bool b) {save_icp_map_=b;}
  void setFilePath(std::string fp) {file_path_=fp;}
  void setReuse(const bool b) {use_reuse_=b;}

  const StampedTransform &getOldTransform() {return transform_old_;}
  bool isFirst() {return first_;}
  bool getUseReferenceMap() {return use_reference_map_;}
  double getFitness() {return fitness_;}
  double getComputionTime() {return compution_time_;}

private:
  pcl::PointCloud<Point> map_;  //FOV ICP map
  pcl::PointCloud<Point> ref_map_;  //reference map

  bool use_reference_map_;
  bool first_;
  bool use_reuse_;

  bool save_icp_map_;
  std::string file_path_;

  double compution_time_;

  // Parameters for ICP
  int icp_max_iterations_;
  int *pctr_;
  double icp_max_corr_dist_,icp_max_corr_dist_on_first_;
  double icp_trf_epsilon_;

  StampedTransform transform_old_;

  ipa_env_model::FieldOfViewSegmentation<Point> seg_;

  Eigen::Vector3d n_up_t_;
  Eigen::Vector3d n_down_t_;
  Eigen::Vector3d n_right_t_;
  Eigen::Vector3d n_left_t_;
  Eigen::Vector3d n_origin_t_;
  Eigen::Vector3d n_max_range_t_;
  Eigen::Matrix4f old_icp_transform_;

  //debug infos
  double fitness_;


  bool doFOVICP(const pcl::PointCloud<Point>::Ptr& pc,
           pcl::PointCloud<Point>& pc_aligned,
           Eigen::Matrix4f& final_transformation, const cob_env_model_msgs::GetFieldOfView *get_fov_srv);
  bool doFOVICPUsingReference(const pcl::PointCloud<Point>::Ptr& pc,
           pcl::PointCloud<Point>& pc_aligned,
           Eigen::Matrix4f& final_transformation);
  void doICP(const pcl::PointCloud<Point>::Ptr& pc);
};
