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
  typedef pcl::PointXYZ Point;

  PointMap(int *pctr):
    use_reference_map_(false), first_(true), fitness_(0.), compution_time_(0.), pctr_(pctr), save_icp_map_(false), use_reuse_(true)
  {
    old_icp_transform_ = Eigen::Matrix4f::Identity();
  }


  /**
   * @brief transforms two cloud points with one transformation
   *
   * transforms two cloud points with one transformation
   *
   * @param pc_in point cloud one
   * @param pc point cloud tow
   * @param tranform transformation
   *
   * @return nothing
   */
  void transform(const pcl::PointCloud<Point>::Ptr& pc_in, const pcl::PointCloud<Point>::Ptr& pc, const StampedTransform &transform);

  /**
   * @brief computes the new 3d map using a new point cloud as input
   *
   * computes the new 3d map using a new point cloud as input which will be aligned to the existing reference map
   * if it isn't possible the input data will be ignored; algorithm: ICP
   * pc is the downsampled version of pc_in, while get_fov_srv is optional
   *
   * @param pc_in point cloud one
   * @param pc point cloud two (downsampled from point cloud one)
   * @param tranform transformation (optional, can be NULL if not needed)
   *
   * @return nothing
   */
  bool compute(const pcl::PointCloud<Point>::Ptr& pc_in, const pcl::PointCloud<Point>::Ptr& pc, const StampedTransform &transform, const cob_env_model_msgs::GetFieldOfView *get_fov_srv);


  /**
   * @brief clears map
   *
   * deletes 3d map of the environment
   *
   * @return nothing
   */
  void clearMap();

  /**
   * @brief sets reference map
   *
   * sets the 3d map representing the environment which is used to align new frames
   *
   * @param req reference map
   *
   * @return nothing
   */
  bool setReferenceMap(cob_env_model_msgs::SetReferenceMap::Request &req);

  /**
   * @brief test function to shift the cloud to test if better results are possible
   *
   * test function to shift the cloud to test if better results are possible
   *
   * @deprecated just for testing
   *
   * @param pc point cloud to shift
   *
   * @return nothing
   */
  void shiftCloud(const pcl::PointCloud<Point>::Ptr& pc);

  /**
   * @brief gets reference map
   *
   * gets reference map
   *
   * @return reference map
   */
  pcl::PointCloud<Point> *getRefMap() {return &ref_map_;}

  /**
   * @brief gets map
   *
   * gets map
   *
   * @return map
   */
  pcl::PointCloud<Point> *getMap() {return &map_;}

  /**
   * @brief sets use of reference map
   *
   * sets use of reference map
   *
   * @param b enables/disables reference map
   *
   * @return noting
   */
  void setUseReferenceMap(const bool b) {use_reference_map_=b;}

  /**
   * @brief sets maximum number of iterations
   *
   * sets maximum number of iterations
   *
   * @param v number of iterations
   *
   * @return noting
   */
  void setICP_maxIterations(const int v) {icp_max_iterations_=v;}

  /**
   * @brief sets maximum correspondence distance in first computation
   *
   * sets maximum correspondence distance in first computation
   *
   * @param v correspondence distance
   *
   * @return noting
   */
  void setICP_maxFirstCorrDist(const double v) {icp_max_corr_dist_on_first_=v;}

  /**
   * @brief sets maximum correspondence distance
   *
   * sets maximum correspondence distance
   *
   * @param v correspondence distance
   *
   * @return noting
   */
  void setICP_maxCorrDist(const double v) {icp_max_corr_dist_=v;}

  /**
   * @brief sets transformation epsilon
   *
   * sets transformation epsilon
   *
   * @param v epsilon, break condition if two consecutive transformation differ less thand epsilon
   *
   * @return noting
   */
  void setICP_trfEpsilon(const double v) {icp_trf_epsilon_=v;}

  /**
   * @brief save map as pcd?
   *
   * save map as pcd?
   *
   * @param b enable/disable
   *
   * @return noting
   */
  void setSaveICPMap(const bool b) {save_icp_map_=b;}

  /**
   * @brief sets filepath to store debug files
   *
   * sets filepath to store debug files
   *
   * @param fp filepath
   *
   * @return noting
   */
  void setFilePath(std::string fp) {file_path_=fp;}

  /**
   * @brief enables the reuse of old transformation for icp
   *
   * enables the reuse of old transformation for icp which will be used as initial setting
   * for approximation of new transformation parameters
   *
   * @param b enable/disable
   *
   * @return noting
   */
  void setReuse(const bool b) {use_reuse_=b;}

  /**
   * @brief gets the last transformation matrix
   *
   * gets the last transformation matrix
   *
   * @return last transformation
   */
  const StampedTransform &getOldTransform() {return transform_old_;}

  /**
   * @brief checks wether it is the first computation
   *
   * checks wether it is the first computation
   * will return false after a computation returned successfully
   *
   * @return false after a computation returned successfully
   */
  bool isFirst() {return first_;}

  /**
   * @brief gets wether the reference map is used
   *
   * gets wether the reference map is used
   *
   * @return wether the reference map is used
   */
  bool getUseReferenceMap() {return use_reference_map_;}

  /**
   * @brief gets fitness of icp, will be usually not be computed
   *
   * gets fitness of icp, will be usually not be computed
   *
   * @return fitness
   */
  double getFitness() {return fitness_;}

  /**
   * @brief gets time needed for an computation
   *
   * gets time needed for an computation
   *
   * @return time needed for an computation
   */
  double getComputionTime() {return compution_time_;}

private:
  pcl::PointCloud<Point> map_;  /// FOV ICP map
  pcl::PointCloud<Point> ref_map_;  /// reference map

  bool use_reference_map_; /// wether use reference map
  bool first_; /// false if icp succeeded
  bool use_reuse_; /// use old transformation as hint for icp

  bool save_icp_map_; /// save map data from icp
  std::string file_path_; /// filepath to save data

  double compution_time_; /// time needed to compute

  // Parameters for ICP
  int icp_max_iterations_; /// maximum of iteration (icp)
  int *pctr_; /// pointer to int value, representing loop count (only needed to save data for debugging)
  double  icp_max_corr_dist_,                   /// maximum correspondence distance (icp)
          icp_max_corr_dist_on_first_;          /// maximum correspondence distance until first is false (icp)
  double icp_trf_epsilon_;                      /// epsilon parameter of icp

  StampedTransform transform_old_;      /// last transformation (this won't be reused)

  ipa_env_model::FieldOfViewSegmentation<Point> seg_;           /// segmentationo for field of view culling

  Eigen::Vector3d n_up_t_;
  Eigen::Vector3d n_down_t_;
  Eigen::Vector3d n_right_t_;
  Eigen::Vector3d n_left_t_;
  Eigen::Vector3d n_origin_t_;
  Eigen::Vector3d n_max_range_t_;
  Eigen::Matrix4f old_icp_transform_;   /// last transformation of icp (which can be reused)

  //debug infos
  double fitness_;      /// fitness level


  bool doFOVICP(const pcl::PointCloud<Point>::Ptr& pc,
           pcl::PointCloud<Point>& pc_aligned,
           Eigen::Matrix4f& final_transformation, const cob_env_model_msgs::GetFieldOfView *get_fov_srv);
  bool doFOVICPUsingReference(const pcl::PointCloud<Point>::Ptr& pc,
           pcl::PointCloud<Point>& pc_aligned,
           Eigen::Matrix4f& final_transformation);
  void doICP(const pcl::PointCloud<Point>::Ptr& pc);
};
