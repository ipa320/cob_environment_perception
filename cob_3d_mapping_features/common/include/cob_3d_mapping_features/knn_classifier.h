/*
 * knn_classifier.h
 *
 *  Created on: 04.10.2011
 *      Author: goa-sf
 */

#ifndef COB_3D_MAPPING_FEATURES_KNN_CLASSIFIER_H_
#define COB_3D_MAPPING_FEATURES_KNN_CLASSIFIER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace cob_3d_mapping_features
{
  template<typename PointT> class KNNClassifier
  {
  public:
    KNNClassifier();

    ~KNNClassifier();

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef boost::shared_ptr<PointCloud> PointCloudPtr;
    typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
    typedef pcl::KdTree<PointT> KdTree;
    typedef boost::shared_ptr<KdTree> KdTreePtr;

    void setTrainingFeatures(const PointCloudPtr &features, 
			     std::string &distanceMetric = "L2");
    void setTrainingLabels(const std::vector<int> &labels);
    void setKNeighbors(int k);
    int saveTrainingData(std::string &features_file_name,
			 std::string &labels_file_name);
    int loadTrainingData(std::string &features_file_name,
			 std::string &labels_file_name);
    int classify(const PointT &p_q);

  protected:
    KdTreePtr tree_;
    std::vector<int> labels_;
    std::string distanceMetric_;
    int labels_max_;
    int k_;

//    int getDominantLabel(std::vector<int> 
  };
}

#endif
