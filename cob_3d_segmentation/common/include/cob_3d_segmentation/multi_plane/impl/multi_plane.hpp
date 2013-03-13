/*
 * ransac.hpp
 *
 *  Created on: 14.01.2013
 *      Author: josh
 */

#include <cob_3d_segmentation/eval.h>


template <typename Point, typename PointTypeNormal, typename PointLabel>
bool Segmentation_MultiPlane<Point, PointLabel>::compute() {
  coef.clear();
  regions.clear();
  inlier_indices.clear();
  label_indices.clear();
  boundary_indices.clear();
  ne.setInputCloud(input);
  ne.compute(*n);
  float* distance_map = ne.getDistanceMap();
  std::cout << t.precisionStop() << " | ";

  pcl::EdgeAwarePlaneComparator<PointT, NormalT>::Ptr comperator;

  t.precisionStart();
  comparator.reset(new pcl::EdgeAwarePlaneComparator<Point, PointTypeNormal>(distance_map));
  omps.setComparator(comparator);
  omps.setInputCloud(cloud);
  omps.setInputNormals(n);
  omps.segmentAndRefine(regions, coef, inlier_indices, l, label_indices, boundary_indices);
  std::cout << t.precisionStop() << std::endl;

  ROS_INFO("finished");

  return true;
}

template <typename Point, typename PointTypeNormal, typename PointLabel>
boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_MultiPlane<Point, PointLabel>::getReconstructedOutputCloud() {
  boost::shared_ptr<pcl::PointCloud<PointLabel> > cloud (new pcl::PointCloud<PointLabel>);
  cloud->header = input_->header;

  for(size_t i=0; i<shapes_.size(); i++) {
    boost::shared_ptr<pcl::SampleConsensusModel<Point> >
    model;
    pcl::PointCloud<Point> temp;
    pcl::PointCloud<PointLabel> temp2;

    if(shapes_[i].type_==SHAPE_S::PLANE)
      model.reset(new pcl::SampleConsensusModelPlane<Point> (input_));
    //else if(shapes_[i].type_==SHAPE_S::CYLINDER)
    //  model.reset(new pcl::SampleConsensusModelCylinder<Point,Point> (input_));
    else if(shapes_[i].type_==SHAPE_S::SPHERE)
      model.reset(new pcl::SampleConsensusModelSphere<Point> (input_));
    else
      ROS_ASSERT(0);

    model->projectPoints(shapes_[i].inliers_, shapes_[i].coeff_, temp, false);

    temp2.width = temp.width;
    temp2.height = temp.height;
    temp2.header = temp.header;
    temp2.resize(temp.size());

    for(size_t j=0; j<temp.size(); j++) {
      temp2[j].x=temp[j].x;
      temp2[j].y=temp[j].y;
      temp2[j].z=temp[j].z;
      SetLabeledPoint<PointLabel>( temp2[j], i);
    }

    *cloud += temp2;
  }

  return cloud;
}

template <typename Point, typename PointTypeNormal, typename PointLabel>
void Segmentation_MultiPlane<Point,PointLabel>::compute_accuracy(float &mean, float &var, float &mean_weighted, float &var_weighted, size_t &used, size_t &mem, size_t &points, float &avg_dist, const boost::shared_ptr<const pcl::PointCloud<PointLabel> > &labeled_pc, double &true_positive, double &false_positive)
{
  BinaryClassification bc;
  RunningStat rstat;
  points = 0;
  avg_dist = 0;
  mem = 0;

  bc.addPC(labeled_pc);

  for(size_t i=0; i<shapes_.size(); i++) {
    boost::shared_ptr<pcl::SampleConsensusModel<Point> >
    model;
    pcl::PointCloud<Point> temp;
    pcl::PointCloud<PointLabel> temp2;

    if(shapes_[i].type_==SHAPE_S::PLANE)
      model.reset(new pcl::SampleConsensusModelPlane<Point> (input_));
    //else if(shapes_[i].type_==SHAPE_S::CYLINDER)
    //  model.reset(new pcl::SampleConsensusModelCylinder<Point,Point> (input_));
    else if(shapes_[i].type_==SHAPE_S::SPHERE)
      model.reset(new pcl::SampleConsensusModelSphere<Point> (input_));
    else
      ROS_ASSERT(0);

    mem+=shapes_[i].inliers_.size()*4;
    mem+=shapes_[i].coeff_.rows()*shapes_[i].coeff_.cols()*4+8;

    model->projectPoints(shapes_[i].inliers_, shapes_[i].coeff_, temp, false);

    temp2.width = temp.width;
    temp2.height = temp.height;
    temp2.header = temp.header;
    temp2.resize(temp.size());

    for(size_t j=0; j<temp.size(); j++) {
      temp2[j].x=temp[j].x;
      temp2[j].y=temp[j].y;
      temp2[j].z=temp[j].z;
      SetLabeledPoint<PointLabel>( temp2[j], i);
    }

    for(size_t j=0; j<shapes_[i].inliers_.size(); j++) {
      Point pr = (*input_)[shapes_[i].inliers_[j]];
      Point pm = temp[j];
      const float z = pr.z;
      const float d = (pr.getVector3fMap()-pm.getVector3fMap()).norm();

      if(labeled_pc && shapes_[i].inliers_[j]<(int)labeled_pc->size())
        bc.addMark(i, (*labeled_pc)[shapes_[i].inliers_[j]].label);

      if(!pcl_isfinite(d)) continue;

      rstat.Push(d);
      avg_dist += z;
      ++points;
    }
  }

  bc.finish().get_rate(true_positive, false_positive);

  //points = levels_[0].w*levels_[0].h;
  used = rstat.NumDataValues();

  mean = rstat.Mean();
  var = rstat.Variance();
  avg_dist /= points;
}
