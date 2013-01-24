/*
 * ransac.hpp
 *
 *  Created on: 14.01.2013
 *      Author: josh
 */

#include <cob_3d_segmentation/eval.h>


template <typename Point, typename PointLabel>
bool Segmentation_RANSAC<Point, PointLabel>::compute() {
  shapes_.clear();

  bool found = true;
  boost::shared_ptr<pcl::PointCloud<Point> > cloud (new pcl::PointCloud<Point>(*input_));

  while(found) {
    found = false;
    SHAPE_S shape;

    if(!found && planes_) {
      boost::shared_ptr<pcl::SampleConsensusModelPlane<Point> >
        model (new pcl::SampleConsensusModelPlane<Point> (cloud));

      pcl::RandomSampleConsensus<Point> ransac (model);
      ransac.setDistanceThreshold (.03);
      ransac.computeModel();
      ransac.getInliers(shape.inliers_);

      if(shape.inliers_.size()>0) {
        ransac.getModelCoefficients(shape.coeff_);
        shape.type_ = SHAPE_S::PLANE;
        found = true;
      }
    }

    if(!found && spheres_) {
      boost::shared_ptr<pcl::SampleConsensusModelSphere<Point> >
        model (new pcl::SampleConsensusModelSphere<Point> (cloud));

      pcl::RandomSampleConsensus<Point> ransac (model);
      ransac.setDistanceThreshold (.03);
      ransac.computeModel();
      ransac.getInliers(shape.inliers_);

      if(shape.inliers_.size()>0) {
        ransac.getModelCoefficients(shape.coeff_);
        shape.type_ = SHAPE_S::SPHERE;
        found = true;
      }
    }

    if(found) {
      ROS_INFO("add %d %d",shape.type_, shape.inliers_.size());
      shapes_.push_back( shape );
      for(size_t i=0; i<shape.inliers_.size(); i++)
        (*cloud)[shape.inliers_[i]].x = (*cloud)[shape.inliers_[i]].y = (*cloud)[shape.inliers_[i]].z =
            std::numeric_limits<float>::quiet_NaN();
    }

  }
  ROS_INFO("finished");

  return true;
}

template <typename Point, typename PointLabel>
 boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_RANSAC<Point, PointLabel>::getReconstructedOutputCloud() {
  boost::shared_ptr<pcl::PointCloud<PointLabel> > cloud (new pcl::PointCloud<PointLabel>);
  cloud->header = input_->header;

  for(size_t i=0; i<shapes_.size(); i++) {
    boost::shared_ptr<pcl::SampleConsensusModel<Point> >
      model;
    pcl::PointCloud<PointLabel> temp;

    if(shapes_[i].type_==SHAPE_S::PLANE)
      model.reset(new pcl::SampleConsensusModelPlane<Point> (input_));
    //else if(shapes_[i].type_==SHAPE_S::CYLINDER)
    //  model.reset(new pcl::SampleConsensusModelCylinder<Point,Point> (input_));
    else if(shapes_[i].type_==SHAPE_S::SPHERE)
      model.reset(new pcl::SampleConsensusModelSphere<Point> (input_));
    else
      ROS_ASSERT(0);

    model->projectPoints(shapes_[i].inliers_, shapes_[i].coeff_, temp, false);

    *cloud += temp;
  }

  return cloud;
}

template <typename Point, typename PointLabel>
void Segmentation_RANSAC<Point, PointLabel>::compute_accuracy(float &mean, float &var, size_t &used, size_t &mem, size_t &points, float &avg_dist) {
  RunningStat rstat;
  points = 0;
  avg_dist = 0;
  mem = 0;

  for(size_t i=0; i<shapes_.size(); i++) {
    boost::shared_ptr<pcl::SampleConsensusModel<Point> >
      model;
    pcl::PointCloud<PointLabel> temp;

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

    for(size_t j=0; j<shapes_[i].inliers_.size(); j++) {
      Point pr = (*input_)[shapes_[i].inliers_[j]];
      Point pm = temp[j];
      const float z = pr.z;
      const float d = (pr.getVector3fMap()-pm.getVector3fMap()).norm();

      if(!pcl_isfinite(d)) continue;

      rstat.Push(d);
      avg_dist += z;
      ++points;
    }
  }

  //points = levels_[0].w*levels_[0].h;
  used = rstat.NumDataValues();

  mean = rstat.Mean();
  var = rstat.Variance();
  avg_dist /= points;
}
