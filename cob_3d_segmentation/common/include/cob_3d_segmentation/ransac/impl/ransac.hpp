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
      ROS_INFO("add %d %d",shape.type_, (int)shape.inliers_.size());
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

template <typename Point, typename PointLabel>
void Segmentation_RANSAC<Point,PointLabel>::compute_accuracy(float &mean, float &var, float &mean_weighted, float &var_weighted, size_t &used, size_t &mem, size_t &points, float &avg_dist, const boost::shared_ptr<const pcl::PointCloud<PointLabel> > &labeled_pc, double &true_positive, double &false_positive)
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
