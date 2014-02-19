/*
 * nurbs.hpp
 *
 *  Created on: 14.01.2013
 *      Author: josh
 */

#include <cob_3d_segmentation/eval.h>


template <typename Point, typename PointLabel>
bool Segmentation_NURBS<Point, PointLabel>::compute() {
  shapes_.clear();

  pcl::on_nurbs::SequentialFitter sf;

  sf.setInputCloud(input_);
  fs.setCamera(Eigen::Matrix3f::Identity());


  sf.compute();

  SHAPE_S shape;
  shape.nurbs_ = sf.getNurbs();

  shapes_.push_back( shape );

  return true;
}

template <typename Point, typename PointLabel>
boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_NURBS<Point, PointLabel>::getReconstructedOutputCloud() {
  boost::shared_ptr<pcl::PointCloud<PointLabel> > cloud (new pcl::PointCloud<PointLabel>);
  cloud->header = input_->header;

  /*for(size_t i=0; i<shapes_.size(); i++) {
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
  }*/

  return cloud;
}

template <typename Point, typename PointLabel>
void Segmentation_NURBS<Point, PointLabel>::compute_accuracy(float &mean, float &var, size_t &used, size_t &mem, size_t &points, float &avg_dist) {
  RunningStat rstat;
  points = 0;
  avg_dist = 0;
  mem = 0;

  pcl::on_nurbs::SequentialFitter sf;

  for(size_t i=0; i<input_.size(); i++) {
    Eigen::Vector3d pt, pta;
    pt(0) = (*input)[i].x;
    pt(1) = (*input)[i].y;
    pt(2) = (*input)[i].z;
    if(!pcl_isfinite(pt.sum())) continue;

    Eigen::Vector2d pt2;
    sf.getClosestPointOnNurbs(shapes_[0].nurbs_, pt, pt2);

    double p[3];
    surf.Evaluate (pt2(0), pt2(1), 0, 3, p);
    for(int j=0; j<3; j++) pta(j) = p[j];

    mem+=shapes_[i].inliers_.size()*4;
    mem+=shapes_[i].coeff_.rows()*shapes_[i].coeff_.cols()*4+8;

    const float d = (pt-pta).norm();

    if(!pcl_isfinite(d)) continue;

    rstat.Push(d);
    avg_dist += z;
    ++points;
  }

  //points = levels_[0].w*levels_[0].h;
  used = rstat.NumDataValues();

  mean = rstat.Mean();
  var = rstat.Variance();
  avg_dist /= points;
}
