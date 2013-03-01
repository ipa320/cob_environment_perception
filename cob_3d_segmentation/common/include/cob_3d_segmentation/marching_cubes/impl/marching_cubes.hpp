/*
 * ransac.hpp
 *
 *  Created on: 14.01.2013
 *      Author: josh
 */

#include <cob_3d_segmentation/eval.h>


template <typename Point, typename PointTypeNormal, typename PointLabel>
bool Segmentation_MarchingCubes<Point, PointTypeNormal, PointLabel>::compute() {
  pcl::PolygonMesh mesh;
  // Normal estimation*
  pcl::NormalEstimation<Point, PointTypeNormal> norm_est;
  boost::shared_ptr<pcl::PointCloud<PointTypeNormal> > pointcloudNormal (new pcl::PointCloud<PointTypeNormal>);
  boost::shared_ptr<pcl::KdTreeFLANN<Point> > tree (new pcl::KdTreeFLANN<Point>);
  tree->setInputCloud (input_);
  norm_est.setInputCloud (input_);
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  norm_est.compute (*pointcloudNormal);

  // Concatenate the XYZ and normal fields*
  pcl::copyPointCloud (*input_, *pointcloudNormal);

  // Create the search method
  boost::shared_ptr<pcl::KdTreeFLANN<PointTypeNormal> > tree2 (new pcl::KdTreeFLANN<PointTypeNormal>);
  tree2->setInputCloud (pointcloudNormal);

  // Initialize objects
  pcl::MarchingCubesGreedy<PointTypeNormal> mc;
  // Set parameters
  mc.setLeafSize(leafSize_);
  mc.setIsoLevel(isoLevel_);   //ISO: must be between 0 and 1.0
  mc.setSearchMethod(tree2);
  mc.setInputCloud(pointcloudNormal);
  // Reconstruct
  mc.reconstruct (mesh);

  mesh_ = mesh;


  std::cout<<mesh_.polygons.size()<<"\n";

  return true;
}

template <typename Point, typename PointTypeNormal, typename PointLabel>
boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_MarchingCubes<Point, PointTypeNormal, PointLabel>::getReconstructedOutputCloud() {
  boost::shared_ptr<pcl::PointCloud<PointLabel> > cloud (new pcl::PointCloud<PointLabel>);
  cloud->header = input_->header;

  pcl::PointCloud<pcl::PointXYZ> tmp;
  pcl::fromROSMsg(mesh_.cloud, tmp);

  pcl::copyPointCloud (tmp, *cloud);

  return cloud;
}

template <typename Point, typename PointTypeNormal, typename PointLabel>
void Segmentation_MarchingCubes<Point, PointTypeNormal, PointLabel>::compute_accuracy(float &mean, float &var, size_t &used, size_t &mem, size_t &points, float &avg_dist) {
  RunningStat rstat;
  points = 0;
  avg_dist = 0;
  mem = 0;

  pcl::PointCloud<pcl::PointXYZ> tmp;
  pcl::fromROSMsg(mesh_.cloud, tmp);

  for(size_t j=0; j<input_->size(); j++) {
    Eigen::Vector3f s = (*input_)[j].getVector3fMap();

    float dmax = std::numeric_limits<float>::max();
    for(size_t i=0; i<mesh_.polygons.size(); i++) {
      Eigen::Vector3f a,b,c,d;

      a = tmp[mesh_.polygons[i].vertices[0]].getVector3fMap();
      b = tmp[mesh_.polygons[i].vertices[1]].getVector3fMap();
      c = tmp[mesh_.polygons[i].vertices[2]].getVector3fMap();

//      std::cout<<a<<"\n";
//      std::cout<<b<<"\n";
//      std::cout<<c<<"\n\n";

      float x = (s-a).dot(b-a)/(b-a).squaredNorm();
      float y = (s-a).dot(c-a)/(c-a).squaredNorm();

      if(x<0 || x>1 || y<0 || y>1) continue;

      d = (b-a).cross(c-a);
      float t = std::abs(d.dot(s-a))/d.norm();

      dmax = std::min(dmax,t);

      //(b-a) (c-a)
    }

    if(dmax<0.5) {
      rstat.Push(dmax);
      avg_dist += s(2);
      ++points;
    }

    /*boost::shared_ptr<pcl::SampleConsensusModel<Point> >
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
    }*/
  }

  //points = levels_[0].w*levels_[0].h;
  used = rstat.NumDataValues();

  mean = rstat.Mean();
  var = rstat.Variance();
  avg_dist /= points;
  used = tmp.size();
}
