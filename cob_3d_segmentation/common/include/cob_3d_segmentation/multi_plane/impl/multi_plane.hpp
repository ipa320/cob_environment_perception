/*
 * ransac.hpp
 *
 *  Created on: 14.01.2013
 *      Author: josh
 */

#include <cob_3d_segmentation/eval.h>


template <typename Point, typename PointTypeNormal, typename PointLabel>
bool Segmentation_MultiPlane<Point, PointTypeNormal,PointLabel>::compute() {
  coef.clear();
  regions.clear();
  inlier_indices.clear();
  label_indices.clear();
  boundary_indices.clear();
  typename pcl::IntegralImageNormalEstimation<Point, PointTypeNormal> ne;
  typename pcl::PointCloud<PointTypeNormal>::Ptr n(new pcl::PointCloud<PointTypeNormal>());

l.reset(new pcl::PointCloud<pcl::Label>());

    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(40.0f);

  ne.setInputCloud(input_);
  ne.compute(*n);
  float* distance_map = ne.getDistanceMap();

  typename pcl::EdgeAwarePlaneComparator<Point, PointTypeNormal>::Ptr comparator;
  typename pcl::OrganizedMultiPlaneSegmentation<Point, PointTypeNormal, pcl::Label> omps;

    omps.setAngularThreshold(2.5f/180.0f*M_PI);
    omps.setMaximumCurvature(0.01); // default 0.001
    omps.setDistanceThreshold(0.01f);
    omps.setMinInliers(100);

  comparator.reset(new pcl::EdgeAwarePlaneComparator<Point, PointTypeNormal>(distance_map));
  omps.setComparator(comparator);
  omps.setInputCloud(input_);
  omps.setInputNormals(n);
  omps.segmentAndRefine(regions, coef, inlier_indices, l, label_indices, boundary_indices);

  ROS_INFO("finished");

  return true;
}

template <typename Point, typename PointTypeNormal, typename PointLabel>
boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_MultiPlane<Point, PointTypeNormal,PointLabel>::getReconstructedOutputCloud() {
  boost::shared_ptr<pcl::PointCloud<PointLabel> > cloud (new pcl::PointCloud<PointLabel>);
  cloud->header = input_->header;

for(size_t i=0; i<inlier_indices.size(); i++) {
    boost::shared_ptr<pcl::SampleConsensusModel<Point> >
    model;
    pcl::PointCloud<Point> temp;
    pcl::PointCloud<PointLabel> temp2;

	for(size_t j=0; j<inlier_indices[i].indices.size(); j++) {
		temp.push_back( (*input_)[inlier_indices[i].indices[j]] );
	}

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
void Segmentation_MultiPlane<Point,PointTypeNormal,PointLabel>::compute_accuracy(float &mean, float &var, float &mean_weighted, float &var_weighted, size_t &used, size_t &mem, size_t &points, float &avg_dist, const boost::shared_ptr<const pcl::PointCloud<PointLabel> > &labeled_pc, double &true_positive, double &false_positive)
{
  BinaryClassification bc;
  RunningStat rstat;
  points = 0;
  avg_dist = 0;
  mem = 0;

  bc.addPC(labeled_pc);

std::cout<<l->size()<<"\n";
  for(size_t i=0; i<l->size(); i++) {
      if(labeled_pc)
        bc.addMark((*l)[i].label, (*labeled_pc)[i].label);

	if(pcl_isfinite((*input_)[i].z)) {
      	++points;
      	avg_dist += (*input_)[i].z;}
  }

   for(size_t ind=0; ind<regions.size(); ind++) {
mem += 4*4;
  for(size_t i=0; i<regions[ind].getContour ().size(); i++) {
		float d = std::abs( regions[ind].getCoefficients().head(3).dot( regions[ind].getContour ()[i].getVector3fMap() ) + regions[ind].getCoefficients()(3) );
      		rstat.Push(d);}
  }

for(size_t i=0; i<boundary_indices.size(); i++)
  mem += 4*boundary_indices[i].indices.size();

#if 0
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
#endif

  bc.finish().get_rate(true_positive, false_positive);

  //points = levels_[0].w*levels_[0].h;
  used = rstat.NumDataValues();

  mean = rstat.Mean();
  var = rstat.Variance();
  avg_dist /= points;
}
