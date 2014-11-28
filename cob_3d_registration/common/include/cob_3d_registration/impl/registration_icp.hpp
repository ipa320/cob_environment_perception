/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: dynamic_tutorials
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-jh
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Oct 26, 2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

/*
 * point_map.cpp
 *
 * Created on: Sep 6, 2011
 * Author: goa-jh
 */


#ifndef REGISTRATION_ICP_HPP_
#define REGISTRATION_ICP_HPP_

namespace cob_3d_registration {

  template <typename Point>
  bool Registration_ICP<Point>::compute_features()
  {

    //pre-transform input data to achieve incremental transformation
    boost::shared_ptr<pcl::PointCloud<Point> > transformed_pc(new pcl::PointCloud<Point>);
    pcl::transformPointCloud(*this->input_, *transformed_pc, this->transformation_);
    this->setInputCloud(transformed_pc);

    return true;
  }

  template <typename Point>
  bool Registration_ICP<Point>::compute_corrospondences()
  {
    return true;
#if 0
    if(register_.size()==0) { //first time
      register_ = *this->input_;
      return true;
    }

    float radius_=0.1;

    // Initialize estimators for surface normals and FPFH features
    boost::shared_ptr<pcl::KdTreeFLANN<Point> > tree (new pcl::KdTreeFLANN<Point>);

    pcl::NormalEstimation<Point, pcl::Normal> norm_est;
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (radius_);
    pcl::PointCloud<pcl::Normal> normals;

    pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setSearchMethod (tree);
    fpfh_est.setRadiusSearch (radius_);
    pcl::PointCloud<pcl::FPFHSignature33> features_source, features_target;

    // Estimate the FPFH features for the source cloud
    norm_est.setInputCloud (this->input_);
    norm_est.compute (normals);
    ROS_INFO("abc 1");
    fpfh_est.setInputCloud (this->input_);
    fpfh_est.setInputNormals (normals.makeShared ());
    fpfh_est.compute (features_source);

    // Estimate the FPFH features for the target cloud
    norm_est.setInputCloud (register_.makeShared());
    norm_est.compute (normals);
    ROS_INFO("abc 2");
    fpfh_est.setInputCloud (register_.makeShared());
    fpfh_est.setInputNormals (normals.makeShared ());
    fpfh_est.compute (features_target);

    // Initialize Sample Consensus Initial Alignment (SAC-IA)
    pcl::SampleConsensusInitialAlignment<Point, Point, pcl::FPFHSignature33> reg;
    reg.setMinSampleDistance (radius_);
    reg.setMaxCorrespondenceDistance (icp_max_corr_dist_);
    reg.setMaximumIterations (1000);

    reg.setInputCloud (this->input_);
    reg.setInputTarget (register_.makeShared());
    reg.setSourceFeatures (features_source.makeShared ());
    reg.setTargetFeatures (features_target.makeShared ());

    // Register
    pcl::PointCloud<Point> result;
    reg.align (result);

    this->transformation_ = reg.getFinalTransformation();

    std::cout<<"transf\n"<<this->transformation_<<"\n";

    register_ += result;
#endif
    return true;
  }

  template <typename Point>
  bool Registration_ICP<Point>::compute_transformation()
  {
    if(register_.size()==0) { //first time
      register_ = *this->input_;
      return true;
    }

    //do ICP
    ModifiedICP<Point> icp_;
    pcl::IterativeClosestPoint<Point,Point> *icp = &icp_;

#ifdef GICP_ENABLE
    ModifiedGICP<Point> gicp_;
    if(use_gicp_)
      icp = &gicp_;
    setSettingsForICP(&gicp_);
#else
    setSettingsForICP(&icp_);
#endif

    icp->setInputSource(this->input_);
    //icp->setIndices(boost::make_shared<pcl::PointIndices>(indices));
    icp->setInputTarget(register_.makeShared());
    icp->setMaximumIterations(icp_max_iterations_);
    icp->setRANSACOutlierRejectionThreshold(outlier_rejection_threshold_);
    icp->setMaxCorrespondenceDistance(icp_max_corr_dist_);
    icp->setTransformationEpsilon (icp_trf_epsilon_);

    ROS_INFO("icp with %d, %d", (int)register_.size(), (int)this->input_->size());

    pcl::PointCloud<Point> result;
    icp->align(result);

    bool res = false;

#ifdef GICP_ENABLE
    if(use_gicp_)
      res=gicp_.getMaximumIterations()!=gicp_.getNeededIterations()&&gicp_.getNeededIterations()>0;
    else
#endif
      res=icp_.getMaximumIterations()!=icp_.getNeededIterations()&&icp_.getNeededIterations()>0;

    if(!res)
      return false;

    this->transformation_ = this->transformation_*icp->getFinalTransformation();

    std::cout<<"transf\n"<<this->transformation_<<"\n";

    if(use_only_last_refrence_)
      register_ = result;
    else
      register_ += result;

    return res;
  }

  template <typename Point>
  void Registration_ICP<Point>::setSettingsForICP(ModifiedICP_G *icp) {
    if(non_linear_)
      icp->setLM();
  }

  template <typename Point>
  void Registration_ICP_Features<Point>::setSettingsForICP(ModifiedICP_G *icp) {
    Registration_ICP<Point>::setSettingsForICP(icp);

    this->Registration_ICP<Point>::setSettingsForICP(icp);

    if(features_)
      icp->setSearchFeatures(features_);
  }


  template <typename Point, typename FeatureType>
  bool Registration_ICP_Features_Extra<Point,FeatureType>::compute_features()
  {
    boost::shared_ptr<pcl::PointCloud<Point> > inp = boost::shared_ptr<pcl::PointCloud<Point> >(new pcl::PointCloud<Point>);
    boost::shared_ptr<pcl::PointCloud<Point> > out = boost::shared_ptr<pcl::PointCloud<Point> >(new pcl::PointCloud<Point>);

    if(!calculateFeature(((Registration_ICP_Features<Point>*)this)->input, inp, ((Registration_ICP_Features<Point>*)this)->features_.getTargetFeature()))
      return false;

    if(!calculateFeature(((Registration_ICP_Features<Point>*)this)->register_, out, ((Registration_ICP_Features<Point>*)this)->features_.getSourceFeature()))
      return false;

    ((Registration_ICP_Features<Point>*)this)->input_    = inp;
    ((Registration_ICP_Features<Point>*)this)->register_ = out;

    return true;
  }
  /*
template <typename Point, typename FeatureType>
bool Registration_ICP_Features_Extra<Point,FeatureType>::calculateFeature(boost::shared_ptr<pcl::PointCloud<Point> > input, boost::shared_ptr<pcl::PointCloud<Point> > output, boost::shared_ptr<pcl::PointCloud<FeatureType> > features)
{
  ASSERT(input.size() == );

  findFeatureCorrespondences

}
   */


}

#define PCL_INSTANTIATE_Registration_ICP(T) template class PCL_EXPORTS cob_3d_registration::Registration_ICP<T>;
#define PCL_INSTANTIATE_Registration_ICP_Features(T) template class PCL_EXPORTS cob_3d_registration::Registration_ICP_Features<T>;


#endif
