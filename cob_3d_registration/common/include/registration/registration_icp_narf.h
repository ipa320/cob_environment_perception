/*
 * registration_icp_narf.h
 *
 *  Created on: Nov 15, 2011
 *      Author: goa-jh
 */

#ifndef REGISTRATION_ICP_NARF_H_
#define REGISTRATION_ICP_NARF_H_

#include "registration_icp.h"
#include "features/narf.h"

template <typename Point>
class Registration_ICP_NARF : public Registration_ICP_Features<Point>
{
  Feature_NARF<Point> *mom_;
  pcl::PointCloud<Point> tmp_input_;

public:
  Registration_ICP_NARF():
    mom_(new Feature_NARF<Point>)
  {
    this->setCorrDist(0.1);
    this->setFeatures(mom_);
  }

  virtual ~Registration_ICP_NARF() {
    delete mom_;
  }

  virtual boost::shared_ptr<const pcl::PointCloud<Point> > getMarkers() {return this->input_;}

protected:
  virtual bool compute_features() {
    //save input first
    tmp_input_ = *this->input_;

    //transform pc first
    if(!this->Registration_ICP_Features<Point>::compute_features())
      return false;

    mom_->setSearchRadius(this->icp_max_corr_dist_);
    if(this->register_.size()>0) {
      mom_->build(this->register_, *this->input_);
    }

    return true;
  }

  virtual bool compute_transformation() {
    ROS_INFO("compute_transformation");
    bool b;

    if(this->register_.size()>0) {
      pcl::PointCloud<Point> tmp = this->register_;

      //replace input cloud...
      this->register_= mom_->getFilteredInputCloud();
      this->input_  = mom_->getFilteredOutputCloud().makeShared();

      b=Registration_ICP_Features<Point>::compute_transformation();

      this->register_ = tmp;
      if(b) {
        pcl::transformPointCloud(tmp_input_, tmp_input_, this->transformation_);
        this->register_+= tmp_input_;
      }

    }
    else
      b=Registration_ICP_Features<Point>::compute_transformation();

    ROS_INFO("done");
    return b;
  }
};

#endif /* REGISTRATION_ICP_NARF_H_ */
