/*
 * registration_icp_normals.h
 *
 *  Created on: Nov 14, 2011
 *      Author: goa-jh
 */

#ifndef REGISTRATION_ICP_NORMALS_H_
#define REGISTRATION_ICP_NORMALS_H_

#include "registration_icp.h"
#include "features/fast_pfh.h"

namespace cob_3d_registration {

template <typename Point>
class Registration_ICP_FPFH : public Registration_ICP_Features<Point>
{
  Feature_FPFH<Point> *mom_;

public:
  Registration_ICP_FPFH():
    mom_(new Feature_FPFH<Point>)
  {
    this->setCorrDist(0.1);
    this->setFeatures(mom_);
  }

  virtual ~Registration_ICP_FPFH() {
    delete mom_;
  }

  void setFPFHRadius(float v) {mom_->setFPFHRadius(v);}

protected:
  virtual bool compute_features() {
    //transform pc first
    if(!this->Registration_ICP_Features<Point>::compute_features())
      return false;

    mom_->setSearchRadius(this->icp_max_corr_dist_);
    if(this->register_.size()>0)
      mom_->build(this->register_, *this->input_);

    return true;
  }
};

}

#endif /* REGISTRATION_ICP_NORMALS_H_ */
