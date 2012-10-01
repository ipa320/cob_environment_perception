/*
 * registration_ipc_moments.h
 *
 *  Created on: Nov 14, 2011
 *      Author: goa-jh
 */

#ifndef REGISTRATION_IPC_MOMENTS_H_
#define REGISTRATION_IPC_MOMENTS_H_

#include "registration_icp.h"
#include "features/moments.h"

template <typename Point>
class Registration_ICP_Moments : public Registration_ICP_Features<Point>
{
  Feature_MomentInvariants<Point> *mom_;

public:
  Registration_ICP_Moments():
    mom_(new Feature_MomentInvariants<Point>)
  {
    this->setCorrDist(0.1);
    this->setFeatures(mom_);
  }

  virtual ~Registration_ICP_Moments() {
    delete mom_;
  }

  void setMomentRadius(float v) {mom_->setMomentRadius(v);}

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

#endif /* REGISTRATION_IPC_MOMENTS_H_ */
