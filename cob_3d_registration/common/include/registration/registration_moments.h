/*
 * registration_moments.h
 *
 *  Created on: Nov 11, 2011
 *      Author: goa-jh
 */

#ifndef REGISTRATION_MOMENTS_H_
#define REGISTRATION_MOMENTS_H_

#include "registration_icp.h"
#include "features/moments.h"

template <typename Point>
class Registration_ICP_Features : public Registration_ICP<Point>
{
  void setFeatures(const FeatureContainerInterface* features) {features_ = features;}
protected:

  const FeatureContainerInterface* features_;

  virtual void setSettingsForICP(ModifiedICP<Point> &icp);
};

#endif /* REGISTRATION_MOMENTS_H_ */
