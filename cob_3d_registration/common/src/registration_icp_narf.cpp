/*
 * registration_icp_narf.cpp
 *
 *  Created on: 19.11.2012
 *      Author: josh
 */



// external includes:
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

// package includes:
#include "cob_3d_registration/general_registration.h"
#include "cob_3d_registration/registration_icp_narf.h"


#define PCL_INSTANTIATE_Registration_ICP_NARF(T) template class PCL_EXPORTS cob_3d_registration::Registration_ICP_NARF<T>;

PCL_INSTANTIATE(Registration_ICP_NARF, PCL_XYZ_POINT_TYPES)




