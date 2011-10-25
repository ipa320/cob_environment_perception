#ifndef __FEATURE_MAP_VISUALISATION_H__
#define __FEATURE_MAP_VISUALISATION_H__


//--
#include <sstream>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>



#include "cob_env_model/map/feature_map.h"


class FeatureMapVisualisation
{



public:

// Constructor
FeatureMapVisualisation()
{
	//void
}
// Destructor
~FeatureMapVisualisation()
{
	//void
}
void
showPolygon(FeatureMap::MapEntryPtr polygon , int id);


void
getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,
                                                   const Eigen::Vector3f &origin, Eigen::Affine3f &transformation);


void
getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
                                            Eigen::Vector3f &u, Eigen::Vector3f &v);
private:

};


#endif //__FEATURE_MAP_H__
