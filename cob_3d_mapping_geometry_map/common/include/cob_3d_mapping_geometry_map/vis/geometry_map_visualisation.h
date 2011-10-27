#ifndef __GEOMETRY_MAP_VISUALISATION_H__
#define __GEOMETRY_MAP_VISUALISATION_H__


//--
#include <sstream>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>



#include "cob_3d_mapping_geometry_map/geometry_map.h"


class GeometryMapVisualisation
{



public:

// Constructor
GeometryMapVisualisation()
{
	//void
}
// Destructor
~GeometryMapVisualisation()
{
	//void
}
void
showPolygon(GeometryMap::MapEntryPtr polygon , int id);

void
showPolygon2(GeometryMap::MapEntryPtr polygon , int id);
void
getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,
                                                   const Eigen::Vector3f &origin, Eigen::Affine3f &transformation);


void
getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
                                            Eigen::Vector3f &u, Eigen::Vector3f &v);
private:

};


#endif //__GEOMETRY_MAP_H__
