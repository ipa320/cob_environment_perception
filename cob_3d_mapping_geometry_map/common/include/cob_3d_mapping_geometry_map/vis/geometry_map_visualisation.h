#ifndef __GEOMETRY_MAP_VISUALISATION_H__
#define __GEOMETRY_MAP_VISUALISATION_H__


//--
#include <sstream>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>


#ifndef __MAP_ENTRY_H__
#include "cob_3d_mapping_geometry_map/map_entry.h"

#endif

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
showPolygon(MapEntryPtr polygon , int id);

void
showPolygon(MapEntryPtr polygon );

void
showMap(boost::shared_ptr<std::vector<MapEntryPtr> > map);


private:

};


#endif //__GEOMETRY_MAP_H__
