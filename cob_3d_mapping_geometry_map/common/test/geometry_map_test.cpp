/*
 * GeometryMapTest.cpp
 *
 *  Created on: 04.11.2011
 *      Author: goa-hh
 */
#include "cob_3d_mapping_geometry_map/vis/geometry_map_visualisation.h"
#include "cob_3d_mapping_geometry_map/vis/TestPlanes.h"
#include "cob_3d_mapping_geometry_map/geometry_map.h"
#include "cob_3d_mapping_geometry_map/map_entry.h"



class GeometryMapTest {

public:

	GeometryMapTest();

	~GeometryMapTest();
};

int main (int argc, char** argv)
{
	TestPlanes tp;
	GeometryMapVisualisation gmv;
	GeometryMap gm;


	MapEntryPtr m_p = MapEntryPtr(new MapEntry());
	MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());
	MapEntryPtr m_p3 = MapEntryPtr(new MapEntry());
	MapEntryPtr m_p4 = MapEntryPtr(new MapEntry());
	MapEntryPtr m_p5 = MapEntryPtr(new MapEntry());
	MapEntryPtr m_p6 = MapEntryPtr(new MapEntry());
	MapEntryPtr m_p7 = MapEntryPtr(new MapEntry());


	bool merged=true;
//	//tp.plane_1(m_p2);
//
////	while (merged){
//	tp.figure_1(m_p,m_p2,m_p3,m_p4,m_p5);
//	//tp.moveplane(m_p,m_p2,0.5,0.5,0);
//	//tp.overlap(m_p,m_p2,1.9999);
//	//x +=0.1;
//	//gmv.showPolygon(m_p);
//	//gmv.showPolygon(m_p4);
//
//
//	gm.addMapEntry(m_p,merged);
//	gm.addMapEntry(m_p2,merged);
//	gm.addMapEntry(m_p3,merged);
//	gm.addMapEntry(m_p4,merged);
//	gm.addMapEntry(m_p5,merged);
//
//	tp.moveX1X2(m_p6 ,0.5,0.5,1);
//	gmv.showPolygon(m_p6);
//
//	gm.addMapEntry(m_p6,merged);
//
//	m_p7 = (*gm.getMap())[1];
//
//	gmv.showPolygon(m_p7);
	tp.moveX1X2(m_p ,0,0,1);
	tp.moveX1X2(m_p2 ,0.5,0,1);
	tp.moveX1X2(m_p3 ,1,0,1);

	gm.addMapEntry(m_p);
	gm.addMapEntry(m_p3);
	gm.addMapEntry(m_p2);

	gmv.showMap(gm.getMap());
//	}
}
