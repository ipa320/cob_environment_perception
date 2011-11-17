/*
 * GeometryMapTest.cpp
 *
 *  Created on: 04.11.2011
 *      Author: goa-hh
 */
#include "cob_3d_mapping_geometry_map/vis/geometry_map_visualisation.h"
#include "cob_3d_mapping_geometry_map/test/TestPlanes.h"
#include "cob_3d_mapping_geometry_map/geometry_map.h"
#include "cob_3d_mapping_geometry_map/map_entry.h"
#include "cob_3d_mapping_geometry_map/test/geometry_map_test.h"




GeometryMapTest::GeometryMapTest() {}

GeometryMapTest::~GeometryMapTest() {}


	void GeometryMapTest::fillingHole()
	{
		MapEntryPtr m_p = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p3 = MapEntryPtr(new MapEntry());

		tp.moveX1X2(m_p ,0,0,1);
		tp.moveX1X2(m_p3 ,1.5,0,1);

		gm.addMapEntry(m_p);
		gm.addMapEntry(m_p3);

		gmv.showMap(gm.getMap());
		tp.moveX1X2(m_p2 ,0.7,0,1);

		gm.addMapEntry(m_p2);

		gmv.showMap(gm.getMap());
	}



	void GeometryMapTest::box()
	{

			MapEntryPtr m_p = MapEntryPtr(new MapEntry());
			MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());
			MapEntryPtr m_p3 = MapEntryPtr(new MapEntry());
			MapEntryPtr m_p4 = MapEntryPtr(new MapEntry());
			MapEntryPtr m_p5 = MapEntryPtr(new MapEntry());
			MapEntryPtr m_p6 = MapEntryPtr(new MapEntry());
			MapEntryPtr m_p7 = MapEntryPtr(new MapEntry());

			tp.figure_1(m_p,m_p2,m_p3,m_p4,m_p5);
			tp.moveX1X2(m_p6 ,0.8,0.8,1);

			tp.moveX1X2(m_p7 ,1.2,1.2,1);
			gm.addMapEntry(m_p);
			gm.addMapEntry(m_p2);
			gm.addMapEntry(m_p3);
			gm.addMapEntry(m_p4);
			gm.addMapEntry(m_p5);
			gmv.showMap(gm.getMap());
			gm.addMapEntry(m_p7);
			gmv.showMap(gm.getMap());
			gm.addMapEntry(m_p6);
			gmv.showMap(gm.getMap());
	}

	void GeometryMapTest::rotate()
	{
		MapEntryPtr m_p = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());

			tp.rotate(m_p,0,0,125);
			gm.addMapEntry(m_p);
			//gmv.showMap(gm.getMap());
			tp.rotate(m_p2,0,0,143.1);                                   // intersection wenn winkel kleiner wie 18.5crad
			gm.addMapEntry(m_p2);
			gmv.showMap(gm.getMap());



	}



int main (int argc, char** argv)
{

	GeometryMapTest gmt;
	//gmt.box();
	//gmt.fillingHole();
	gmt.rotate();
//	TestPlanes tp;
//	GeometryMapVisualisation gmv;
//	GeometryMap gm;


//	MapEntryPtr m_p = MapEntryPtr(new MapEntry());
//	MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());
//	MapEntryPtr m_p3 = MapEntryPtr(new MapEntry());
//	MapEntryPtr m_p4 = MapEntryPtr(new MapEntry());
//	MapEntryPtr m_p5 = MapEntryPtr(new MapEntry());
//	MapEntryPtr m_p6 = MapEntryPtr(new MapEntry());
//	MapEntryPtr m_p7 = MapEntryPtr(new MapEntry());

//	tp.rotate(m_p,0,0,10);


//	//tp.plane_1(m_p2);
//
//	tp.figure_1(m_p,m_p2,m_p3,m_p4,m_p5);
//	tp.moveX1X2(m_p6 ,0.8,0.8,1);
//
//	tp.moveX1X2(m_p7 ,1.2,1.2,1);
//	gm.addMapEntry(m_p);
//	gm.addMapEntry(m_p2);
////	gm.addMapEntry(m_p3);
//	gm.addMapEntry(m_p4);
////	gm.addMapEntry(m_p5);
//	gmv.showMap(gm.getMap());
//	gm.addMapEntry(m_p7);
//	gmv.showMap(gm.getMap());
//	gm.addMapEntry(m_p6);
//	gmv.showMap(gm.getMap());





//	//tp.moveplane(m_p,m_p2,0.5,0.5,0);
//	//tp.overlap(m_p,m_p2,1.9999);
//	//gmv.showPolygon(m_p);
//	//gmv.showPolygon(m_p4);
//
//

//	gmv.showMap(gm.getMap());
//

//	gmv.showPolygon(m_p6);

//	gm.addMapEntry(m_p6,merged);
//
//	m_p7 = (*gm.getMap())[1];
//
//	gmv.showPolygon(m_p7);


//	tp.moveX1X2(m_p ,0,0,1);
//	tp.moveX1X2(m_p3 ,1.5,0,1);
//
//	gm.addMapEntry(m_p);
//	gm.addMapEntry(m_p3);
//
//	gmv.showMap(gm.getMap());
//	tp.moveX1X2(m_p2 ,0.7,0,1);
//
//	gm.addMapEntry(m_p2);
//
//	gmv.showMap(gm.getMap());

//	gm.removeMapEntry(2);
//	gmv.showMap(gm.getMap());
//	}*/
}
