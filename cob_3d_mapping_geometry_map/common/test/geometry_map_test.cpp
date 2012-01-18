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

	void GeometryMapTest::curve()
	{
		MapEntryPtr m_p = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p3 = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p4 = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p5 = MapEntryPtr(new MapEntry());

		tp.rotate(m_p,0,0,25);
		tp.rotate(m_p2,-0.75*sin(30*M_PI / 180),0.75*cos(30*M_PI / 180),12);
		tp.rotate(m_p3,-0.75*sin(30*M_PI / 180)-0.75*sin(15*M_PI / 180),0.75*cos(30*M_PI / 180)+0.75*cos(15*M_PI / 180),0);
		tp.rotate(m_p4,-0.75*sin(30*M_PI / 180)-0.75*sin(15*M_PI / 180),0.75*cos(30*M_PI / 180)+0.75*cos(15*M_PI / 180)+0.75,-12);
		tp.rotate(m_p5,-0.75*sin(30*M_PI / 180),0.75*cos(30*M_PI / 180)+2*0.75*cos(15*M_PI / 180)+0.75,-25);

		std::cout << "m_p2 d " << m_p2->d << " normal " <<std::endl << m_p2->normal << std::endl;
		std::cout << "m_p3 d " << m_p3->d << " normal " <<std::endl<< m_p3->normal << std::endl;
		std::cout << "dot " << m_p2->normal.dot(m_p3->normal) << std::endl;

		gm.addMapEntry(m_p);
		gm.addMapEntry(m_p2);
		gm.addMapEntry(m_p3);
		gm.addMapEntry(m_p4);
		gm.addMapEntry(m_p5);

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
			gmv.showPolygon(m_p);
			//gmv.showMap(gm.getMap());
			tp.rotate(m_p2,0,0,140);                                   // intersection wenn winkel kleiner wie 18.5crad
			gm.addMapEntry(m_p2);
			gmv.showMap(gm.getMap());



	}

	void GeometryMapTest::strongMerged()
	{
		MapEntryPtr m_p = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());

	//	tp.rotate(m_p,0,0,0);
	//	gm.addMapEntry(m_p);

		m_p->merged=10;

		tp.rotate(m_p2,0,0,18.5);                                   // intersection wenn winkel kleiner wie 18.5crad
		gm.addMapEntry(m_p2);
		gmv.showMap(gm.getMap());
	}

	void GeometryMapTest::testnormaldirection()
	{

		MapEntryPtr m_p = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());
		MapEntryPtr m_p3 = MapEntryPtr(new MapEntry());

		tp.moveX1X2(m_p ,0,0,1);
		gmv.showPolygon(m_p);
		tp.moveX1X2(m_p3 ,0.5,0,1);
		gmv.showPolygon(m_p3);

		m_p3->normal << 0,0,-1;
		m_p3->d=-1;

		gm.addMapEntry(m_p3);
		gm.addMapEntry(m_p);

		gmv.showMap(gm.getMap());
	}
	void GeometryMapTest::easyplane()
	{
		  MapEntryPtr m_p = MapEntryPtr(new MapEntry());

		  m_p->id = 0;
		  m_p->normal << 0,0,1;
		  m_p->d = 1;
		  std::vector<Eigen::Vector3f> vv;
		  Eigen::Vector3f v;
		  v << 1,0,1;
		  vv.push_back(v);
		  v << 1,1,1;
		  vv.push_back(v);
		  v << 0,1,1;
		  vv.push_back(v);
		  v << 0,0,1;
		  vv.push_back(v);
		  m_p->polygon_world.push_back(vv);
		  gm.addMapEntry(m_p);
		  std::cout << m_p->transform_from_world_to_plane.matrix();
		  gmv.showMap(gm.getMap());

	}

	void GeometryMapTest::bugsearch()
	{
		  MapEntryPtr m_p = MapEntryPtr(new MapEntry());
		  MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());
		  MapEntryPtr m_p3 = MapEntryPtr(new MapEntry());

		  tp.rotateTable(m_p,10.0);
		  tp.rotateTable(m_p2,5.0);
		  tp.rotateTable(m_p3,17.0);
	//	  gmv.showPolygon(m_p);
	//	  gmv.showPolygon(m_p2);
	//	  gmv.showPolygon(m_p3);



		  m_p2->normal<<0,cos((90.0-5.0)/180.0*3.1415926535),-sin((90.0-5.0)/180.0*3.1415926535);
		  m_p2->d=-1;

		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p3);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p3);
		  gm.addMapEntry(m_p2);


		  gmv.showMap(gm.getMap());
//		  m_p->id = 0;
//		  m_p->normal << 0,cos(10/180*3.1415926535),sin(10/180*3.1415926535);
//		  m_p->d = 1;
//		  std::vector<Eigen::Vector3f> vv;
//		  Eigen::Vector3f v;
//		  v << 1,cos(10/180*3.1415926535),1+sin(10/180*3.1415926535);
//		  vv.push_back(v);
//		  v << -1,cos(10/180*3.1415926535),1+sin(10/180*3.1415926535);
//		  vv.push_back(v);
//		  v << -1,-cos(10/180*3.1415926535),1-sin(10/180*3.1415926535);
//		  vv.push_back(v);
//		  v << 1,-cos(10/180*3.1415926535),1-sin(10/180*3.1415926535);
//		  vv.push_back(v);
//		  m_p->polygon_world.push_back(vv);


	}


	void GeometryMapTest::threedirection()
	{
		  MapEntryPtr m_p = MapEntryPtr(new MapEntry());
		  MapEntryPtr m_p2 = MapEntryPtr(new MapEntry());
		  MapEntryPtr m_p3 = MapEntryPtr(new MapEntry());



		  Eigen::Matrix3f x;
		  x << 1,0,0,
			   0,cos(15.0/180.0*3.1415926535),-sin(15.0/180.0*3.1415926535),
			   0,sin(15.0/180.0*3.1415926535),cos(15.0/180.0*3.1415926535);

		  Eigen::Matrix3f y;
		  y << cos(15.0/180.0*3.1415926535),0,-sin(15.0/180.0*3.1415926535),
			   0,1,0,
			   sin(15.0/180.0*3.1415926535),0,cos(15.0/180.0*3.1415926535);

		  Eigen::Matrix3f y2;
		  y2 << cos(5.0/180.0*3.1415926535),0,-sin(5.0/180.0*3.1415926535),
			   0,1,0,
			   sin(5.0/180.0*3.1415926535),0,cos(5.0/180.0*3.1415926535);
		  Eigen::Matrix3f xx;

		  x=x*y;
		  m_p->id = 0;
		  m_p->normal << 0,0,-1;
		  m_p->normal=x*m_p->normal;
		  m_p->d = m_p->normal(2);
		  std::vector<Eigen::Vector3f> vv;
		  Eigen::Vector3f v;
		  v << 1,0,1;
		  v=x*v;
		  vv.push_back(v);
		  v << 1,1,1;
		  v=x*v;
		  vv.push_back(v);
		  v << 0,1,1;
		  v=x*v;
		  vv.push_back(v);
		  v << 0,0,1;
		  v=x*v;
		  vv.push_back(v);
		  m_p->polygon_world.push_back(vv);
		  gmv.showPolygon(m_p);


		  xx=x*y2;
		  m_p2->id = 0;
		  m_p2->normal << 0,0,1;
		  m_p2->normal=xx*m_p2->normal;
		  m_p2->d = m_p2->normal(2);
		  std::vector<Eigen::Vector3f> vv2;
		  Eigen::Vector3f v2;
		  v2 << 1,0,1;
		  v2=xx*v2;
		  vv2.push_back(v2);
		  v2 << 1,1,1;
		  v2=xx*v2;
		  vv2.push_back(v2);
		  v2 << 0,1,1;
		  v2=xx*v2;
		  vv2.push_back(v2);
		  v2 << 0,0,1;
		  v2=xx*v2;
		  vv2.push_back(v2);
		  m_p2->polygon_world.push_back(vv2);
		  gmv.showPolygon(m_p2);



		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p2);
		  gm.addMapEntry(m_p);
		  gm.addMapEntry(m_p);	 gm.addMapEntry(m_p);	 gm.addMapEntry(m_p);	 gm.addMapEntry(m_p);	 gm.addMapEntry(m_p);

		  gmv.showMap(gm.getMap());

	}



int main (int argc, char** argv)
{

	GeometryMapTest gmt;
	//gmt.testnormaldirection();
	//gmt.box();
	gmt.fillingHole();
	//gmt.rotate();
	//gmt.strongMerged();
	//gmt.curve();
	//gmt.easyplane();
//	TestPlanes tp;
//	GeometryMapVisualisation gmv;
	//GeometryMap gm;
	//gmt.bugsearch();
//	gmt.threedirection();

/*	Eigen::Vector3f average_normal;
	average_normal <<0.01857,
			-0.999828,
			0.000355565;

	double average_d=-0.829225;
	Eigen::Vector3f ft_pt;
	gm.getPointOnPlane(average_normal,average_d,ft_pt);
	std::cout << "erste " << std::endl << ft_pt;
	average_normal << 0.0185884,
			-0.999827,
			0.000322292;
	average_d=-0.82914;
	gm.getPointOnPlane(average_normal,average_d,ft_pt);
	std::cout << "zweite " << std::endl << ft_pt;

*/

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
