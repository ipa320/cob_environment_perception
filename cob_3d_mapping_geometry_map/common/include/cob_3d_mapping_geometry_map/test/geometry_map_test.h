/*
 * TestPlanes.h
 *
 *  Created on: 02.11.2011
 *      Author: goa-hh
 */

#ifndef GeometryMapTest_H_
#define GeometryMapTest_H_

#include <vector>
#include <Eigen/Geometry>
#include <math.h>


#include "cob_3d_mapping_geometry_map/map_entry.h"


class GeometryMapTest {
public:
	GeometryMapTest();
	~GeometryMapTest();

	void
	fillingHole();

	void
	box();

	void
	rotate();

	void
	curve();

	void
	easyplane();



	void
	strongMerged();

	void
	testnormaldirection();

	void
	bugsearch();

	void
	threedirection();





private:
	TestPlanes tp;
	GeometryMapVisualisation gmv;
	GeometryMap gm;
};

#endif /* GeometryMapTest_H_ */
