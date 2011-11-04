/*
 * TestPlanes.h
 *
 *  Created on: 02.11.2011
 *      Author: goa-hh
 */

#ifndef TESTPLANES_H_
#define TESTPLANES_H_

#include <vector>
#include <Eigen/Geometry>


#include "cob_3d_mapping_geometry_map/map_entry.h"


class TestPlanes {
public:
	TestPlanes();
	~TestPlanes();

	void
	plane_1(MapEntryPtr m_p);

	void
	plane_2(MapEntryPtr m_p);

	void
	overlap(MapEntryPtr m_p , MapEntryPtr m_p2 , double x);
};

#endif /* TESTPLANES_H_ */
