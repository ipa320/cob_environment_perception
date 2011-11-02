/*
 * TestPlanes.h
 *
 *  Created on: 02.11.2011
 *      Author: goa-hh
 */

#ifndef TESTPLANES_H_
#define TESTPLANES_H_

class TestPlanes {
public:
	TestPlanes();
	virtual ~TestPlanes();

	void
	plane_1(GeometryMap::MapEntry& m_p);

	void
	plane_2(GeometryMap::MapEntry& m_p);
};

#endif /* TESTPLANES_H_ */
