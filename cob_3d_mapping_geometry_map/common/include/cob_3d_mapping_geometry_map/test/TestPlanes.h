/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
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

	void
	moveplane(MapEntryPtr m_p , MapEntryPtr m_p2 , double x_1 , double x_2 , double x_3);

	void
	moveX1X2(MapEntryPtr m_p , double x_1 , double x_2 , double x_3);

	void
	figure_1(MapEntryPtr m_p , MapEntryPtr m_p2 ,MapEntryPtr m_p3 , MapEntryPtr m_p4 ,MapEntryPtr m_p5 );

	void
	rotate(MapEntryPtr m_p , double x_1 , double x_2 , double alpha );

	void
	rotateTable(MapEntryPtr m_p ,  double alpha );

};

#endif /* TESTPLANES_H_ */
