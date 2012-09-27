/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
* \author
* Supervised by: Richard Bormann, email:richard.bormann@ipa.fhg.de
*
* \date Date of creation: 08/2010
*
* \brief
* Precise Stopwatch
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#ifndef STOPWATCH_H_
#define STOPWATCH_H_


#include <ctime>
#include <sstream>
#ifdef _WIN32
#include <Windows.h>
#endif


// low resolution stop watch
class SimpleStopWatch {
	clock_t startClock;

public:
	// start time measurement at this moment
	void start() { startClock = clock(); };

	// returns elapsed time since last start() in seconds
	double stop() { return ((double)(clock()-startClock))/(double)CLOCKS_PER_SEC; };
};


// high performance stop watch
#ifdef _WIN32
class PrecisionStopWatch {
	LARGE_INTEGER precisionClock;
	LARGE_INTEGER precisionFrequency;

public:
	PrecisionStopWatch() {
		precisionClock.QuadPart = 0;
		if(!QueryPerformanceFrequency(&precisionFrequency)) printf("PrecisionStopWatch: Error: Frequency query failed.\n");
	};

	// start time measurement at this moment
	void precisionStart() {
		if(!QueryPerformanceCounter(&precisionClock)) printf("PrecisionStopWatch: Error: precisionStart query failed.\n");
	};

	// returns elapsed time since last precisionStart() in seconds
	double precisionStop() {
		LARGE_INTEGER precisionClockEnd;
		if(!QueryPerformanceCounter(&precisionClockEnd)) printf("PrecisionStopWatch: Error: precisionStop query failed.\n");
		return (double)(precisionClockEnd.QuadPart - precisionClock.QuadPart) / (double)precisionFrequency.QuadPart;
	};
};
#else
// Linux version of precision host timer. See http://www.informit.com/articles/article.aspx?p=23618&seqNum=8
class PrecisionStopWatch {
	struct timeval precisionClock;

public:
	PrecisionStopWatch() {
		gettimeofday(&precisionClock, NULL);
	};

	void precisionStart() {
		gettimeofday(&precisionClock, NULL);
	};

	double precisionStop() {
		struct timeval precisionClockEnd;
		gettimeofday(&precisionClockEnd, NULL);
		return ((double)precisionClockEnd.tv_sec + 1.0e-6 * (double)precisionClockEnd.tv_usec) - ((double)precisionClock.tv_sec + 1.0e-6 * (double)precisionClock.tv_usec);
	};
};
#endif

#endif //STOPWATCH_H_
