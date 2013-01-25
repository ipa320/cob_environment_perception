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
 * time.h
 *
 *  Created on: Nov 21, 2011
 *      Author: now me Joshua Hampp
 */

#ifndef TIME_H_
#define TIME_H_

namespace measurement_tools {

  // Linux version of precision host timer. See http://www.informit.com/articles/article.aspx?p=23618&seqNum=8
  /**
   * stop watch to measure duration
   */
  class PrecisionStopWatchAll {
    struct timeval precisionClock;

  public:
    PrecisionStopWatchAll() {
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

  /**
   * stop watch to measure time needed by this thread
   */
  class PrecisionStopWatchThread {
    timespec precisionClock;

    timespec diff(timespec start, timespec end)
    {
      timespec temp;
      if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
      } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
      }
      return temp;
    }

  public:
    PrecisionStopWatchThread() {
      precisionStart();
    };

    void precisionStart() {
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &precisionClock);
    };

    double precisionStop() {
      timespec t;
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
      t=diff(precisionClock,t);
      precisionStart();
      return t.tv_sec+t.tv_nsec/1000000000.;
    };
  };
}

#endif /* TIME_H_ */
