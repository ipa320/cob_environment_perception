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
