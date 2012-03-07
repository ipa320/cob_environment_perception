/*
 * measure.h
 *
 *  Created on: Nov 21, 2011
 *      Author: goa-jh
 */

#ifndef MEASURE_H_
#define MEASURE_H_

#include "time.h"
#include "memory.h"

/*
 * EXAMPLE
 *

#include <registration/measurements/measure.h>

  measurement_tools::MemoryUsage mu;
  measurement_tools::PrecisionStopWatch psw;
  while(ros::ok()) {
    new int[1024];
  ROS_ERROR("memory %d", mu.getKB());
  ROS_ERROR("time %f", psw.precisionStop());
  sleep(1);
  }

 */

#endif /* MEASURE_H_ */
