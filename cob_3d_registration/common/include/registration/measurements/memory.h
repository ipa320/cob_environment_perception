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
 * memory.h
 *
 *  Created on: Nov 21, 2011
 *      Author: goa-jh
 */

#ifndef MEMORY_H_
#define MEMORY_H_

#include <sys/resource.h>

namespace measurement_tools {

  namespace hidden {
    int parseLine(char* line){
      int i = strlen(line);
      while (*line < '0' || *line > '9') line++;
      line[i-3] = '\0';
      i = atoi(line);
      return i;
    }


    int getValue(FILE *file){ //Note: this value is in KB!
      int resultA = -1;
      int resultB = -1;
      char line[128];


      while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmSize:", 7) == 0) resultA = parseLine(line);
        else if (strncmp(line, "VmRSS:", 6) == 0) resultB = parseLine(line);
      }
      if(resultA<0||resultB<0)
        return -1;

      return resultA+resultB;
    }

  }

  /// in kB
  int getMemoryUsage(int pid=getpid())
  {
    char buffer[256];
    sprintf(buffer,"/proc/%d/status", pid);
    FILE* status = fopen( buffer, "r" );
    int used = hidden::getValue(status);
    fclose(status);

    return used;
  }

  /**
   * memory usage gets the ammount of memory in kB from process with pid
   * it only returns the difference between start and request
   */
  class MemoryUsage {
    int pid,start;
  public:
    MemoryUsage(int pid=getpid()):
      pid(pid),
      start(getMemoryUsage(pid))
    {}

    int getKB() {
      return getMemoryUsage(pid)-start;
    }
  };
}


#endif /* MEMORY_H_ */
