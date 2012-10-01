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
