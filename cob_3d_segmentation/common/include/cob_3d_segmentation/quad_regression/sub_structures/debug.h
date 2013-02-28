/*
 * debug.h
 *
 *  Created on: 16.07.2012
 *      Author: josh
 */

#ifndef QPPF_DEBUG_H_
#define QPPF_DEBUG_H_


namespace QQPF_Debug
{

  /*
   * debug function to output occopy map to ppm file
   */
  void ppm(const char *fn, const int w, const int h, const int *ch) {
    //ppm
    FILE *fp = fopen(fn,"w");

    char buffer[128];
    sprintf(buffer,"P3\n%d %d\n255\n",h,w);
    fputs(buffer,fp);

    for(int x=0; x<w; x++) {
      for(int y=0; y<h; y++) {
        int v=0;
        if(ch[(x+w*y)])
          v=255;

        if(ch[(x+w*y)]<-1) {
          sprintf(buffer,"%d %d %d  ",0,255,0);
        }
        else if(ch[(x+w*y)]<0) {
          sprintf(buffer,"%d %d %d  ",255,0,0);
        }
        else
          sprintf(buffer,"%d %d %d  ",v,v,v);
        fputs(buffer,fp);
      }
    }
    fclose(fp);
  }

}

#endif /* DEBUG_H_ */
