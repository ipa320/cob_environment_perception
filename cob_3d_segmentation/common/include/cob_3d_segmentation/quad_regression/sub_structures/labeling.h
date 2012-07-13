/*
 * labeling.h
 *
 *  Created on: 21.06.2012
 *      Author: josh
 */

#ifndef LABELING_H_
#define LABELING_H_

template<typename Point>
void SetLabeledPoint(Point &pt, const int mark)
{
  pt.r=((11<<(mark%3+1))%256);
  pt.g=((3<<(mark%11+1))%256);
  pt.b=((71<<(mark%2+1))%256);
}


//TODO: alternatives

#endif /* LABELING_H_ */
