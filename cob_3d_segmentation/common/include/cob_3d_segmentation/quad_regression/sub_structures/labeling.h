/*
 * labeling.h
 *
 *  Created on: 21.06.2012
 *      Author: josh
 */

#ifndef LABELING_H_
#define LABELING_H_

// convert mark to "some" color
template<typename Point>
void SetLabeledPoint(Point &pt, const int mark)
{
  srand(mark);
  int color = rand();
  pt.r= (color>>0)&0xff;
  pt.g= (color>>8)&0xff;
  pt.b=(color>>16)&0xff;
}

template<>
void SetLabeledPoint<PointXYZLabel>(PointXYZLabel &pt, const int mark)
{
}

template<>
void SetLabeledPoint<PointXYZILabel>(PointXYZILabel &pt, const int mark)
{
}


//TODO: alternatives

#endif /* LABELING_H_ */
