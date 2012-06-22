/*
 * polygon_extraction.h
 *
 *  Created on: 22.06.2012
 *      Author: josh
 */

#ifndef POLYGON_EXTRACTION_H_
#define POLYGON_EXTRACTION_H_

#include <vector>
#include <algorithm>
#include "impl/poly2d.hpp"

namespace Segmentation
{
  /*
   * point has to have integer x,y and must be sortable
   *
   * polygon must have following methods:
   *    -
   */

  class PolygonExtraction
  {
    int *ch_; /// mark-array
    size_t ch_size_;
    bool *outline_check_;         ///needed for outline, no need to reallocate every time
    size_t outline_check_size_;    ///remember size for var. above

    template <typename TPoint>
    static int getPos(int *ch, const int xx, const int yy, const int w, const int h);

  public:

    PolygonExtraction():ch_(NULL),ch_size_(0),outline_check_(NULL),outline_check_size_(0)
    {
    }

    virtual ~PolygonExtraction()
    {
      delete [] ch_;
      delete [] outline_check_;
    }

    template<typename TPoint, typename TPolygon>
    void outline(const int w, const int h, std::vector<TPoint> out, TPolygon &poly);

  };

#include "impl/polygon_extraction.hpp"

}


#endif /* POLYGON_EXTRACTION_H_ */
