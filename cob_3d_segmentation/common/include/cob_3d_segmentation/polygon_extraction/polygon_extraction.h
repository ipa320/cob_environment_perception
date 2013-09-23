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
 * polygon_extraction.h
 *
 *  Created on: 22.06.2012
 *      Author: josh
 */

#ifndef POLYGON_EXTRACTION_H_
#define POLYGON_EXTRACTION_H_

#include <vector>
#include <stack>
#include <algorithm>

namespace cob_3d_segmentation
{
  /*
   * point has to have integer x,y and must be sortable
   *
   * polygon must have following methods:
   *    -
   */

  class PolygonExtraction
  {
    int *ch_; // array of border points
    size_t ch_size_;
    bool *outline_check_;         ///needed for outline, no need to reallocate every time
    size_t outline_check_size_;    ///remember size for var. above

    template <typename TPoint>
    static int getPos(int *ch, const int xx, const int yy, const int w, const int h);

    inline bool hasMultiplePositions(unsigned int i) { return !((i != 0) && ((i & (~i + 1)) == i)); }

  public:
    PolygonExtraction();

    virtual ~PolygonExtraction()
    {
      delete [] ch_;
      delete [] outline_check_;
    }

    template<typename TPoint, typename TPolygon>
    void outline(const int w, const int h, std::vector<TPoint> out, TPolygon &poly);
    void ppm(const char *fn, const int w, const int h, const int *ch);

  };
}

#endif /* POLYGON_EXTRACTION_H_ */
