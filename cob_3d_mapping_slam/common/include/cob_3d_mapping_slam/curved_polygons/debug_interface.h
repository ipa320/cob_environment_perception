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
 * debug_interface.h
 *
 *  Created on: 12.06.2012
 *      Author: josh
 */

#ifndef DEBUG_INTERFACE_H_
#define DEBUG_INTERFACE_H_

#ifdef DEBUG_

namespace Debug
{
  class Interface
  {
    struct ARROW {
      Eigen::Vector3f from, to;
      int r,g,b;
    };

    std::vector<ARROW> arrows_;
    float time_;
  public:
    void addArrow(const Eigen::Vector3f &from, const Eigen::Vector3f &to, int r=255, int g=255, int b=255)
    {
      ARROW A;A.from=from;A.to=to;
      A.r=r;A.g=g;A.b=b;
      arrows_.push_back(A);
    }

    bool getArrow(Eigen::Vector3f &from, Eigen::Vector3f &to, unsigned char &r, unsigned char &g, unsigned char &b)
    {
      if(arrows_.empty())
        return false;
      from = arrows_.back().from;
      to = arrows_.back().to;
      r = arrows_.back().r;
      g = arrows_.back().g;
      b = arrows_.back().b;
      arrows_.pop_back();
      return true;
    }

    void clear()
    {
      arrows_.clear();
    }

    void setTime(const float t) {time_ = t;}
    float getTime() const {return time_;}

    static Interface &get() {
      static Interface intf;
      return intf;
    }
  };
}
#endif

#endif /* DEBUG_INTERFACE_H_ */
