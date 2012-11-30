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
 * hypothesis.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef HYPOTHESIS_H_
#define HYPOTHESIS_H_


namespace Slam  /// namespace for all Slam related stuff
{

  template<typename NODE>
  class Hypothesis
  {
    typedef typename NODE::OBJCTXT OBJCTXT;
    typedef typename OBJCTXT::DOF6 DOF6;

    struct SHYPO
    {
      typename DOF6::TYPE error_rate_, success_rate_;
      std::vector<SWAY<NODE> > cons_;
    };

    typedef std::map<typename NODE::Ptr,SHYPO> HIST;

    HIST history_;
  public:

    typename DOF6::TYPE getOverallScore() const {
      typename DOF6::TYPE mx=0;

      for(typename HIST::const_iterator it=history_.begin(); it!=history_.end(); ++it)
      {
        mx = std::max(mx,it->error_rate_);
      }

      return mx;
    }
  };

}


#endif /* HYPOTHESIS_H_ */
