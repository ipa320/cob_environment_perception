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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: registration
 *
 * \author
 *  Author: goa-jh
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: Nov 11, 2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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


//##################
//#### includes ####
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


template<typename PointT>
void
preprocessing::KinectErrorGenerator<PointT>::applyFilter (PointCloud &pc_out)
{
  //normal distribution
  boost::mt19937 rng;
  rng.seed(clock());

  // set the parameters for output poincloud (pc_out)
  if(&*input_ == &pc_out)
    input_.reset(new PointCloud(pc_out));
  else
    pc_out = *input_;

  //Go through all points and discard points with amplitude outside filter limits
  for(int x=0; x<(int)input_->width; x++) {
    for(int y=0; y<(int)input_->height; y++) {

      //step 1: deviation

      float diff=0;
      int num=0;
      for(int dx=-2; dx<2; dx++) {
        for(int dy=-2; dy<2; dy++) {
          if(dx+x<0||dx+x>=(int)input_->width
              || dy+y<0||dy+y>=(int)input_->height)
            continue;

          diff += input_->points[x+y*input_->width].z -
              input_->points[x+dx+(y+dy)*input_->width].z;
          ++num;
        }
      }
      diff/=num;

      float mean=diff;

      PointT &p = pc_out.points[x+y*input_->width];

      diff = std::max(0.005f, std::abs(diff));

      diff = 0.01;
      mean = 0;

      diff *= p.z*p.z;

      diff = std::min(diff, 2.f);

      boost::normal_distribution<> nd(0, diff*standard_deviation_);

      boost::variate_generator<boost::mt19937&,
      boost::normal_distribution<> > var_nor(rng, nd);

      float add=0;
      do {
        add = var_nor();
      } while(std::abs(add)>std::max(p.z*p.z*0.005f,std::abs(mean)));

      p.x /= p.z;
      p.y /= p.z;

      p.z += add-mean;

      //step 2: quantization error
      int raw_depth = ((1./p.z)-3.3309495161)/-0.0030711016;

      p.z = 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);

      p.x *= p.z;
      p.y *= p.z;

    }
  }

}


#define PCL_INSTANTIATE_KinectErrorGenerator(T) template class preprocessing::KinectErrorGenerator<T>;
