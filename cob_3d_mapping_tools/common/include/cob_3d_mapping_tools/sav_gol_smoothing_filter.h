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
#ifndef SAV_GOL_SMOOTHING_FILTER_H_
#define SAV_GOL_SMOOTHING_FILTER_H_

#include <pcl/point_types.h>
#include <Eigen/LU>

template <typename PointInT, typename PointOutT>
class SavGolSmoothingFilter
{
  public:

    typedef pcl::PointCloud<PointInT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef pcl::PointCloud<PointOutT> PointCloudOut;

  public:
    SavGolSmoothingFilter () : high_th_(5.0)
    { };

    inline void
      setInputCloud(const PointCloudInConstPtr &cloud)
    {
      input_ = cloud;
    }

    inline void
      setFilterCoefficients(int size, int order)
    {
      int n_coef = 0; //2*order+1; // number of coefficients
      for (int j=0;j<=order;j++) for (int i=0;i<=(order-j);i++) n_coef++;
      double sum, fac;

      Eigen::MatrixXd A2, C, A = Eigen::MatrixXd::Zero(size*size,n_coef);
      std::vector<double> coef;
      size_ = size;
      r_ = (size-1)/2;

      int d = 0;
      for (int y = -(size-1)/2; y <= (size-1)/2; y++)
      {
	for (int x = -(size-1)/2; x <= (size-1)/2; x++)
	{
	  int a = 0;
	  for (int j = 0; j <= order; j++)
	  {
	    for (int i = 0; i <= (order - j); i++)
	    {
	      A(d,a) = pow(x,i)*pow(y,j);
	      a++;
	    }
	  }
	  d++;
	}
      }

      A2 = A.transpose() * A;
      C = A2.inverse() * A.transpose();

      int mm;
      for(mm=0;mm<size*size;mm++) 
      {
	//std::cout << C(0,mm) << std::endl;
	coef.push_back(C(0,mm));
      }
      coef_ = coef;
      return;
    }

    inline void
      getFilterCoefficients(std::vector<double> &coef)
    {
      coef = coef_;
    }

    inline void
      setDistanceThreshold(float threshold)
    {
      high_th_ = threshold;
    }

    bool
      smoothPoint(int index, float &z);

    void
      reconstruct(PointCloudOut &output, std::vector<size_t> &ignored_indices);

    void
      reconstruct2ndPass(std::vector<size_t> &indices, 
			 PointCloudOut &output);
      

  private:

    std::vector<double> coef_;
    int size_;
    int r_;
    PointCloudInConstPtr input_;
    float high_th_;
  
};

#endif // SAV_GOL_SMOOTHING_FILTER_H_
