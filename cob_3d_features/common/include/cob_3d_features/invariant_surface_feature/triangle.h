/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_features
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Joshua Hampp, email:joshua.hampp@ipa.fraunhofer.de
 *
 * Date of creation: 03/2014
 * ToDo:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#pragma once

#include <complex>
#include <Eigen/Geometry>

namespace cob_3d_features
{
namespace invariant_surface_feature
{
	template<class Scalar, class Samples, class Values>
	class SingleTriangle {
	protected:
		typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
		typedef std::complex<Scalar> ScalarC;

		Vector3 p3_[3];
		Vector3 d1, d2;
		Scalar cr;
		Values f_;
		bool computed_;
		//int CONSTRUCT;
		
	public:
		SingleTriangle(): cr(-1), computed_(false)
		//, CONSTRUCT(11)
		 {}
		
		/*SingleTriangle(const SingleTriangle &o) {
			*this = o;
			CONSTRUCT=33;
		}*/
		
		SingleTriangle(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3): computed_(false)
		//, CONSTRUCT(22)
		{
			p3_[0] = p1;
			p3_[1] = p2;
			p3_[2] = p3;
			d1 = p3_[2]-p3_[1];
			d2 = p3_[1]-p3_[0];
			cr = d1.cross(d2).norm();
		}
		
		void copy(const SingleTriangle &o) {	//keeps f_ uninitialized!
			for(int i=0; i<3; i++)
				p3_[i] = o.p3_[i];
			d1 = o.d1;
			d2 = o.d2;
			cr = o.cr;
			computed_ = false;
		}
		
		inline void init() {
			d1 = p3_[2]-p3_[1];
			d2 = p3_[1]-p3_[0];
			cr = d1.cross(d2).norm();
		}
		
		void compute(const Samples &samples);
		const Values &values() const {assert(computed_); return f_;}
		operator const Values&() const {return values();}
		const Vector3 &operator[](const size_t ind) const {assert(ind<3); return p3_[ind];}

		std::complex<Scalar> kernel_lin_tri(const Vector3 &at, const bool debug=false) const;

		inline Scalar area() const {if(cr<0) print(); /*assert(computed_);*/ assert(cr>=0); return cr/2;}
		inline Scalar getArea() const {return area();}
		void print() const;
	private:
		std::complex<Scalar> kernel_lin(const Scalar m, const Scalar n, const Scalar p, const Scalar x0, const Scalar y0, const Scalar y1, const Scalar d1, const Scalar d2) const;

	};
}
}
