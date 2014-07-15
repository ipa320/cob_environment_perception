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
 * ROS package name: cob_3d_features
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

#include "cob_3d_features/invariant_surface_feature/triangle.h"

template<class T>
inline std::complex<T> dpi_polar(const T &a, const T &b) {
	return std::polar<T>(a, 2*M_PI*b);
}

template<class Scalar, class Samples, class Values>
std::complex<Scalar> cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar,Samples,Values>::kernel_lin_tri(const Vector3 &at, const bool debug) const {
	static const Scalar BORDER = 0.00001;

	if(cr<BORDER*0.02 || cr!=cr) //quite small area
		return 0;

	if( std::abs(at(0))+std::abs(at(1))+std::abs(at(2))<BORDER)
		return cr/2;

	static const std::complex<Scalar> I(0,2*M_PI);
	//const Scalar n = at(0), m = at(1), p = at(2);	//yea, I messed n and m up...

	const Scalar dot0 = p3_[0].dot(at);
	const Scalar dot1 = d1.dot(at);
	const Scalar dot2 = d2.dot(at);

	if(std::abs(dot1)<BORDER) {
		//-((%e^(%i*dot2)-%i*dot2-1)*%e^(-%i*dot2-%i*dot0))/dot2^2
		//if(debug) std::cout<<"LIMIT1 "<<(-cr*(-I*dot2 + std::exp(I*dot2) - std::complex<Scalar>(1,0))*std::exp(-I*dot0 - I*dot2)/std::pow(dot2, 2))<<std::endl;
		return -cr*(-I*dot2 + std::exp(I*dot2) - std::complex<Scalar>(1,0))*std::exp(-I*dot0 - I*dot2)/std::pow(dot2, 2);
		
		//if(debug) std::cout<<"LIMIT1 "<<(cr*(B-C)/(dot2*dot2))<<std::endl;
		/*if(std::abs(dot1)>0.00001) std::cout<<"HHHHHEEEERE"<<std::endl;
		std::cout<<"look "<<(-(dot1*C-(dot1+dot2)*B+(dot2)*A))<<std::endl;
		std::cout<<"look "<<(val)<<std::endl;
		std::cout<<"look "<<(-(-(dot1+dot2)*B+(dot2)*A))<<std::endl;
		std::cout<<"look "<<(-(dot1*C-(dot1+dot2)*A+(dot2)*A))<<std::endl;
		std::cout<<"look "<<(-(dot1*C-(dot1+dot2)*B+(dot2)*B))<<std::endl;
		std::cout<<"look "<<(-(dot1*C-(dot1+dot2+0.1)*B+(dot2+0.1)*A))<<std::endl;
		return cr*(B-C)/(dot2*dot2);*/
	}
	else if(std::abs(dot2)<BORDER) {
		//-(((%i*dot1-1)*%e^(%i*dot1)+1)*%e^(-%i*dot1-%i*dot0))/dot1^2
		//if(debug) std::cout<<"LIMIT2 "<<(-cr*((I*dot1 - std::complex<Scalar>(1,0))*std::exp(I*dot1) + std::complex<Scalar>(1,0))*std::exp(-I*dot0 - I*dot1)/std::pow(dot1, 2))<<std::endl;
		return -cr*((I*dot1 - std::complex<Scalar>(1,0))*std::exp(I*dot1) + std::complex<Scalar>(1,0))*std::exp(-I*dot0 - I*dot1)/std::pow(dot1, 2);
		
		//if(debug) std::cout<<"LIMIT2 "<<(cr*(C-A)/(dot1*dot1))<<std::endl;
		//return cr*(C-A)/(dot1*dot1);
	}
	else if(std::abs(dot1+dot2)<BORDER) {
		//-(((%i*dot2-1)*%e^(%i*dot2)+1)*%e^(-%i*dot2-%i*dot0))/dot2^2
		//if(debug) std::cout<<"LIMIT3 "<<(-cr*((I*dot2 - std::complex<Scalar>(1,0))*std::exp(I*dot2) + std::complex<Scalar>(1,0))*std::exp(-I*dot0 - I*dot2)/std::pow(dot2, 2))<<std::endl;
		return -cr*((I*dot2 - std::complex<Scalar>(1,0))*std::exp(I*dot2) + std::complex<Scalar>(1,0))*std::exp(-I*dot0 - I*dot2)/std::pow(dot2, 2);
		
		//if(debug) std::cout<<"LIMIT3 "<<(cr*(B-C)/(dot1*dot2))<<std::endl;
		//return cr*(B-C)/(dot1*dot2);
	}
	
//	const Scalar div = (d1(0)*n + d1(1)*m + d1(2)*p)*(d2(0)*n + d2(1)*m + d2(2)*p)*(d1(0)*n + d2(0)*n + d1(1)*m + d2(1)*m + d1(2)*p + d2(2)*p);
	const Scalar div = dot1*dot2*(dot1+dot2);
	
	const std::complex<Scalar> A=dpi_polar<Scalar>(1, -dot0-dot2-dot1);
	const std::complex<Scalar> B=dpi_polar<Scalar>(1, -dot0-dot2);
	const std::complex<Scalar> C=dpi_polar<Scalar>(1, -dot0);
//	const std::complex<Scalar> val = -(d2(0)*n + d2(1)*m + d2(2)*p + (d1(2)*p*std::exp(I*d1(0)*n + I*d2(0)*n + I*d1(1)*m + I*d2(1)*m) + (d1(0)*n*std::exp(I*d1(1)*m + I*d2(1)*m) + d1(1)*m*std::exp(I*d1(1)*m + I*d2(1)*m))*std::exp(I*d1(0)*n + I*d2(0)*n))*std::exp(I*d1(2)*p + I*d2(2)*p) + (p*(-d1(2) - d2(2))*std::exp(I*d1(0)*n + I*d1(1)*m) + (m*(-d1(1) - d2(1))*std::exp(I*d1(1)*m) + n*(-d1(0) - d2(0))*std::exp(I*d1(1)*m))*std::exp(I*d1(0)*n))*std::exp(I*d1(2)*p))*std::exp(-I*d1(0)*n - I*d2(0)*n - I*d1(1)*m - I*d2(1)*m - I*d1(2)*p - I*d2(2)*p - I*m*p3_[0](1) - I*n*p3_[0](0) - I*p*p3_[0](2));
//perhaps try this one
/*-(dz1*p*%e^(%i*dz2*p+%i*dz1*p+%i*dx2*n+%i*dx1*n+%i*dy2*m+%i*dy1*m)+dx1*n*%e^(%i*dz2*p+%i*dz1*p+%i*dx2*n+%i*dx1*n+%i*dy2*m+%i*dy1*m)+dy1*m*
%e^(%i*dz2*p+%i*dz1*p+%i*dx2*n+%i*dx1*n+%i*dy2*m+%i*dy1*m)-dz2*p*%e^(%i*dz1*p+%i*dx1*n+%i*dy1*m)-dz1*p*%e^(%i*dz1*p+%i*dx1*n+%i*dy1*m)-dx2*n*%e^(%i*dz1*p+%i*dx1*n+%i*dy1*m)-dx1*n*
%e^(%i*dz1*p+%i*dx1*n+%i*dy1*m)-dy2*m*%e^(%i*dz1*p+%i*dx1*n+%i*dy1*m)-dy1*m*%e^(%i*dz1*p+%i*dx1*n+%i*dy1*m)+dz2*p+dx2*n+dy2*m)*
%e^(-%i*p*z0-%i*m*y0-%i*n*x0-%i*dz2*p-%i*dz1*p-%i*dx2*n-%i*dx1*n-%i*dy2*m-%i*dy1*m)
*/
	
	/****const std::complex<Scalar> val = -(d2.dot(at) + (
			dpi_polar<Scalar>(d1(2)*p,d1(0)*n + d2(0)*n + d1(1)*m + d2(1)*m) + 
			( dpi_polar<Scalar>(d1(0)*n, d1(1)*m + d2(1)*m) + dpi_polar<Scalar>(d1(1)*m, d1(1)*m + d2(1)*m))*
			dpi_polar<Scalar>(1,d1(0)*n + d2(0)*n))*
			dpi_polar<Scalar>(1,d1(2)*p + d2(2)*p) + (dpi_polar<Scalar>(p*(-d1(2) - d2(2)), d1(0)*n + d1(1)*m) + (dpi_polar<Scalar>(m*(-d1(1) - d2(1)), d1(1)*m) + dpi_polar<Scalar>(n*(-d1(0) - d2(0)), d1(1)*m))*
			dpi_polar<Scalar>(1,d1(0)*n))*dpi_polar<Scalar>(1,d1(2)*p))
			*dpi_polar<Scalar>(1, -(d1.dot(at)+d2.dot(at)+p3_[0].dot(at)) );***/
	const std::complex<Scalar> val = -(dot1*C-(dot1+dot2)*B+dot2*A);
	
	/*
const std::complex<Scalar> val = (polar_simp(d1(0)*n, d1(0)*n + d1(1)*m + d1(2)*p) - polar_simp(d1(0)*n, d1(0)*n + d2(0)*n + d1(1)*m + d2(1)*m + d1(2)*p + d2(2)*p) + 
polar_simp(d2(0)*n, d1(0)*n + d1(1)*m + d1(2)*p) - d2(0)*n + polar_simp(d1(1)*m, d1(0)*n + d1(1)*m + d1(2)*p) - polar_simp(d1(1)*m, d1(0)*n + d2(0)*n + d1(1)*m + d2(1)*m + d1(2)*p + d2(2)*p) + 
polar_simp(d2(1)*m, d1(0)*n + d1(1)*m + d1(2)*p) - d2(1)*m + polar_simp(d1(2)*p, d1(0)*n + d1(1)*m + d1(2)*p)
 - polar_simp(d1(2)*p, d1(0)*n + d2(0)*n + d1(1)*m + d2(1)*m + d1(2)*p + d2(2)*p) + polar_simp(d2(2)*p, d1(0)*n + d1(1)*m + d1(2)*p) - d2(2)*p)
 *polar_simp(1, -(d1.dot(at)+d2.dot(at)+p3_[0].dot(at)) );
 */

	/*if(std::abs(div)<BORDER) {
	//if(std::abs(cr*val/div)>10) */
	
#if 0
	const std::complex<Scalar> r = cr*val/div;
	if(debug||r!=r) {
		std::cout<<std::endl;
		std::cout<<d1.dot(at)<<" "<<d2.dot(at)<<" "<<d2.cross(d1).dot(at)<<std::endl;
		std::cout<<d1.transpose()<<" | "<<d1.norm()<<std::endl;
		std::cout<<d2.transpose()<<" | "<<d2.norm()<<std::endl;
		for(int i=0; i<3; i++) std::cout<<p3_[i].transpose()<<" | "<<p3_[i].norm()<<std::endl;
		std::cout<<"mnp "<<at.transpose()<<" with area "<<cr/2<<std::endl;
		std::cout<<cr<<" * "<<val<<" / "<<div<<std::endl;
		std::cout<<" = "<<cr*val/div<<std::endl;
		std::cout<<cr<<" * "<<std::abs(val)<<" / "<<div<<std::endl;
		std::cout<<" = "<<std::abs(cr*val/div)<<std::endl;
		
		//std::cout<<"beside "<<kernel_lin_tri(at+at.Random()*0.01)<<std::endl;
		
		std::cout<<"dot0 "<<dot0<<std::endl;
		std::cout<<"dot1 "<<dot1<<std::endl;
		std::cout<<"dot2 "<<dot2<<std::endl;
		
		std::cout<<"A "<<A<<std::endl;
		std::cout<<"B "<<B<<std::endl;
		std::cout<<"C "<<C<<std::endl;
		
		print();
		
		//assert(r==r);
	}
#endif

	return cr*val/div;
}

template<class Scalar, class Samples, class Values>
void cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar,Samples,Values>::compute(const Samples &samples) {
	++g_num_computations;
	
	if(cr<0) {	//if not already set
		d1 = p3_[2]-p3_[1];
		d2 = p3_[1]-p3_[0];
		cr = d1.cross(d2).norm();
	}
	/*std::cout<<"CONSTRUCT "<<CONSTRUCT<<std::endl;
	print();
	assert(cr == d1.cross(d2).norm());
	assert((p3_[2]-p3_[1]) == d1);
	assert((p3_[1]-p3_[0]) == d2);
	
	d1*=3;
	d2*=3;
	cr*=3;
	p3_[0]*=3;*/
	
	const Scalar weight = std::abs(Vector3::UnitZ().dot(d1.cross(d2))/cr)*0.8+0.2;
	
	f_.resize(samples.size());
	for(size_t r=0; r<samples.size(); r++) {
		f_[r].resize(samples[r].size());
		for(size_t i=0; i<samples[r].size(); i++) {
			f_[r][i] = weight*kernel_lin_tri(samples[r][i]);
			/*if(!std::isfinite(std::abs(f_[r][i]))||std::abs(f_[r][i])>20) {
				kernel_lin_tri(samples[r][i], true);
				print();
				
				assert(0);
				f_[r][i]=0;	//keep data valid, it's soo bad
			}*/
		}
	}
	
	/*d1/=3;
	d2/=3;
	cr/=3;
	p3_[0]/=3;*/

	computed_ = true;
}

template<class Scalar, class Samples, class Values>
void cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar,Samples,Values>::print() const {
	std::cout<<"Triangle: "<<cr<<" "<<computed_<<"\n\t"<<p3_[0].transpose()<<"\n\t"<<p3_[1].transpose()<<"\n\t"<<p3_[2].transpose()<<std::endl;;
	
	std::cout<<cr <<" == "<< d1.cross(d2).norm()<<std::endl;
	std::cout<<d1.transpose()<<"    "<<d2.transpose()<<std::endl;
}
