/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 08/2010
*
* \brief
* Definition of PCL point types
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#ifndef IPA_POINT_TYPES_HPP_
#define IPA_POINT_TYPES_HPP_

//using namespace pcl;

//Point obtained from Swissranger
struct PointXYZCI
{
	PCL_ADD_POINT4D;
	union
	{
		struct
		{
			float confidence;
			float intensity;
		};
		float data_c[4];
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


//Point obtained from Camboard
struct PointXYZA
{
	PCL_ADD_POINT4D;
	union
	{
		struct
		{
			float amplitude;
		};
		float data_c[4];
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZRGBF
{
	PCL_ADD_POINT4D;
	union
	{
		struct
		{
			float rgb;
			uint8_t isF;
		};
		float data_c[4];
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointLabel
{
	int label;
};

template<int N>
struct PointXYZFeature
{
	enum {DIMENSION=N};
		
	PCL_ADD_POINT4D;
	float area;
	float feature[N];
	
	operator pcl::PointXYZ() const {
		return pcl::PointXYZ(x,y,z);
	}
} EIGEN_ALIGN16;



#endif /* IPA_POINT_TYPES_HPP_ */
