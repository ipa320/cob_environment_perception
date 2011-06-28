/*
 * cpc_point.h
 *
 *  Created on: 26.08.2010
 *      Author: goa
 */

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



#endif /* IPA_POINT_TYPES_HPP_ */
