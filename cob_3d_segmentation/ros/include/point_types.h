/*
 * point_types.h
 *
 *  Created on: 14.03.2013
 *      Author: josh
 */

#ifndef POINT_TYPES_H_
#define POINT_TYPES_H_


struct PointXYZRGBLabel
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    union
    {
      struct
      {
        uint8_t b;
        uint8_t g;
        uint8_t r;
        uint8_t _unused;
      };
      float rgb;
    };
    uint32_t rgba;
  };
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBLabel,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (uint32_t, label, label));


struct PointXYZLabel
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZLabel,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, label, label));


struct PointXYZILabel
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  uint32_t intensity;
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZILabel,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, intensity, intensity)
    (uint32_t, label, label));

#endif /* POINT_TYPES_H_ */
