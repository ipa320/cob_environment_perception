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
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_mapping_common
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#ifndef COB_3D_MAPPING_COMMON_IO_H_
#define COB_3D_MAPPING_COMMON_IO_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <stdint.h>
#include <float.h>
#include <iomanip>

namespace cob_3d_mapping_common
{
  /*! @brief 2D image PPM file format reader.
   */
  class PPMReader
  {
  public:
    /*! Empty constructor */
    PPMReader() { }
    /*! Empty destructor */
    ~PPMReader() { }

    /*! @brief Reads a 2D image from .ppm ASCII format and maps the RGB values on a PointCloud.
     *
     * @param[in] file_name the input path of the .ppm ASCII file
     * @param[out] cloud the point cloud with RGB fields to which the .ppm is mapped
     * @return < 0 (-1) on error, else 0
     */
    int mapLabels (const std::string &file_name, pcl::PointCloud<pcl::PointXYZRGB>& cloud, bool remove_undef_points=false);
    int mapRGB(const std::string &file_name, pcl::PointCloud<pcl::PointXYZRGB>& cloud, bool remove_undef_points=false);
  };

  /*! @brief 2D image PPM file format writer.
   */
  class PPMWriter
  {
  public:
    /*! Empty constructor */
    PPMWriter()
      : fixed_max_(false), fixed_min_(false), max_z_(FLT_MIN), min_z_(FLT_MAX)
    { }
    /*! Empty destructor */
    ~PPMWriter() { }

    /*! @brief Writes the color information of a point cloud to an ASCII formated .ppm 2D image.
     *
     * @param[in] file_name the output .ppm filename
     * @param[in] cloud the input organized point cloud providing RGB fields
     * @return -1 for error, else 0
     */
    int writeRGB (const std::string &file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud);

    /*! @brief Writes the depth information of a point cloud to an ASCII formated .ppm
     *         2D color image
     *
     * The Z values of the point cloud are encoded as a color gradient with the minimum value defined
     * by @a setMinZ as @a RGB(255,0,0) and the maximum value defined by @a setMaxZ as @a RGB(0,0,255).
     * If no values are defined, the minimum and maximum values of the point cloud are determined
     * automatically.
     *
     * @param[in] file_name the output .ppm filename
     * @param[in] cloud the input organized point cloud
     * @return < 0 (-1) on error, else 0
     */
    int writeDepth (const std::string &file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud);

    int writeDepthLinear (const std::string &file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud);

    /*! @brief Sets the maximum Z value for the color gradient
     *
     * @param[in] max the maximum Z value
     */
    void setMaxZ (const float &max);
    /*! @brief Sets the minimum Z value for the color gradient
     *
     * @param[in] min the minimum Z value
     */
    void setMinZ (const float &min);

    /*! @brief Flag whether a max value was defined manually */
    bool fixed_max_;
    /*! @brief Flag whether a min value was defined manually */
    bool fixed_min_;
    /*! @brief Holds the maximum Z value */
    float max_z_;
    /*! @brief Holds the minimum Z value */
    float min_z_;
  };

  /*! @brief Converts a normalized Z value into an RGB color value
   *
   * @param[in] position the normalized Z value to be represented as color
   *     (0.0 for red, 1 for blue)
   * @param[out] the output rgb color value separated
   *      - @a rgb[0]: red (0..255)
   *      - @a rgb[1]: green (0..255)
   *      - @a rgb[2]: blue (0..255)
   * @return the rgb color value as combined integer
   */
  uint32_t getGradientColor(double position, uint8_t rgb[]);

  /**
   * \brief Convert a color value to human readable format.
   *
   * \param[in] id The color value to be converted.
   *
   * \return The human readable string.
   */
  std::string colorHumanReadable(int id)
  {
    std::stringstream ss;
    ss << "0x" << std::setfill('0') << std::setw(6) << std::right << std::hex << id << std::dec;
    return ss.str();
  }
}

#endif // #ifndef COB_3D_MAPPING_COMMON_IO_H_
