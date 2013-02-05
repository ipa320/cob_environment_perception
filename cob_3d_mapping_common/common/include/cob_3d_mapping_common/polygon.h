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
* \date Date of creation: 03/2012
*
* \brief
* Class representing polygon shapes
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

#ifndef POLYGON_H_
#define POLYGON_H_

#include "cob_3d_mapping_common/shape.h"
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
extern "C" {
#include "cob_3d_mapping_common/gpc.h"
}
#include <fstream>
#include <iostream>

//#include "cob_3d_mapping_common/stop_watch.h"

namespace cob_3d_mapping
{
  template<typename T>
    inline T min3(const T& a, const T& b, const T& c) { return ( a < b ? std::min<T>(a,c) : std::min<T>(b,c) ); }
  template<typename T>
    inline T max3(const T& a, const T& b, const T& c) { return ( a > b ? std::max<T>(a,c) : std::max<T>(b,c) ); }

  class DominantColor
  {
    private:
    enum { HIST_SIZE = 180 }; // uint8_t limits hue to 0..255

    public:
    DominantColor() : sum_colors_(0), sum_sat_(0), sum_val_(0), hue_histogram_(HIST_SIZE,0), sat_values_(HIST_SIZE,0)
    { }

    ~DominantColor() { }

    void addColor(uint8_t r, uint8_t g, uint8_t b)
    {
      //std::cout << "rgb_in" << (int)r << "," << (int)g << "," << (int)b << std::endl;
      int h,s,v;
      rgb2hsv(r,g,b,h,s,v);
      //std::cout << "hsv_in" << (int)h << "," << (int)s << "," << (int)v << std::endl;
      int pos = incrBin(h);
      sat_values_[pos] += s;
      sum_sat_ += s;
      sum_val_ += v;
      ++sum_colors_;
    }

    inline void getColor(uint8_t& r, uint8_t& g, uint8_t& b) const
    {
      if(!sum_colors_) { r=0; g=0; b=0; return; }
      int pos = getMaxBin();
      //std::cout << "hsv_out" << round(pos*bin_size+bin_center) << "," << sat_values_[pos]/hue_histogram_[pos] << "," << sum_val_/sum_colors_ << std::endl;
      hsv2rgb( round(pos*bin_size+bin_center), sat_values_[pos]/hue_histogram_[pos], sum_val_/sum_colors_, r,g,b );
      //std::cout << "rgb_out" << (int)r << "," << (int)g << "," << (int)b << std::endl;
    }

    inline int incrBin(int h)
    { int bin = h*inv_bin_size; hue_histogram_[bin] += 1; return bin; }

    inline int getMaxBin() const
    {
      int max_bin=0, max_size=0;
      for(int i=0;i<HIST_SIZE;++i)
      {
        if (hue_histogram_[i] >= max_size)
        {
          max_size = hue_histogram_[i];
          max_bin = i;
        }
      }
      //std::cout<<"max_(bin/size): "<<max_bin<<"/"<<max_size<<std::endl;
      return max_bin;
    }

    inline void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, int& h, int& s, int& v) const
    {
      int rgb_min = min3(r,g,b);
      int rgb_max = max3(r,g,b);
      int delta = rgb_max - rgb_min;
      v = rgb_max;
      if (v == 0) { s = h = 0; return; }
      s = round(float(delta) / float(v) * 100.0f);
      v /= 2.55f;
      float h_tmp;
      if (s == 0) { h = 0; return; }
      if      ((int)r == rgb_max)   h_tmp =     (float(g - b)) / (float(delta));
      else if ((int)g == rgb_max)   h_tmp = 2.0f + (float(b - r)) / (float(delta));
      else                          h_tmp = 4.0f + (float(r - g)) / (float(delta));

      h = h_tmp * 60.0f;
      if(h<0) h+=360;
    }

    inline void hsv2rgb(int h, int s, int v, uint8_t& r, uint8_t& g, uint8_t& b) const
    {
      if (s == 0) { r = g = b = v*2.55f; return; }

      float hh = h / 60.0f; // sector 0..5
      int i = floor(hh);
      float f = hh - i;
      v = round(v*2.55f);
      int p = v * (100 - s) * 0.01f;
      int q = v * (100 - s * f) * 0.01f;
      int t = v * (100 - s * (1.0f - f)) * 0.01f;

      switch(i)
      {
      case 0:
      { r = v; g = t; b = p; break; }
      case 1:
      { r = q; g = v; b = p; break; }
      case 2:
      { r = p; g = v; b = t; break; }
      case 3:
      { r = p; g = q; b = v; break; }
      case 4:
      { r = t; g = p; b = v; break; }
      default:// case 5:
      { r = v; g = p; b = q; break; }
      }
    }

    private:
    int sum_colors_;
    int sum_sat_;
    int sum_val_;
    std::vector<int> hue_histogram_;
    std::vector<int> sat_values_;

    static const float inv_bin_size = 1.0f / 360.0f * HIST_SIZE;
    static const float bin_size = 360.0f / HIST_SIZE;
    static const float bin_center = ((360.0f / HIST_SIZE) - 1.0f) * 0.5f;
  };


  /**
  * \brief Struct with merge configuration
  * \note Members of struct determine merge thresholds
  * and the weighting method , which is applied.
  */
  struct merge_config
  {
    /**
    * \brief Angular threshold.
    * \note Used, when an angular kind of threshold is needed.
    * For example while comparing normals of shapes.
    */
    double angle_thresh;
    /**
    * \brief Distance threshold.
    * \note Used, when a metrical threshold is needed.
    * For Example, while comparing positions of shapes.
    */
    float  d_thresh;
    /**
    * \brief Weighting method.
    * \note Method, that is used to determine the merge weight
    * of a shape.
    * \note Implemented methods are: "COUNTER","AREA"
    */
    std::string weighting_method;
  };


  /**
  * \brief Class representing Polygon shapes.
  * \note Comparison and Merging of Polygons is handled.
  */
  class Polygon : public Shape
  {
  public:
  /** \brief Polygon pointer.
  * \details Boost shared pointer to polygon.
  */
    typedef boost::shared_ptr<Polygon> Ptr;
    /**
    *  \brief Const. Polygon pointer.
    * \details  Const boost shared pointer to polygon.
    */
    typedef boost::shared_ptr<const Polygon> ConstPtr;
  public:

  /**
  * \brief Constructor for Polygon object.
  */
    Polygon()
      : normal(Eigen::Vector3f::Zero())
      , d(0.0)
      , transform_from_world_to_plane(Eigen::Affine3f::Identity())
      , merge_weight_(1.0)
    { }


    /**
    * \brief Get index of first non-hole conour.
    * \return Index of first non-hole contour.
    */
    inline size_t outerContourIndex() const
    {
      for(size_t i=0;i<contours.size();++i) { if(!holes[i]) { return i; } };
      return 0;
    }

    //##########methods for instantiation##############

    /**
    * \brief Compute attributes for Polygon.
    *
    * Set of attributes is completed with respect to the input paramers.
    * \param[in] new_normal Normal of Polygon.
    * \param[in] new_centroid Centroid of Polygon.
    */
    virtual void computeAttributes(const Eigen::Vector3f &new_normal, const Eigen::Vector4f & new_centroid);


    /**
    * \brief Transform polygon to target coordinate system.
    *
    * Polygon is transformed with input transformation.
    * Operation is applied to both parameters and contour points.
    * \param[in] trafo Affine transformation to target coordinate system
    */
    virtual void transform2tf(const Eigen::Affine3f& trafo);


    /**
    * \brief Smooth contours of polygon.
    *
    * \details Outline of polygon is smoothed.
    * \see smoothGpcStructure()
    */
    void smoothPolygon();


    //###########methods for merging##################
    /**
    * \brief Check for merge candidates.
    *
    * \param[in] poly_vec Vector of Polygons, that are checked.
    * \param[out] intersections Vector with indices of merge candidates.
    */
    virtual void getMergeCandidates(const std::vector<Polygon::Ptr>& poly_vec, std::vector<int>& intersections) const;


    /**
    * \brief Check for intersection of two polygons.
    * \param[in] poly Polygon, this Polygon is checked for intersections with.
    * \return bool True if polygons are intersected.
    */
    virtual bool isIntersectedWith(const Polygon::Ptr& poly) const;


    /**
    * \brief Get intersection of two polygons.
    * \details GPC methods are used to calculate the intersection.
    * \param[in] poly Polygon for calculation of intersection.
    * \param[out] gpc_intersection GPC structure of the intersection of the Polygons.
    */
    void getIntersection(const Polygon::Ptr& poly, gpc_polygon* gpc_intersection) const;


    /**
    * \brief Compute overlap of two Polygons.
    *
    * \details Relative overlap and absolute overlap of two Polygons is computed.
    * \param[in] Polygon, the overlap is computed with.
    * \param[out] Relative overlap of the polygons.
    * \param[out] Absolute overlap of the polygons.
    * \return bool true if  there is some overlap.
    */
    bool getContourOverlap(const Polygon::Ptr& poly, float& rel_overlap, int& abs_overlap) const;


    /**
    * \brief Compute similarity of two polygons.
    *
    * Similarity of Polygons is computed based on parameters and overlap.
    * \param[in] poly Polygon, the similarity is calculated with.
    * \return Similarity of polygons.
    */
    float computeSimilarity(const Polygon::Ptr& poly) const;


    /**
    * \brief Merging of polygons.
    *
    * Complete merge process for this polygon with a vector of merge candidates.
    * Merging is performed relative to an average Polygon.
    * \param[in] poly_vec Vector with merge candidate Polygons
    */
    virtual void merge(std::vector<Polygon::Ptr>& poly_vec);


    /**
    * \brief Calculate merge union of Polygons.
    *
    * The union of contours and the resultant parameters are computed
    * with respect to an average Polygon.Results are stored in Polygon
    * object the method is called for.
    * \param[in] poly_vec Vector of merge candidate Polygons.
    * \param[in] p_average Polygon providing the resultant parameters.
    */
    void merge_union(std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr&  p_average);


    /**
    * \brief Assign merge weight to Polygon.
    *
    * \details Assignment of merge weight to Polygon object depending on weighting method.
    * The weighting method is determined from the merge_config parameters.
    * \see cob_3d_mapping::merge_config
    */
    void assignWeight();


    /**
    * \brief Assign ID to Polygon
    *
    * \details Lowest ID of input vector is assigned to Polygon.
    * \param[in] poly_vec Vector with Polygon pointers.
    */
    void assignID(const std::vector<Polygon::Ptr>& poly_vec);


    /**
    * \brief Average polygon is calculated.
    *
    * Calculation of average polygon based on merge weights of individual polygons.
    * \param[in] poly_vec Polygons, that weighting is performed with
    * \param[out] p_average Resultant polygon.
    * \see cob_3d_mapping::merge_config
    * \see assignWeight()
    */
    virtual void applyWeighting(const std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr& p_average);


    /**
    * \brief Get 2D GPC structure from polygon.
    *
    * \details Transformation from 3D to 2D is performed with given transformation.
    * \param[in] external_trafo Affine transformation.
    * \param[out] Resultant GPC structure.
    */
    void getGpcStructure(const Eigen::Affine3f& external_trafo, gpc_polygon* gpc_p) const;


    /**
    * \brief Conversion from GPC structure to polygon.
    *
    * \details Transformation from 2D GPC structure to 3D polygon object using an external transformation.
    * \param[in] external_trafo from 2D to 3D
    * \param[in] gpc_p Input GPC structure
    */
    void applyGpcStructure(const Eigen::Affine3f& external_trafo, const gpc_polygon* gpc_p);


    /**
    * \brief Check for similar polygon parameters.
    *
    * \details Check is performed based on merge settings of Polygons.
    * \param[in] poly Polygon, whose parameters are compared to this Polygon.
    */
    inline bool hasSimilarParametersWith(const Polygon::Ptr& poly) const
    {
      Eigen::Vector3f d = (this->centroid - poly->centroid).head(3);
      return ( fabs(poly->normal.dot(this->normal)) > this->merge_settings_.angle_thresh &&
               fabs( d.dot(this->normal) ) < this->merge_settings_.d_thresh &&
               fabs( d.dot(poly->normal) ) < this->merge_settings_.d_thresh );
    }

    //#######methods for calculation#####################

    /**
    * \brief Computation of centroid of polygn
    */
    void computeCentroid();


    /**
    * \brief Computation of area of 2D polygon.
    *
    * This method is only suitable for polygons with no expansion
    * in z-direction.
    */
    double computeArea() const;  // http://paulbourke.net/geometry/polyarea/


    /**
    * \brief Computation of area of 3D polygon.
    *
    * This method is suitable for all polygons.
    */
    double computeArea3d() const;


    /**
    * \brief compute Transformation from world coordinate system to coordinate
    * system on polygon plane.
    * \note The coordinate system on the plane is determined by the orgin and the normal,
    * which serves as one axis. The other axes are calculated arbitrarily.
    * \param[in] normal Normal of plane, serves as one axis.
    * \param[in] origin Origin of coordinate system on plane.
    * \param[out] transformation The resultant transformation.
    */
    void getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,const Eigen::Vector3f &origin,
                                           Eigen::Affine3f &transformation) const;


    /**
    * \brief compute Transformation from world coordinate system to coordinate
    * system on polygon plane.
    * \note The coordinate system on the plane is determined by the orgin and the Z-axis.
    * The other axes are calculated arbitrarily.
    * \param[in] z_axis Z-Axis of the coordinate system on the plane.
    * \param[in] origin Origin of coordinate system on plane.
    * \param[out] transformation The resultant transformation.
    */
    void getTransformationFromPlaneToWorld(const Eigen::Vector3f z_axis,const Eigen::Vector3f &normal,
                                           const Eigen::Vector3f &origin, Eigen::Affine3f &transformation);


    /**
    * \brief Transform polygon contours with given transformation and return copy.
    * \param[in] trafo Transformation, which is applied.
    * \param[out] new_contours Transformed copy of the contours of this Polygon.
    */
    void getTransformedContours(const Eigen::Affine3f& trafo, std::vector<std::vector<Eigen::Vector3f> >& new_contours ) const;


    /**
    * \brief Transform polygon contours with given transformation.
    * \param[in] trafo Transforamtion, that is applied to the conoturs of this Polygon.
    */
    void TransformContours(const Eigen::Affine3f& trafo);


    /**
    * \brief Computation of bounding box and pose of structure.
    * \param[out] pose Orientation of bounding box, as an affine transformation matrix.
    * \param[out] min_pt Mininmal boundary of bounding box.
    * \param[out] max_pt Maximal boundary of bounding box.
    */
    void computePoseAndBoundingBox(Eigen::Affine3f& pose, Eigen::Vector4f& min_pt, Eigen::Vector4f& max_pt);



    //#############debugging methods#######################


    /**
    * \brief Output of contour vertices to file.
    * \param[in] name Name of output file
    * \todo Change output path to value on individual system
    */
    void debug_output(std::string name);


    //########## member variables#################
    //needed for 32-bit systems: see http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<std::vector<Eigen::Vector3f> > contours;/**< Contours of polygon sturcure. */
    Eigen::Vector3f normal;/**< Normal of polygons structure. */
    double d;
    Eigen::Affine3f transform_from_world_to_plane;/**< Transformation from world coordinate system to cvoordinate system on polygon. */
    std::vector<bool> holes;/**< Bool vector indicating holes in polygon structure*/
    double merge_weight_;/**< Merge weight of polygon - regulating its influence in merge processes.*/
    merge_config merge_settings_;/** Merge settings - configuring assignment of weight and merge processes*/
    DominantColor d_color_;

  };
}
#endif /* POLYGON_H_ */
