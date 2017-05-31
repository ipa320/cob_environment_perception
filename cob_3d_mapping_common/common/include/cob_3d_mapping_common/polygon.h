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

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cob_3d_mapping_common/shape.h"
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
extern "C" {
#include "libgpc/gpc.h"
}
#include <libpolypartition/polypartition.h>
#include <cob_3d_mapping_common/dominant_color.h>
#include <fstream>
#include <iostream>

//#include "cob_3d_mapping_common/stop_watch.h"

namespace cob_3d_mapping
{
  /**
  * \brief Struct with merge configuration
  * \note Members of struct determine merge thresholds
  * and the weighting method , which is applied.
  */
  struct MergeConfig
  {
    MergeConfig()
    : angle_thresh(0.97),
      d_thresh(0.1),
      weighting_method("AREA")
    {};

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

  /**
  * \brief Constructor for Polygon object.
  */
    Polygon()
      : Shape(),
        normal_(Eigen::Vector3f::Zero()),
        d_(0.0),
        merge_weight_(1.0)
    { }

    Polygon(unsigned int id,
            Eigen::Vector3f normal,
            Eigen::Vector3f centroid,
            std::vector<std::vector<Eigen::Vector3f> >& contours_3d,
            std::vector<bool> holes,
            std::vector<float> color);

    Polygon(unsigned int id,
            Eigen::Vector3f normal,
            Eigen::Vector3f centroid,
            std::vector<pcl::PointCloud<pcl::PointXYZ> >& contours_3d,
            std::vector<bool> holes,
            std::vector<float> color);

    //Polygon(Polygon::Ptr polygon);


    /**
    * \brief Get index of first non-hole contour.
    * \return Index of first non-hole contour.
    */
    inline size_t outerContourIndex() const
    {
      for(size_t i = 0; i < contours_.size(); ++i) { if(!holes_[i]) { return i; } };
      return 0;
    }

    virtual void setContours3D(std::vector<std::vector<Eigen::Vector3f> >& contours_3d);
    virtual std::vector<std::vector<Eigen::Vector3f> > getContours3D() const;


    void setContours2D(const std::vector<std::vector<Eigen::Vector2f> >& contours_2d) {contours_ = contours_2d;};
    const std::vector<std::vector<Eigen::Vector2f> > &getContours2D() const {return contours_;};

    //##########methods for instantiation##############

    /**
    * \brief Compute attributes for Polygon.
    *
    * Set of attributes is completed with respect to the input parameters.
    * \param[in] new_normal Normal of Polygon.
    * \param[in] new_centroid Centroid of Polygon.
    */
    virtual void updateAttributes(const Eigen::Vector3f &new_normal, const Eigen::Vector3f & new_centroid);


    /**
    * \brief Transform polygon to target coordinate system.
    *
    * Polygon is transformed with input transformation.
    * Operation is applied to both parameters and contour points.
    * \param[in] trafo Affine transformation to target coordinate system
    */
    virtual void transform(const Eigen::Affine3f& trafo);
    //virtual void transform2tf(const Eigen::Affine3f& trafo);


    /**
    * \brief Smooth contours of polygon.
    *
    * \details Outline of polygon is smoothed.
    * \see smoothGpcStructure()
    */
    void smoothContours();


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

	float getOverlap(const Polygon& poly);

    /**
    * \brief Merging of polygons.
    *
    * Complete merge process for this polygon with a vector of merge candidates.
    * Merging is performed relative to an average Polygon.
    * \param[in] poly_vec Vector with merge candidate Polygons
    */
    virtual void merge(std::vector<Polygon::Ptr>& poly_vec);


    void mergeDifference(Polygon::Ptr& p_merge);

    virtual void projectContour(const Polygon& p, std::vector<std::vector<Eigen::Vector2f> >& contours) const;

    /**
    * \brief Assign merge weight to Polygon.
    *
    * \details Assignment of merge weight to Polygon object depending on weighting method.
    * The weighting method is determined from the merge_config parameters.
    * \see cob_3d_mapping::merge_config
    */
    void assignWeight();


    virtual void setParamsFrom(Polygon::Ptr& p);


    /**
    * \brief Check for similar polygon parameters.
    *
    * \details Check is performed based on merge settings of Polygons.
    * \param[in] poly Polygon, whose parameters are compared to this Polygon.
    */
    inline bool hasSimilarParametersWith(const Polygon::Ptr& poly) const
    {
      Eigen::Vector3f d = (this->pose_.translation() - poly->pose_.translation());
      const float fact = computeCentroid().squaredNorm()/(1.8f*1.8f);
      const float fact2 = 0.5f + 0.5f*(1-std::max(std::abs(Eigen::Vector3f::UnitZ().dot(this->normal_)), std::abs(Eigen::Vector3f::UnitZ().dot(poly->normal_))));
      return ( fabs(poly->normal_.dot(this->normal_)) > this->merge_settings_.angle_thresh*fact2 &&
               fabs( d.dot(this->normal_) ) < this->merge_settings_.d_thresh*fact &&
               fabs( d.dot(poly->normal_) ) < this->merge_settings_.d_thresh*fact );
    }
    
    inline bool hasSimilarColorWith(const Polygon::Ptr& poly) const
    {
	  if(color_.size() == poly->color_.size()) {
		  float cd=0;
		  for(size_t i=0; i<color_.size(); i++) cd += std::pow(color_[i]-poly->color_[i],2);
		  if(cd>0.15f) return false;
	  }
      return true;
    }

    //#######methods for calculation#####################


    //void computePose();

    void computePose(std::vector<std::vector<Eigen::Vector3f> >& contours_3d);

    /**
    * \brief Computation of centroid of polygn
    */
    Eigen::Vector3f computeCentroid() const;

    Eigen::Vector3f computeCentroid(std::vector<std::vector<Eigen::Vector3f> >& contours_3d) const;


    /**
    * \brief Computation of area of 2D polygon.
    *
    * This method is only suitable for polygons with no expansion
    * in z-direction.
    */
    //double computeArea() const;  // http://paulbourke.net/geometry/polyarea/


    /**
    * \brief Computation of area of 3D polygon.
    *
    * This method is suitable for all polygons.
    */
    double computeArea3d() const;


    virtual void triangulate(std::list<TPPLPoly>& tri_list) const;

    /**
    * \brief Computation of bounding box and pose of structure.
    * \param[out] pose Orientation of bounding box, as an affine transformation matrix.
    * \param[out] min_pt Minimal boundary of bounding box.
    * \param[out] max_pt Maximal boundary of bounding box.
    */
    //void computePoseAndBoundingBox(Eigen::Affine3f& pose, Eigen::Vector4f& min_pt, Eigen::Vector4f& max_pt);
    
    inline Eigen::Vector3f operator[](const Eigen::Vector2f &p) const {
        return pose_*Eigen::Vector3f(p(0), p(1), 0);
	}



    //#############debugging methods#######################


    /**
    * \brief Output of contour vertices to file.
    * \param[in] name Name of output file
    * \todo Change output path to value on individual system
    */
    void debugOutput(std::string name);


    //########## member variables#################
    //needed for 32-bit systems: see http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<std::vector<Eigen::Vector2f> > contours_;/**< Contours of polygon structure. */
    Eigen::Vector3f normal_;/**< Normal of polygons structure. */
    double d_;
    //Eigen::Affine3f transform_from_world_to_plane;/**< Transformation from world coordinate system to coordinate system on polygon. */
    std::vector<bool> holes_;/**< Bool vector indicating holes in polygon structure*/
    double merge_weight_;/**< Merge weight of polygon - regulating its influence in merge processes.*/
    MergeConfig merge_settings_;/** Merge settings - configuring assignment of weight and merge processes*/

  protected:
    /**
    * \brief Average polygon is calculated.
    *
    * Calculation of average polygon based on merge weights of individual polygons.
    * \param[in] poly_vec Polygons, that weighting is performed with
    * \param[out] p_average Resultant polygon.
    * \see cob_3d_mapping::merge_config
    * \see assignWeight()
    */
    virtual void computeAverage(const std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr& p_average);

    /**
    * \brief Get intersection of two polygons.
    * \details GPC methods are used to calculate the intersection.
    * \param[in] poly Polygon for calculation of intersection.
    * \param[out] gpc_intersection GPC structure of the intersection of the Polygons.
    */
    void getIntersection(const Polygon::Ptr& poly, gpc_polygon* gpc_intersection) const;

    /**
    * \brief Calculate merge union of Polygons.
    *
    * The union of contours and the resultant parameters are computed
    * with respect to an average Polygon.Results are stored in Polygon
    * object the method is called for.
    * \param[in] poly_vec Vector of merge candidate Polygons.
    * \param[in] p_average Polygon providing the resultant parameters.
    */
    void mergeUnion(std::vector<Polygon::Ptr>& poly_vec, Polygon::Ptr&  p_average);

    /**
    * \brief Get 2D GPC structure from polygon.
    *
    * \details Transformation from 3D to 2D is performed with given transformation.
    * \param[in] external_trafo Affine transformation.
    * \param[out] Resultant GPC structure.
    */
    void getGpcStructure(gpc_polygon* gpc_p, const std::vector<std::vector<Eigen::Vector2f> >& contours) const;


    /**
    * \brief Conversion from GPC structure to polygon.
    *
    * \details Transformation from 2D GPC structure to 3D polygon object using an external transformation.
    * \param[in] external_trafo from 2D to 3D
    * \param[in] gpc_p Input GPC structure
    */
    void applyGpcStructure(const gpc_polygon* gpc_p);

    /**
    * \brief Assign ID to Polygon
    *
    * \details Lowest ID of input vector is assigned to Polygon.
    * \param[in] poly_vec Vector with Polygon pointers.
    */
    void assignID(const std::vector<Polygon::Ptr>& poly_vec);

    //DominantColor d_color_;


  };
}
