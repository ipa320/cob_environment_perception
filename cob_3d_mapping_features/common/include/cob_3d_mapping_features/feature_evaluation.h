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
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
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

#ifndef COB_3D_MAPPING_FEATURES_FEATURE_EVALUATION_H_
#define COB_3D_MAPPING_FEATURES_FEATURE_EVALUATION_H_

#include <iostream>
#include <string>

#define RES_RSD  0
#define RES_PC   1
#define RES_FPFH 2

#define NUM_LBL 4

namespace cob_3d_mapping_features
{
  /*! @brief Stores the evaluation results */
  class EvalResults
  {
  private:
    /*! @brief used algorithm identifier */
    char alg_type_;

    /*! @brief number of expected positives per label */
    int ref_ [NUM_LBL];

    /*! @brief number of predicted false positves */
    int fp_ [NUM_LBL];

    /*! @brief number of predicted false negatives */
    int fn_ [NUM_LBL]; 

    /*! @brief number of predicted true positives */
    int tp_ [NUM_LBL];

    /*! @brief number of predicted true negatives */
    int tn_ [NUM_LBL];

    /*! @brief points identified correctly as not edge and not plane */
    int tp_not_edge_not_plane_;

    /*! @brief number of undefined points */
    int undef_;

    /*! @brief all labeled points in the point cloud */
    int all_; // all reference points

  public:
    /*! Constructor with specified algorithm type */
    EvalResults(char alg_type) : alg_type_(alg_type), undef_(0), 
      t_normal_(0.0), t_feature_(0.0), t_classifier_(0.0), points_(0)
    {
      // initialize arrays:
      for (size_t i = 0; i < NUM_LBL; ++i)
      {
	ref_[i] = 0;
	fp_[i] = 0;
	fn_[i] = 0;
	tp_[i] = -1;
	tn_[i] = -1;
      }
      tp_not_edge_not_plane_ = 0;
    }

    /*! @brief increment expected positives of @a label */
    void incRef(const int &label) { ++ref_[label]; }

    /*! @brief increment predicted false positives of @a label */
    void incFP(const int &label) { ++fp_[label]; }

    /*! @brief increment predicted false negatives of @a label */
    void incFN(const int &label) { ++fn_[label]; }

    /*! @brief increment predicted true positives of the curved label */
    void incTPnEnP() { ++tp_not_edge_not_plane_; }

    /*! @brief increment undefined points */
    void incUndef() { ++undef_; }

    /*! @brief return expected positives counter of @a label */
    int getRef(const int &label) {return (ref_[label]);}

    /*! @brief return predicted false positives counter of @a label */
    int getFP(const int &label) {return (fp_[label]);}

    /*! @brief return predicted false negative counter of @a label */
    int getFN(const int &label) {return (fn_[label]);}

    /*! @brief return undefined points counter */
    int getUndef() {return (undef_);}

    /*! @brief return predicted true positives counter of @a label */
    int getTP(const int &label) {return (tp_[label]);}

    /*! @brief return predicted true negative counter of @a label */
    int getTN(const int &label) {return (tn_[label]);}

    /*! @brief return counter of all labeled points */
    int getAll() {return (all_);}

    /*! @brief return predicted true positives of the curved label */
    int getTPnEnP() {return tp_not_edge_not_plane_;}

    /*! @brief calculates values of TP and TN depending on the counted FP and FN */
    void updateValues();

    /*! @brief returns the precision (true positive ratio) of @a label in percent */
    float calcTruePosAcc(const int &label);

    /*! @brief returns the true negative ratio of @a label in percent */
    float calcTrueNegAcc(const int &label);

    float calcFalseNegAcc(const int &label);
    float calcFalsePosAcc(const int &label);

    /*! @brief returns the proportion in percent of a @a label in relation to all points */
    float calcProportion(const int &label);

    /*! @brief returns the allover accuracy of all four classes */
    float calcAllAcc();

    /*! @brief returns the allover accuracy of the three classes (plane, edge, curved)*/
    float calcEdgePlaneAcc();

    /*!  @brief prints the formated results to console */
    void printAllToConsole(const std::string &prefix="");

    /*!  @brief prints the formated results of a single label to console */
    void printToConsole(const int &label, const std::string &prefix="");

    /*! @brief writes the logfile for the time measurements */
    void writeTimerLog(const std::string &file);

    std::string getLabel();

    int points_;

    /*! @brief time measured for normal estimation */
    double t_normal_;

    /*! @brief time measured for feature estimation */
    double t_feature_;

    /*! @brief time measured for the classifier */
    double t_classifier_;
  };
}

// The following global variables hold the boost::program_option option:
bool vis_enable_;
float pass_depth_;

bool rsd_enable_;          // enable RSD estimation
bool rsd_mls_enable_;      // enable surface smoothing
bool rsd_vox_enable_;      // enable voxel grid filtering
float rsd_vox_;            // voxel size
float rsd_normal_r_;       // normal search radius
float rsd_r_max_;          // r_max_ limit
float rsd_r_;              // search radius
float rsd_r_min_th_upper_; // min value to determine planes
float rsd_r_min_th_cylinder_lower_; // max value to determine edges
float rsd_r_min_th_sphere_lower_;
float rsd_max_min_ratio_;    // threshold between sphere and cylinder (r_max = x * r_min)
//float rsd_r_max_offset_;

bool pc_enable_;           // enable principal curvature estimation
bool pc_mls_enable_;       // enable surface smoothing
bool pc_vox_enable_;       // enable voxel grid filtering
float pc_vox_;             // voxel size
float pc_normal_r_;        // normal search radius
float pc_r_;               // search radius
float pc_c_max_th_cylinder_upper_; // min value to determine edges
float pc_c_max_th_sphere_upper_; // min value to determine edges
float pc_c_max_th_lower_; // max value to determine planes
float pc_max_min_ratio_;   // threshold between sphere and cylinder (r_max = x * r_min)
//float pc_c_min_offset_;

bool fpfh_enable_;           // enable FPFH estimation
bool fpfh_mls_enable_;       // enable surface smoothing
bool fpfh_vox_enable_;       // enable voxel grid filtering
float fpfh_vox_;             // voxel size
float fpfh_normal_r_;        // normal search radius
float fpfh_r_;               // search radius
int fpfh_knn_k_;             // define k for knn classifier

#endif
